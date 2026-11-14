#include "sync.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace mocap {

const char* syncModeName(int mode) {
    switch (mode) {
        case kSyncModeSoftware: return "software";
        case kSyncModeHardware: return "hardware";
        case kSyncModeNone:     return "none";
        default:                return "unknown";
    }
}

static void appendf(std::string& s, const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) s.append(buf, std::min(size_t(n), sizeof(buf) - 1));
}

// JSON has no NaN or Inf; a non-finite value is written null rather than
// as something a reader would choke on or take for a number.
static void num(std::string& s, double v, int decimals) {
    if (std::isfinite(v)) appendf(s, "%.*f", decimals, v);
    else s += "null";
}

static void frameJson(std::string& s, const SyncFrame& f, bool unexpected) {
    appendf(s, "{\"serial\":%d,\"present\":true,", f.serial);
    if (unexpected) s += "\"unexpected\":true,";
    appendf(s, "\"frame_id\":%d,\"object\":%s,\"invalid\":%s,\"ts\":", f.frameId,
            f.object ? "true" : "false", f.invalid ? "true" : "false");
    num(s, f.timeStamp, 6);
    appendf(s, ",\"hw_valid\":%s,\"hw_ts\":", f.hwStampValid ? "true" : "false");
    num(s, f.hwTimeStamp, 9);
    appendf(s, ",\"hw_ticks\":%llu,\"hw_freq\":%u,\"hw_master\":%s,\"sync_video\":%s,"
               "\"hw_timing\":%s,\"blobs\":[",
            (unsigned long long)f.hwTicks, f.hwFreq, f.hwMaster ? "true" : "false",
            f.syncVideo ? "true" : "false", f.hwTiming ? "true" : "false");
    for (size_t i = 0; i < f.blobs.size(); ++i) {
        const Blob& b = f.blobs[i];
        s += i ? ",[" : "[";
        num(s, b.x, 3); s += ",";
        num(s, b.y, 3); s += ",";
        num(s, b.area, 2); s += ",";
        num(s, b.roundness, 4);
        s += "]";
    }
    s += "]}";
}

std::string groupJson(const SyncGroup& g, const std::vector<int>& expected) {
    std::string s;
    s.reserve(256 + 220 * expected.size());
    appendf(s, "{\"g\":%ld,\"host_mono_ns\":%lld,\"group_frame_id\":%d,\"mode\":\"%s\",\"group_ts\":",
            g.index, (long long)g.hostMonoNs, g.frameId, syncModeName(g.mode));
    num(s, g.timeStamp, 6);
    s += ",\"ts_spread\":";   num(s, g.timeSpread, 9);
    s += ",\"ts_earliest\":"; num(s, g.earliest, 6);
    s += ",\"ts_latest\":";   num(s, g.latest, 6);
    s += ",\"sdk_dropped\":[";
    for (size_t i = 0; i < g.sdkDropped.size(); ++i) appendf(s, i ? ",%d" : "%d", g.sdkDropped[i]);
    s += "],\"cams\":[";
    bool first = true;
    for (int serial : expected) {
        if (!first) s += ",";
        first = false;
        const SyncFrame* hit = nullptr;
        for (auto& f : g.frames) if (f.serial == serial) { hit = &f; break; }
        if (hit) frameJson(s, *hit, false);
        else appendf(s, "{\"serial\":%d,\"present\":false}", serial);
    }
    for (auto& f : g.frames) {
        if (std::find(expected.begin(), expected.end(), f.serial) != expected.end()) continue;
        if (!first) s += ",";
        first = false;
        frameJson(s, f, true);
    }
    s += "]}";
    return s;
}

// ---------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------

SyncStats::SyncStats(std::vector<int> expected) : expected_(std::move(expected)) {
    for (int s : expected_) { PerCamera c; c.serial = s; cams_.push_back(c); }
    byPresent_.assign(expected_.size() + 1, 0);
    hwX_.resize(cams_.size()); hwD_.resize(cams_.size()); hwIv_.resize(cams_.size());
    hwPrev_.assign(cams_.size(), 0.0);
    hwPrevId_.assign(cams_.size(), -1);
}

SyncStats::Dist SyncStats::dist(std::vector<double> v) {
    Dist d;
    if (v.empty()) return d;
    std::sort(v.begin(), v.end());
    // Nearest-rank on the sorted values: an actual observation, no interpolation.
    auto at = [&](double q) { return v[std::min(v.size() - 1, size_t(std::ceil(q * v.size())) - (q > 0 ? 1 : 0))]; };
    d.n = long(v.size());
    d.min = v.front();
    d.median = at(0.5);
    d.p95 = at(0.95);
    d.max = v.back();
    return d;
}

void SyncStats::add(const SyncGroup& g) {
    // Host and group-timestamp intervals, group frame ID steps.
    if (groups_ == 0) {
        firstNs_ = g.hostMonoNs; firstTs_ = g.timeStamp; firstGid_ = g.frameId;
    } else {
        hostMs_.push_back((g.hostMonoNs - lastNs_) / 1e6);
        tsMs_.push_back((g.timeStamp - lastTs_) * 1e3);
        int d = g.frameId - lastGid_;
        if (d > 1) { ++gidGaps_; gidSkipped_ += d - 1; }
        else if (d <= 0) ++gidNotInc_;
    }
    lastNs_ = g.hostMonoNs; lastTs_ = g.timeStamp; lastGid_ = g.frameId;
    ++groups_;
    if (g.mode >= 0 && g.mode <= 2) ++mode_[g.mode];
    sdkSpreadUs_.push_back(g.timeSpread * 1e6);

    size_t present = 0;
    int idMin = 0, idMax = 0, nId = 0;
    double tsMin = 0, tsMax = 0;
    int nTs = 0;
    std::vector<bool> seen(cams_.size(), false);
    std::vector<const SyncFrame*> stamped(cams_.size(), nullptr);

    for (auto& f : g.frames) {
        size_t k = 0;
        while (k < cams_.size() && cams_[k].serial != f.serial) ++k;
        if (k == cams_.size()) { ++unexpected_; continue; }
        if (seen[k]) continue;       // a second frame from one camera in a group: first wins
        seen[k] = true;
        PerCamera& c = cams_[k];
        ++present;
        ++c.present;
        if (!f.object) ++c.wrongMode;
        if (f.hwStampValid) ++c.hwValid;
        if (f.syncVideo) ++c.syncVideo;
        if (f.hwTiming) ++c.hwTiming;
        if (f.hwMaster) ++c.hwMaster;
        if (f.hwStampValid) {
            stamped[k] = &f;
            // Only across consecutive frame IDs: a gap is counted as a gap,
            // not as a long interval.
            if (hwPrevId_[k] >= 0 && f.frameId - hwPrevId_[k] == 1)
                hwIv_[k].push_back((f.hwTimeStamp - hwPrev_[k]) * 1e3);
            hwPrev_[k] = f.hwTimeStamp;
            hwPrevId_[k] = f.frameId;
        }
        if (!f.blobs.empty()) { ++c.framesWithBlobs; c.blobs += long(f.blobs.size()); }

        if (c.present > 1) {             // not this camera's first frame
            int d = f.frameId - c.lastId;
            if (d > 1) { ++c.idGaps; c.idSkipped += d - 1; }
            else if (d <= 0) ++c.idNotIncreasing;
        } else {
            c.firstId = f.frameId;
        }
        c.lastId = f.frameId;

        int off = f.frameId - g.frameId;
        if (!c.anyOffset) { c.anyOffset = true; c.offsetMin = c.offsetMax = off; }
        else { c.offsetMin = std::min(c.offsetMin, off); c.offsetMax = std::max(c.offsetMax, off); }

        if (nId++ == 0) idMin = idMax = f.frameId;
        else { idMin = std::min(idMin, f.frameId); idMax = std::max(idMax, f.frameId); }
        if (nTs++ == 0) tsMin = tsMax = f.timeStamp;
        else { tsMin = std::min(tsMin, f.timeStamp); tsMax = std::max(tsMax, f.timeStamp); }
    }

    // The reference is fixed by the first group that has any stamp: the
    // camera the SDK flags as master timing device, else the first stamped.
    if (hwRef_ < 0) {
        for (size_t k = 0; k < cams_.size() && hwRef_ < 0; ++k)
            if (stamped[k] && stamped[k]->hwMaster) hwRef_ = int(k);
        for (size_t k = 0; k < cams_.size() && hwRef_ < 0; ++k)
            if (stamped[k]) hwRef_ = int(k);
    }
    if (hwRef_ >= 0 && stamped[hwRef_]) {
        const double ref = stamped[hwRef_]->hwTimeStamp;
        for (size_t k = 0; k < cams_.size(); ++k) {
            if (!stamped[k]) continue;
            hwX_[k].push_back(ref);
            hwD_[k].push_back(stamped[k]->hwTimeStamp - ref);
        }
    }
    for (size_t k = 0; k < cams_.size(); ++k) if (!seen[k]) ++cams_[k].missing;
    for (int s : g.sdkDropped)
        for (auto& c : cams_) if (c.serial == s) ++c.sdkDropped;

    ++byPresent_[present];
    if (present == cams_.size() && !cams_.empty()) ++complete_;
    if (nId >= 2) {
        ++idSpreadGroups_;
        if (idMax != idMin) ++idSpreadNonZero_;
        idSpreadMax_ = std::max(idSpreadMax_, idMax - idMin);
    }
    if (nTs >= 2) tsSpreadUs_.push_back((tsMax - tsMin) * 1e6);
}

SyncStats::Summary SyncStats::summary() const {
    Summary s;
    s.groups = groups_;
    s.complete = complete_;
    s.byPresent = byPresent_;
    s.unexpectedFrames = unexpected_;
    s.modeNone = mode_[0]; s.modeSoftware = mode_[1]; s.modeHardware = mode_[2];
    if (groups_ >= 2) {
        s.hostSpanS = (lastNs_ - firstNs_) / 1e9;
        s.groupTsSpanS = lastTs_ - firstTs_;
        if (s.hostSpanS > 0) s.hostRateHz = (groups_ - 1) / s.hostSpanS;
        if (s.groupTsSpanS > 0) s.groupTsRateHz = (groups_ - 1) / s.groupTsSpanS;
    }
    s.hostIntervalMs = dist(hostMs_);
    s.groupTsIntervalMs = dist(tsMs_);
    s.groupIdGaps = gidGaps_; s.groupIdSkipped = gidSkipped_; s.groupIdNotIncreasing = gidNotInc_;
    s.groupIdFirst = firstGid_; s.groupIdLast = lastGid_;
    s.idSpreadGroups = idSpreadGroups_; s.idSpreadNonZero = idSpreadNonZero_;
    s.idSpreadMax = idSpreadMax_;
    s.tsSpreadUs = dist(tsSpreadUs_);
    s.sdkSpreadUs = dist(sdkSpreadUs_);
    s.cameras = cams_;
    s.hwReference = hwRef_ >= 0 ? cams_[hwRef_].serial : 0;
    for (size_t k = 0; k < s.cameras.size(); ++k) {
        PerCamera& c = s.cameras[k];
        c.hwIntervalMs = dist(hwIv_[k]);
        const auto& x = hwX_[k];
        const auto& d = hwD_[k];
        c.hwPairs = long(d.size());
        if (d.empty()) continue;
        // Offsets are seconds apart and wander by microseconds: subtract the
        // first values before summing so the sums keep their precision.
        const double x0 = x.front(), d0 = d.front();
        double sx = 0, sd = 0, sxx = 0, sxd = 0, lo = 0, hi = 0;
        for (size_t i = 0; i < d.size(); ++i) {
            double a = x[i] - x0, b = d[i] - d0;
            sx += a; sd += b; sxx += a * a; sxd += a * b;
            lo = std::min(lo, b); hi = std::max(hi, b);
        }
        const double n = double(d.size());
        c.hwOffsetMeanS = d0 + sd / n;
        c.hwOffsetRangeUs = (hi - lo) * 1e6;
        const double den = n * sxx - sx * sx;
        const double slope = den > 0 ? (n * sxd - sx * sd) / den : 0.0;
        const double icpt = (sd - slope * sx) / n;
        double ss = 0;
        for (size_t i = 0; i < d.size(); ++i) {
            double r = (d[i] - d0) - (icpt + slope * (x[i] - x0));
            ss += r * r;
        }
        c.hwOffsetSlopePpm = slope * 1e6;
        c.hwOffsetResidUs = std::sqrt(ss / n) * 1e6;
    }
    return s;
}

static double pct(long a, long b) { return b ? 100.0 * a / b : 0.0; }

std::string SyncStats::text(double requestedHz) const {
    Summary s = summary();
    std::string o;
    const long n = s.groups, C = long(expected_.size());
    appendf(o, "  groups            %ld over %.3f s (host clock, first to last)\n", n, s.hostSpanS);
    appendf(o, "  complete          %ld of %ld (%.2f%%) have all %ld cameras\n",
            s.complete, n, pct(s.complete, n), C);
    o += "  cameras present  ";
    for (size_t k = 0; k < s.byPresent.size(); ++k)
        if (s.byPresent[k]) appendf(o, " %zu:%ld", k, s.byPresent[k]);
    o += "   (cameras:groups)\n";
    appendf(o, "  group mode        hardware %ld  software %ld  none %ld   (FrameGroup::Mode())\n",
            s.modeHardware, s.modeSoftware, s.modeNone);
    appendf(o, "  group rate        %.3f Hz by host clock, %.3f Hz by group timestamp; requested %g Hz\n",
            s.hostRateHz, s.groupTsRateHz, requestedHz);
    auto d = [&](const char* name, const Dist& x, const char* unit) {
        if (x.n) appendf(o, "  %-17s min %.3f  median %.3f  p95 %.3f  max %.3f %s  (n %ld)\n",
                         name, x.min, x.median, x.p95, x.max, unit, x.n);
        else appendf(o, "  %-17s no data\n", name);
    };
    d("host interval", s.hostIntervalMs, "ms");
    d("group ts interval", s.groupTsIntervalMs, "ms");
    appendf(o, "  group frame ID    %d .. %d; %ld gaps skipping %ld IDs; %ld steps not increasing\n",
            s.groupIdFirst, s.groupIdLast, s.groupIdGaps, s.groupIdSkipped, s.groupIdNotIncreasing);
    appendf(o, "  frame ID spread   %ld of %ld groups hold differing frame IDs; largest spread %d\n",
            s.idSpreadNonZero, s.idSpreadGroups, s.idSpreadMax);
    d("ts spread", s.tsSpreadUs, "us  [max-min of Frame::TimeStamp() in a group]");
    d("SDK ts spread", s.sdkSpreadUs, "us  [FrameGroup::TimeSpread()]");
    if (s.unexpectedFrames) appendf(o, "  UNEXPECTED        %ld frames from serials not opened\n", s.unexpectedFrames);
    o += "\n  serial  present  missing  not-obj  sdk-drop  id-gaps  id-skipped  id-not-inc  "
         "id-offset  hw-stamp  with-blobs   blobs\n";
    for (auto& c : s.cameras) {
        char off[32] = "-";
        if (c.anyOffset) snprintf(off, sizeof(off), "%d..%d", c.offsetMin, c.offsetMax);
        appendf(o, "  %6d  %7ld  %7ld  %7ld  %8ld  %7ld  %10ld  %10ld  %9s  %8ld  %10ld  %6ld\n",
                c.serial, c.present, c.missing, c.wrongMode, c.sdkDropped, c.idGaps, c.idSkipped,
                c.idNotIncreasing, off, c.hwValid, c.framesWithBlobs, c.blobs);
    }

    if (s.hwReference) {
        appendf(o, "\n  hardware stamps, each camera's own clock; offsets are to %d in the same group\n",
                s.hwReference);
        o += "  serial  master  interval ms: min     median        max    offset s (mean)  range us  "
             "drift ppm  rms about drift us\n";
        for (auto& c : s.cameras) {
            if (!c.hwPairs && !c.hwIntervalMs.n) { appendf(o, "  %6d  no hardware stamps\n", c.serial); continue; }
            appendf(o, "  %6d  %6s  %16.4f  %9.4f  %9.4f  %17.6f  %8.1f  %9.3f  %18.2f\n", c.serial,
                    c.hwMaster ? "yes" : "", c.hwIntervalMs.min, c.hwIntervalMs.median, c.hwIntervalMs.max,
                    c.hwOffsetMeanS, c.hwOffsetRangeUs, c.hwOffsetSlopePpm, c.hwOffsetResidUs);
        }
    } else {
        o += "\n  hardware stamps   none on any frame\n";
    }
    return o;
}

static void distJson(std::string& o, const char* key, const SyncStats::Dist& x) {
    appendf(o, "\"%s\":{\"n\":%ld,\"min\":", key, x.n);
    num(o, x.min, 4); o += ",\"median\":"; num(o, x.median, 4);
    o += ",\"p95\":"; num(o, x.p95, 4); o += ",\"max\":"; num(o, x.max, 4);
    o += "}";
}

std::string SyncStats::json(double requestedHz) const {
    Summary s = summary();
    std::string o = "{";
    appendf(o, "\"groups\":%ld,\"complete\":%ld,\"complete_fraction\":", s.groups, s.complete);
    num(o, s.groups ? double(s.complete) / s.groups : 0.0, 6);
    o += ",\"groups_by_cameras_present\":[";
    for (size_t k = 0; k < s.byPresent.size(); ++k) appendf(o, k ? ",%ld" : "%ld", s.byPresent[k]);
    appendf(o, "],\"unexpected_frames\":%ld,\"mode\":{\"hardware\":%ld,\"software\":%ld,\"none\":%ld},",
            s.unexpectedFrames, s.modeHardware, s.modeSoftware, s.modeNone);
    o += "\"requested_hz\":"; num(o, requestedHz, 3);
    o += ",\"host_span_s\":"; num(o, s.hostSpanS, 6);
    o += ",\"host_rate_hz\":"; num(o, s.hostRateHz, 4);
    o += ",\"group_ts_span_s\":"; num(o, s.groupTsSpanS, 6);
    o += ",\"group_ts_rate_hz\":"; num(o, s.groupTsRateHz, 4);
    o += ","; distJson(o, "host_interval_ms", s.hostIntervalMs);
    o += ","; distJson(o, "group_ts_interval_ms", s.groupTsIntervalMs);
    appendf(o, ",\"group_frame_id\":{\"first\":%d,\"last\":%d,\"gaps\":%ld,\"skipped\":%ld,"
               "\"not_increasing\":%ld}",
            s.groupIdFirst, s.groupIdLast, s.groupIdGaps, s.groupIdSkipped, s.groupIdNotIncreasing);
    appendf(o, ",\"frame_id_spread\":{\"groups_compared\":%ld,\"groups_nonzero\":%ld,\"max\":%d}",
            s.idSpreadGroups, s.idSpreadNonZero, s.idSpreadMax);
    o += ","; distJson(o, "ts_spread_us", s.tsSpreadUs);
    appendf(o, ",\"hw_reference_serial\":%d", s.hwReference);
    o += ","; distJson(o, "sdk_ts_spread_us", s.sdkSpreadUs);
    o += ",\"cameras\":[";
    for (size_t k = 0; k < s.cameras.size(); ++k) {
        auto& c = s.cameras[k];
        appendf(o, "%s{\"serial\":%d,\"present\":%ld,\"missing\":%ld,\"not_object\":%ld,"
                   "\"sdk_dropped\":%ld,\"id_gaps\":%ld,\"id_skipped\":%ld,\"id_not_increasing\":%ld,"
                   "\"first_id\":%d,\"last_id\":%d,",
                k ? "," : "", c.serial, c.present, c.missing, c.wrongMode, c.sdkDropped, c.idGaps,
                c.idSkipped, c.idNotIncreasing, c.firstId, c.lastId);
        if (c.anyOffset) appendf(o, "\"id_offset_min\":%d,\"id_offset_max\":%d,", c.offsetMin, c.offsetMax);
        else o += "\"id_offset_min\":null,\"id_offset_max\":null,";
        appendf(o, "\"hw_stamp_frames\":%ld,\"sync_video_frames\":%ld,\"hw_timing_frames\":%ld,"
                   "\"hw_master_frames\":%ld,\"frames_with_blobs\":%ld,\"blobs\":%ld,",
                c.hwValid, c.syncVideo, c.hwTiming, c.hwMaster, c.framesWithBlobs, c.blobs);
        distJson(o, "hw_interval_ms", c.hwIntervalMs);
        appendf(o, ",\"hw_offset\":{\"pairs\":%ld,\"mean_s\":", c.hwPairs);
        num(o, c.hwOffsetMeanS, 9);
        o += ",\"range_us\":"; num(o, c.hwOffsetRangeUs, 3);
        o += ",\"drift_ppm\":"; num(o, c.hwOffsetSlopePpm, 4);
        o += ",\"rms_about_drift_us\":"; num(o, c.hwOffsetResidUs, 3);
        o += "}}";
    }
    o += "]}";
    return o;
}

}  // namespace mocap
