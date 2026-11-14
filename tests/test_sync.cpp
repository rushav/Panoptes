// Unit tests for lib/sync (group records, groups.jsonl lines, statistics)
// and lib/maskfile (reading what 07_mask wrote). No camera, no SDK: the
// groups are synthetic.
//
// Run from the repo root:  tests/build/test_sync [config_dir]
// Exit 0 only if every check passes.

#include "../lib/maskfile.h"
#include "../lib/sync.h"

#include <opencv2/core.hpp>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <unistd.h>

using namespace mocap;
namespace fs = std::filesystem;

static int passed = 0, failed = 0;

static void check(bool ok, const std::string& what) {
    printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
    (ok ? passed : failed)++;
}

static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

static const std::vector<int> kSerials = {33275, 33276, 33277, 33659};
static const double kHz = 30.0;

// A frame as an ideal rig would deliver it: every camera carries the
// group's frame ID and timestamp.
static SyncFrame frame(int serial, int id, double ts) {
    SyncFrame f;
    f.serial = serial; f.frameId = id; f.object = true; f.timeStamp = ts;
    return f;
}

static SyncGroup group(long k, int firstId = 1000) {
    SyncGroup g;
    g.index = k;
    g.frameId = firstId + int(k);
    g.timeStamp = 5.0 + k / kHz;
    g.hostMonoNs = 7000000000LL + int64_t(std::llround(k * 1e9 / kHz));
    g.mode = kSyncModeHardware;
    for (int s : kSerials) g.frames.push_back(frame(s, g.frameId, g.timeStamp));
    return g;
}

static void drop(SyncGroup& g, int serial) {
    for (size_t i = 0; i < g.frames.size(); ++i)
        if (g.frames[i].serial == serial) { g.frames.erase(g.frames.begin() + i); return; }
}

static SyncFrame& at(SyncGroup& g, int serial) {
    for (auto& f : g.frames) if (f.serial == serial) return f;
    throw std::runtime_error("no such frame");
}

static const SyncStats::PerCamera& cam(const SyncStats::Summary& s, int serial) {
    for (auto& c : s.cameras) if (c.serial == serial) return c;
    throw std::runtime_error("no such camera");
}

// One groups.jsonl line through a JSON parser that is not ours.
static cv::FileStorage parse(const std::string& line) {
    return cv::FileStorage(line, cv::FileStorage::READ | cv::FileStorage::MEMORY |
                                 cv::FileStorage::FORMAT_JSON);
}

static void testGroupJson() {
    printf("groups.jsonl line\n");
    SyncGroup g = group(7);
    drop(g, 33277);
    at(g, 33275).blobs = {{100.25f, 200.5f, 31.0f, 0.875f}, {640.0f, 512.0f, 12.5f, 0.5f}};
    at(g, 33276).hwStampValid = true;
    at(g, 33276).hwTimeStamp = 12.000000125;
    g.sdkDropped = {33277};
    g.timeSpread = 0.000012;

    std::string line = groupJson(g, kSerials);
    check(line.find('\n') == std::string::npos, "one line, no newline inside");

    bool parsed = false;
    try {
        cv::FileStorage f = parse(line);
        parsed = f.isOpened();
        check(int(f["g"]) == 7 && int(f["group_frame_id"]) == 1007, "group index and frame ID");
        check(std::string(f["mode"]) == "hardware", "mode written by name");
        // Checked on the text: OpenCV 4.6's parser reads integers as 32-bit,
        // and a monotonic time in ns never fits.
        check(line.find("\"host_mono_ns\":" + std::to_string(g.hostMonoNs) + ",") != std::string::npos,
              "host monotonic ns written as a whole integer");
        check(near(double(f["ts_spread"]), 0.000012, 1e-9), "SDK timestamp spread");
        cv::FileNode cams = f["cams"];
        check(cams.size() == kSerials.size(), "every expected camera has an entry");
        bool order = true;
        for (size_t i = 0; i < kSerials.size(); ++i) order = order && int(cams[int(i)]["serial"]) == kSerials[i];
        check(order, "entries in the expected serial order");
        cv::FileNode miss = cams[2];
        check(int(miss["serial"]) == 33277 && int(miss["present"]) == 0 && miss["frame_id"].empty(),
              "missing camera recorded as present:false, with no frame fields");
        check(int(cams[0]["present"]) == 1 && int(cams[0]["frame_id"]) == 1007, "present camera has its frame ID");
        cv::FileNode b = cams[0]["blobs"];
        check(b.size() == 2 && near(double(b[0][0]), 100.25, 1e-3) && near(double(b[0][1]), 200.5, 1e-3) &&
              near(double(b[0][2]), 31.0, 1e-2) && near(double(b[0][3]), 0.875, 1e-4),
              "blob x, y, area, roundness");
        check(cams[1]["blobs"].size() == 0 && int(cams[1]["present"]) == 1,
              "a frame with no blobs is present with an empty list");
        check(int(cams[1]["hw_valid"]) == 1 && near(double(cams[1]["hw_ts"]), 12.000000125, 1e-9),
              "hardware timestamp to the nanosecond");
        check(f["sdk_dropped"].size() == 1 && int(f["sdk_dropped"][0]) == 33277, "SDK dropped list");
    } catch (const cv::Exception& e) {
        printf("    %s\n", e.what());
    }
    check(parsed, "line parses as JSON");

    // A frame from a camera nobody opened is kept and marked, not dropped.
    SyncGroup u = group(0);
    u.frames.push_back(frame(99999, 1000, 5.0));
    try {
        cv::FileStorage f = parse(groupJson(u, kSerials));
        cv::FileNode cams = f["cams"];
        check(cams.size() == kSerials.size() + 1 && int(cams[4]["serial"]) == 99999 &&
              int(cams[4]["unexpected"]) == 1, "unexpected serial appended and marked");
    } catch (const cv::Exception&) { check(false, "unexpected serial appended and marked"); }

    // Non-finite values must not break the file.
    SyncGroup n = group(0);
    n.timeSpread = std::numeric_limits<double>::quiet_NaN();
    at(n, 33275).timeStamp = std::numeric_limits<double>::infinity();
    std::string nl = groupJson(n, kSerials);
    check(nl.find("nan") == std::string::npos && nl.find("inf") == std::string::npos &&
          nl.find("\"ts_spread\":null") != std::string::npos && nl.find("\"ts\":null") != std::string::npos,
          "NaN and Inf written as null");
    // Not parsed here: null is valid JSON, but OpenCV 4.6's parser rejects it
    // ("Value 'null' is not supported by this parser").

    // No frames at all: ten "missing" entries, not an empty list.
    SyncGroup e = group(0);
    e.frames.clear();
    try {
        cv::FileStorage f = parse(groupJson(e, kSerials));
        bool all = f["cams"].size() == kSerials.size();
        for (auto c : f["cams"]) all = all && int(c["present"]) == 0;
        check(all, "an empty group lists every camera as missing");
    } catch (const cv::Exception&) { check(false, "an empty group lists every camera as missing"); }
}

static void testStats() {
    printf("statistics: ideal stream\n");
    {
        SyncStats st(kSerials);
        for (long k = 0; k < 300; ++k) st.add(group(k));
        auto s = st.summary();
        check(s.groups == 300 && s.complete == 300, "300 groups, all complete");
        check(s.byPresent[4] == 300 && s.byPresent[3] == 0, "histogram of cameras present");
        check(near(s.hostRateHz, 30.0, 1e-6) && near(s.groupTsRateHz, 30.0, 1e-6), "rate 30 Hz on both clocks");
        check(near(s.hostIntervalMs.median, 1000.0 / 30, 1e-3) && s.hostIntervalMs.n == 299, "host interval");
        check(s.groupIdGaps == 0 && s.groupIdSkipped == 0 && s.groupIdNotIncreasing == 0, "no group ID gaps");
        check(s.idSpreadGroups == 300 && s.idSpreadNonZero == 0 && s.idSpreadMax == 0, "no frame ID spread");
        check(s.tsSpreadUs.n == 300 && s.tsSpreadUs.max == 0, "no timestamp spread");
        check(s.hwReference == 0 && s.cameras[0].hwPairs == 0, "no hardware-stamp reference without stamps");
        bool clean = true;
        for (auto& c : s.cameras)
            clean = clean && c.present == 300 && c.missing == 0 && c.idGaps == 0 && c.idNotIncreasing == 0 &&
                    c.firstId == 1000 && c.lastId == 1299 && c.offsetMin == 0 && c.offsetMax == 0;
        check(clean, "per camera: present 300, no gaps, offset 0");
        check(s.modeHardware == 300 && s.modeSoftware == 0, "mode counted");
    }

    printf("statistics: faults\n");
    {
        SyncStats st(kSerials);
        for (long k = 0; k < 100; ++k) {
            SyncGroup g = group(k);
            if (k == 10 || k == 11) drop(g, 33276);          // missing from two groups
            if (k == 20) { drop(g, 33275); drop(g, 33659); } // two missing at once
            if (k >= 50) at(g, 33277).frameId += 3;          // this camera skips 3 IDs at k = 50
            if (k == 60) at(g, 33659).timeStamp += 250e-6;   // 250 us late in one group
            if (k == 70) g.sdkDropped = {33275};
            if (k == 80) at(g, 33275).object = false;
            if (k == 90) g.mode = kSyncModeSoftware;
            if (k == 95) at(g, 33276).blobs = {{1, 2, 3, 0.5f}, {4, 5, 6, 0.5f}};
            st.add(g);
        }
        auto s = st.summary();
        check(s.groups == 100 && s.complete == 97, "97 of 100 complete");
        check(s.byPresent[4] == 97 && s.byPresent[3] == 2 && s.byPresent[2] == 1, "histogram: 4:97 3:2 2:1");
        check(cam(s, 33276).missing == 2 && cam(s, 33275).missing == 1 && cam(s, 33659).missing == 1 &&
              cam(s, 33277).missing == 0, "missing counted per camera");
        // A camera absent from two groups returns with its ID 3 on: one gap, 2 skipped.
        check(cam(s, 33276).idGaps == 1 && cam(s, 33276).idSkipped == 2, "absence shows as a frame ID gap");
        check(cam(s, 33277).idGaps == 1 && cam(s, 33277).idSkipped == 3, "a jump of 4 is one gap, 3 skipped");
        check(cam(s, 33277).offsetMin == 0 && cam(s, 33277).offsetMax == 3, "frame ID offset to the group");
        check(s.idSpreadNonZero == 50 && s.idSpreadMax == 3, "50 groups hold differing frame IDs, spread 3");
        check(s.tsSpreadUs.n == 100 && near(s.tsSpreadUs.max, 250.0, 1e-3) && s.tsSpreadUs.median == 0,
              "timestamp spread: max 250 us, median 0");
        check(cam(s, 33275).sdkDropped == 1, "SDK dropped list counted");
        check(cam(s, 33275).wrongMode == 1, "a non-object frame counted");
        check(s.modeSoftware == 1 && s.modeHardware == 99, "one software-mode group");
        check(cam(s, 33276).framesWithBlobs == 1 && cam(s, 33276).blobs == 2, "blobs counted");
    }

    printf("statistics: group stream faults\n");
    {
        SyncStats st(kSerials);
        long k = 0;
        for (long i = 0; i < 60; ++i) {
            if (i == 30) k += 5;                              // five groups never delivered
            SyncGroup g = group(k++);
            if (i == 40) g.frameId = 0;                       // goes backwards once
            st.add(g);
        }
        auto s = st.summary();
        // i=30 jumps 6 (5 skipped); i=40 steps back; i=41 jumps forward again.
        check(s.groupIdGaps == 2 && s.groupIdNotIncreasing == 1, "group ID: two forward jumps, one step back");
        check(near(s.hostIntervalMs.max, 6 * 1000.0 / 30, 1e-3), "the hole shows as a 200 ms host interval");
        check(s.hostRateHz < 28.0, "and lowers the rate");
        check(s.groups == 60, "groups counted as delivered");
    }

    printf("statistics: hardware stamps, edge cases\n");
    {
        // Four cameras, four clocks seconds apart. 33276 is flagged master.
        // 33275 holds a fixed offset; 33277 drifts 10 ppm; 33659 has a fixed
        // offset plus a +/-20 us alternation, and no stamp in 5 groups.
        SyncStats st(kSerials);
        for (long k = 0; k < 600; ++k) {
            SyncGroup g = group(k);
            const double t = 4000.0 + k / kHz;
            auto set = [&](int serial, double v, bool master = false) {
                at(g, serial).hwStampValid = true; at(g, serial).hwTimeStamp = v; at(g, serial).hwMaster = master;
            };
            set(33276, t, true);
            set(33275, t + 2.5);
            set(33277, t - 3.25 + 10e-6 * (k / kHz));
            if (k < 100 || k >= 105) set(33659, t + 1.0 + (k % 2 ? 20e-6 : -20e-6));
            else at(g, 33659).hwTimeStamp = 99.0;             // not valid: must be ignored
            st.add(g);
        }
        auto s = st.summary();
        check(s.hwReference == 33276, "reference is the camera flagged master, not the first serial");
        check(cam(s, 33276).hwPairs == 600 && cam(s, 33276).hwOffsetRangeUs == 0, "reference: offset 0 to itself");
        check(near(cam(s, 33275).hwOffsetMeanS, 2.5, 1e-9) && cam(s, 33275).hwOffsetRangeUs < 1e-3 &&
              near(cam(s, 33275).hwOffsetSlopePpm, 0, 1e-3), "fixed clock offset: mean 2.5 s, no range, no drift");
        check(near(cam(s, 33277).hwOffsetSlopePpm, 10.0, 1e-3) && cam(s, 33277).hwOffsetResidUs < 1e-3 &&
              near(cam(s, 33277).hwOffsetRangeUs, 10.0 * 599 / kHz, 1e-2),
              "10 ppm drift found; range is the drift over the run; nothing left about the line");
        check(cam(s, 33659).hwPairs == 595 && near(cam(s, 33659).hwOffsetRangeUs, 40.0, 1e-2) &&
              near(cam(s, 33659).hwOffsetResidUs, 20.0, 0.1) && std::fabs(cam(s, 33659).hwOffsetSlopePpm) < 0.1,
              "+/-20 us alternation: range 40, rms 20; invalid stamps left out");
        check(near(cam(s, 33275).hwIntervalMs.median, 1000.0 / kHz, 1e-6) && cam(s, 33275).hwIntervalMs.n == 599,
              "stamp interval per camera");
        check(cam(s, 33659).hwIntervalMs.n == 593, "stamp intervals only across consecutive frame IDs");

        SyncStats one(kSerials);
        one.add(group(0));
        s = one.summary();
        check(s.hostRateHz == 0 && s.hostIntervalMs.n == 0, "one group: no rate, no interval");

        SyncStats none(kSerials);
        auto z = none.summary();
        check(z.groups == 0 && z.complete == 0 && z.tsSpreadUs.n == 0, "no groups: all zero");
        check(!none.text(30).empty() && none.json(30).find("nan") == std::string::npos, "empty summary prints");

        SyncStats un(kSerials);
        SyncGroup u = group(0);
        u.frames.push_back(frame(99999, 1000, 5.0));
        un.add(u);
        check(un.summary().unexpectedFrames == 1 && un.summary().complete == 1,
              "unexpected serial counted, not taken for an expected camera");

        auto d = SyncStats::dist({5, 1, 3, 2, 4});
        check(d.n == 5 && d.min == 1 && d.median == 3 && d.p95 == 5 && d.max == 5, "dist of 1..5");
        std::vector<double> hundred;
        for (int i = 1; i <= 100; ++i) hundred.push_back(i);
        auto h = SyncStats::dist(hundred);
        check(h.median == 50 && h.p95 == 95, "dist of 1..100: median 50, p95 95");
    }

    printf("statistics: summary JSON\n");
    {
        SyncStats st(kSerials);
        for (long k = 0; k < 30; ++k) { SyncGroup g = group(k); if (k == 3) drop(g, 33659); st.add(g); }
        bool ok = false;
        try {
            cv::FileStorage f = parse(st.json(30));
            ok = int(f["groups"]) == 30 && int(f["complete"]) == 29 &&
                 near(double(f["complete_fraction"]), 29.0 / 30, 1e-5) &&
                 f["cameras"].size() == kSerials.size() && int(f["cameras"][3]["missing"]) == 1 &&
                 near(double(f["host_rate_hz"]), 30.0, 1e-3) && near(double(f["requested_hz"]), 30.0, 1e-9);
        } catch (const cv::Exception& e) { printf("    %s\n", e.what()); }
        check(ok, "summary JSON parses and carries the same numbers");
    }
}

static void testMaskFile(const std::string& configDir) {
    printf("mask files\n");
    fs::path tmp = fs::temp_directory_path() / ("panoptes_test_sync_" + std::to_string(getpid()));
    fs::create_directories(tmp / "masks");
    auto write = [&](const fs::path& p, const std::string& text) { std::ofstream(p) << text; };
    const std::string head =
        "{\"serial\": 33275, \"learned_at\": \"2026-11-14T12:45:24-0800\","
        "\"grid\": {\"cell_px\": 2, \"cols\": 640, \"rows\": 512},"
        "\"settings\": {\"exposure_us\": 10, \"gain\": 0, \"intensity\": 15, \"threshold\": 40,"
        " \"frame_rate_hz\": 30, \"ir_filter\": 1},";

    write(tmp / "masks/good.json", head + "\"cells\": [[3, 4], [639, 511]]}");
    MaskFile m = loadMaskFile((tmp / "masks/good.json").string());
    check(m.ok && m.serial == 33275 && m.cols == 640 && m.rows == 512 && m.cellPx == 2, "good file: grid");
    check(m.ok && m.cellsSet == 2 && m.cells.at<uchar>(4, 3) == 1 && m.cells.at<uchar>(511, 639) == 1 &&
          m.cells.at<uchar>(3, 4) == 0, "cells are [x, y], stored row y, column x");
    check(m.ok && m.settings.exposure == 10 && m.settings.threshold == 40 && m.settings.frameRate == 30 &&
          m.learnedAt == "2026-11-14T12:45:24-0800", "settings and learn time");

    write(tmp / "masks/empty.json", head + "\"cells\": []}");
    MaskFile e = loadMaskFile((tmp / "masks/empty.json").string());
    check(e.ok && e.cellsSet == 0, "a mask with no cells is valid");

    write(tmp / "masks/outside.json", head + "\"cells\": [[640, 0]]}");
    check(!loadMaskFile((tmp / "masks/outside.json").string()).ok, "cell outside the grid refused");
    write(tmp / "masks/nogrid.json", "{\"serial\": 1, \"cells\": []}");
    check(!loadMaskFile((tmp / "masks/nogrid.json").string()).ok, "file without grid or settings refused");
    write(tmp / "masks/junk.json", "this is not json");
    check(!loadMaskFile((tmp / "masks/junk.json").string()).ok, "unparseable file refused, no throw");
    MaskFile gone = loadMaskFile((tmp / "masks/absent.json").string());
    check(!gone.ok && !gone.error.empty(), "absent file refused with a reason");

    Settings now = m.settings;
    check(maskSettingsDiff(m.settings, now).empty(), "same settings: no difference");
    now.exposure = 250; now.threshold = 200;
    std::string d = maskSettingsDiff(m.settings, now);
    check(d.find("exposure_us 10 -> 250") != std::string::npos && d.find("threshold 40 -> 200") != std::string::npos,
          "differing settings named with both values");

    write(tmp / "mask.yaml", "%YAML:1.0\ncamera:\n  exposure_us: 10\n  gain: 0\n  intensity: 15\n"
                             "  threshold: 40\n  frame_rate_hz: 30\n  ir_filter: 1\npaths:\n  masks: masks\n");
    bool ok = false;
    try {
        MaskConfig c = loadMaskConfig(tmp.string());
        ok = c.camera.mode == Mode::Object && c.camera.emptyFrames && c.camera.exposure == 10 &&
             c.camera.threshold == 40 && c.camera.frameRate == 30 && fs::path(c.masksDir) == tmp / "masks";
    } catch (const std::exception& ex) { printf("    %s\n", ex.what()); }
    check(ok, "mask.yaml camera block: object mode, empty frames on");

    write(tmp / "mask.yaml", "%YAML:1.0\ncamera:\n  exposure_us: 10\npaths:\n  masks: masks\n");
    bool threw = false;
    try { loadMaskConfig(tmp.string()); } catch (const std::exception&) { threw = true; }
    check(threw, "mask.yaml with a missing key throws rather than reading 0");
    fs::remove_all(tmp);

    // The repo's own config: what 08_sync will be given.
    bool repo = false;
    int n = 0, usable = 0;
    try {
        MaskConfig c = loadMaskConfig(configDir);
        repo = true;
        if (fs::exists(c.masksDir))
            for (auto& p : fs::directory_iterator(c.masksDir)) {
                if (p.path().extension() != ".json") continue;
                ++n;
                MaskFile f = loadMaskFile(p.path().string());
                bool good = f.ok && std::to_string(f.serial) == p.path().stem().string() &&
                            maskSettingsDiff(f.settings, c.camera).empty();
                if (good) ++usable;
                else printf("    %s: %s\n", p.path().filename().c_str(),
                            f.ok ? "serial or settings differ from mask.yaml" : f.error.c_str());
            }
    } catch (const std::exception& ex) { printf("    %s\n", ex.what()); }
    check(repo, configDir + "/mask.yaml loads");
    check(n > 0 && usable == n, "every saved mask in " + configDir + "/masks loads, matches its file name and "
                                "mask.yaml's settings (" + std::to_string(usable) + " of " + std::to_string(n) + ")");
}

int main(int argc, char** argv) {
    const std::string configDir = argc > 1 ? argv[1] : "config";
    testGroupJson();
    testStats();
    testMaskFile(configDir);
    printf("\ntest_sync: %d/%d passed\n", passed, passed + failed);
    return failed ? 1 : 0;
}
