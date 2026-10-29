#include "autocapture.h"

#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mocap {

// ---------------------------------------------------------------- config

AutoConfig loadAutoConfig(const std::string& path) {
    cv::FileStorage fs;
    try {
        fs.open(path, cv::FileStorage::READ);
    } catch (const cv::Exception&) {}
    if (!fs.isOpened())
        throw std::runtime_error("autocapture config: cannot open " + path +
                                 " (run from the repo root, or pass --auto-config)");

    auto fail = [&](const std::string& what) {
        return std::runtime_error("autocapture config " + path + ": " + what);
    };
    auto number = [&](const char* key, double lo, double hi) {
        cv::FileNode n = fs[key];
        if (!n.isInt() && !n.isReal()) throw fail(std::string(key) + " is missing or not a number");
        double v = (double)n;
        if (v < lo || v > hi) {
            std::ostringstream os;
            os << key << " = " << v << " is outside " << lo << ".." << hi;
            throw fail(os.str());
        }
        return v;
    };

    AutoConfig c;
    c.source        = path;
    c.maxFitPx      = number("max_fit_px", 0.01, 1000);
    c.stillPx       = number("still_px", 0.001, 100);
    c.stillMaxPx    = number("still_max_px", 0.001, 100);
    c.minIntervalMs = (int)number("min_interval_ms", 0, 60000);
    c.minMovePx     = number("min_move_px", 0, 10000);
    c.flatDeg       = number("flat_deg", 0.1, 89);
    c.nearSquarePx  = number("near_square_px", 1, 10000);
    c.farSquarePx   = number("far_square_px", 1, 10000);
    c.approxFocalPx = number("approx_focal_px", 1, 100000);
    c.minPerBin     = (int)number("min_per_bin", 1, 1000);
    if (c.farSquarePx > c.nearSquarePx)
        throw fail("far_square_px must not exceed near_square_px");
    if (c.stillMaxPx < c.stillPx)
        throw fail("still_max_px must not be below still_px");
    return c;
}

std::string AutoConfig::toJson(const std::string& in) const {
    std::ostringstream os;
    os << "{\n"
       << in << "  \"source\": \"" << source << "\",\n"
       << in << "  \"max_fit_px\": " << maxFitPx << ",\n"
       << in << "  \"still_px\": " << stillPx << ",\n"
       << in << "  \"still_max_px\": " << stillMaxPx << ",\n"
       << in << "  \"min_interval_ms\": " << minIntervalMs << ",\n"
       << in << "  \"min_move_px\": " << minMovePx << ",\n"
       << in << "  \"flat_deg\": " << flatDeg << ",\n"
       << in << "  \"near_square_px\": " << nearSquarePx << ",\n"
       << in << "  \"far_square_px\": " << farSquarePx << ",\n"
       << in << "  \"approx_focal_px\": " << approxFocalPx << ",\n"
       << in << "  \"min_per_bin\": " << minPerBin << ",\n"
       << in << "  \"tilt_bins\": [\"flat\", \"+x+y\", \"+x-y\", \"-x+y\", \"-x-y\"],\n"
       << in << "  \"bands\": [\"near\", \"mid\", \"far\"]\n"
       << in << "}";
    return os.str();
}

// ---------------------------------------------------------------- names

const char* verdictName(Verdict v) {
    switch (v) {
        case Verdict::Accept:          return "accept";
        case Verdict::NotFull:         return "not_full";
        case Verdict::BadFit:          return "bad_fit";
        case Verdict::NoReference:     return "no_reference";
        case Verdict::Moving:          return "moving";
        case Verdict::Redundant:       return "redundant";
        case Verdict::SettingsChanged: return "settings_changed";
        case Verdict::TooSoon:         return "too_soon";
        case Verdict::SamePose:        return "same_pose";
    }
    return "?";
}

const char* tiltBinName(int t) {
    static const char* n[kTiltBins] = {"flat", "+x+y", "+x-y", "-x+y", "-x-y"};
    return (t >= 0 && t < kTiltBins) ? n[t] : "?";
}

const char* bandName(int b) {
    static const char* n[kBands] = {"near", "mid", "far"};
    return (b >= 0 && b < kBands) ? n[b] : "?";
}

std::string PoseBin::name() const {
    return valid ? std::string(bandName(band)) + "/" + tiltBinName(tilt) : "none";
}

// ---------------------------------------------------------------- pose

double apparentSquarePx(const Detection& d, const BoardSpec& board) {
    const int cx = board.squaresX - 1, cy = board.squaresY - 1;
    std::vector<cv::Point2f> at(cx * cy);
    std::vector<char> seen(cx * cy, 0);
    for (size_t i = 0; i < d.cornerIds.size(); ++i) {
        int id = d.cornerIds[i];
        if (id < 0 || id >= cx * cy) continue;
        at[id] = d.corners[i];
        seen[id] = 1;
    }

    // One board square along x (h) and along y (v), as image vectors:
    // the component-wise median over neighbouring corner pairs.
    std::vector<double> hx, hy, vx, vy;
    for (int id = 0; id < cx * cy; ++id) {
        if (!seen[id]) continue;
        int col = id % cx, row = id / cx;
        if (col + 1 < cx && seen[id + 1]) {
            hx.push_back(at[id + 1].x - at[id].x);
            hy.push_back(at[id + 1].y - at[id].y);
        }
        if (row + 1 < cy && seen[id + cx]) {
            vx.push_back(at[id + cx].x - at[id].x);
            vy.push_back(at[id + cx].y - at[id].y);
        }
    }
    if (hx.empty() || vx.empty()) return 0;
    auto median = [](std::vector<double>& x) {
        std::nth_element(x.begin(), x.begin() + x.size() / 2, x.end());
        return x[x.size() / 2];
    };
    double a = median(hx), b = median(vx), c = median(hy), e = median(vy);

    // Tilt foreshortens the image across the tilt axis, never along it, and
    // with compound tilt neither board axis is the tilt axis: both shrink.
    // The largest singular value of [h v] is the scale along the least
    // foreshortened board direction, which is the tilt axis.
    double t = a * a + b * b + c * c + e * e, det = a * e - b * c;
    return std::sqrt((t + std::sqrt(std::max(0.0, t * t - 4 * det * det))) / 2);
}

PoseBin poseBin(const Detection& d, const BoardSpec& board, cv::Size imageSize,
                const AutoConfig& cfg) {
    PoseBin b;
    if (!d.complete(board)) return b;

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = cfg.approxFocalPx;
    K.at<double>(1, 1) = cfg.approxFocalPx;
    K.at<double>(0, 2) = imageSize.width  / 2.0;
    K.at<double>(1, 2) = imageSize.height / 2.0;

    // Corner id -> board coordinates, as in lib/coverage.
    const int cornersX = board.squaresX - 1;
    std::vector<cv::Point3f> objectPoints;
    for (int id : d.cornerIds) {
        int row = id / cornersX, col = id % cornersX;
        objectPoints.emplace_back((col + 1) * board.squareMM,
                                  (row + 1) * board.squareMM, 0.0f);
    }

    cv::Mat rvec, tvec;
    if (!cv::solvePnP(objectPoints, d.corners, K, cv::Mat(), rvec, tvec)) return b;
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // Board normal in camera coordinates, turned to face the camera.
    double nx = R.at<double>(0, 2), ny = R.at<double>(1, 2), nz = R.at<double>(2, 2);
    if (nz > 0) { nx = -nx; ny = -ny; nz = -nz; }

    const double deg = 180.0 / CV_PI;
    b.tiltXDeg = std::atan2(ny, -nz) * deg;
    b.tiltYDeg = std::atan2(nx, -nz) * deg;
    b.tiltDeg  = std::acos(std::clamp(-nz, 0.0, 1.0)) * deg;

    if (b.tiltDeg < cfg.flatDeg) b.tilt = Flat;
    else if (b.tiltXDeg >= 0)    b.tilt = b.tiltYDeg >= 0 ? XpYp : XpYn;
    else                         b.tilt = b.tiltYDeg >= 0 ? XnYp : XnYn;

    b.squarePx = apparentSquarePx(d, board);
    b.band = b.squarePx >= cfg.nearSquarePx ? Near
           : b.squarePx <  cfg.farSquarePx  ? Far : Mid;
    b.valid = true;
    return b;
}

// ---------------------------------------------------------------- decision

static std::string fixed(double v, int places) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(places) << v;
    return os.str();
}

std::string Decision::acceptReason() const {
    std::string r;
    if (!newCells.empty()) {
        r = "cells ";
        for (size_t i = 0; i < newCells.size(); ++i)
            r += (i ? "," : "") + std::to_string(newCells[i]);
    }
    if (binUnfilled) r += (r.empty() ? "" : " + ") + std::string("bin ") + bin.name();
    return r;
}

std::string Decision::describe(const AutoConfig& c) const {
    switch (verdict) {
        case Verdict::Accept:
            return "ACCEPT " + acceptReason();
        case Verdict::NotFull:
            return "not_full " + std::to_string(corners) + " corners";
        case Verdict::BadFit:
            return "bad_fit " + (fitPx < 0 ? std::string("n/a")
                                           : fixed(fitPx, 1) + " >= " + fixed(c.maxFitPx, 1)) + " px";
        case Verdict::NoReference:
            return "no_reference (previous frame not full)";
        case Verdict::Moving:
            return motionMedPx >= c.stillPx
                 ? "moving median " + fixed(motionMedPx, 2) + " >= " + fixed(c.stillPx, 2) + " px"
                 : "moving max " + fixed(motionMaxPx, 2) + " >= " + fixed(c.stillMaxPx, 2) + " px";
        case Verdict::Redundant:
            return "redundant (no empty cell, bin " + bin.name() + " filled)";
        case Verdict::SettingsChanged:
            return "settings_changed";
        case Verdict::TooSoon:
            return "too_soon (< " + std::to_string(c.minIntervalMs) + " ms since accept)";
        case Verdict::SamePose:
            return "same_pose (moved " + fixed(movePx, 1) + " < " + fixed(c.minMovePx, 1) +
                   " px since accept)";
    }
    return "?";
}

static std::string numOrNull(double v, int places) {
    return v < 0 ? std::string("null") : fixed(v, places);
}

std::string Decision::jsonFields() const {
    std::ostringstream os;
    os << "\"auto\": {\"reason\": \"" << acceptReason() << "\", \"cells\": [";
    for (size_t i = 0; i < newCells.size(); ++i) os << (i ? ", " : "") << newCells[i];
    os << "], \"bin\": \"" << bin.name() << "\""
       << ", \"bin_unfilled\": " << (binUnfilled ? "true" : "false")
       << ", \"motion_median_px\": " << numOrNull(motionMedPx, 4)
       << ", \"motion_max_px\": " << numOrNull(motionMaxPx, 4)
       << ", \"move_since_accept_px\": " << numOrNull(movePx, 2)
       << ", \"fit_px\": " << numOrNull(fitPx, 3)
       << ", \"square_px\": " << fixed(bin.squarePx, 2)
       << ", \"tilt_deg\": " << fixed(bin.tiltDeg, 2)
       << ", \"tilt_x_deg\": " << fixed(bin.tiltXDeg, 2)
       << ", \"tilt_y_deg\": " << fixed(bin.tiltYDeg, 2)
       << ", \"t_ms\": " << fixed(tMs, 1) << "}";
    return os.str();
}

// ---------------------------------------------------------------- gates

AutoCapture::AutoCapture(const AutoConfig& cfg, const Detector& detector, cv::Size imageSize)
    : cfg_(cfg), detector_(detector), imageSize_(imageSize) {}

bool AutoCapture::settingsMatch(const Settings& s) const {
    const Settings& r = reference_;
    return s.exposure == r.exposure && s.gain == r.gain && s.intensity == r.intensity &&
           s.frameRate == r.frameRate && s.irFilter == r.irFilter;
}

std::string AutoCapture::settingsDiff(const Settings& s) const {
    const Settings& r = reference_;
    std::string out;
    auto add = [&](const char* name, int was, int now) {
        if (was == now) return;
        out += (out.empty() ? "" : ", ") + std::string(name) + " " +
               std::to_string(was) + " -> " + std::to_string(now);
    };
    add("exposure", r.exposure, s.exposure);
    add("gain", r.gain, s.gain);
    add("intensity", r.intensity, s.intensity);
    add("frame_rate", r.frameRate, s.frameRate);
    add("ir_filter", r.irFilter, s.irFilter);
    return out;
}

// Distance each of this frame's corners moved from a reference set, matched
// by corner ID. Empty if no ID is in both.
static std::vector<double> cornerMoves(const Detection& d, const std::vector<cv::Point2f>& ref,
                                       const std::vector<char>& seen) {
    std::vector<double> moves;
    for (size_t i = 0; i < d.cornerIds.size(); ++i) {
        int id = d.cornerIds[i];
        if (id < 0 || id >= (int)ref.size() || !seen[id]) continue;
        moves.push_back(cv::norm(d.corners[i] - ref[id]));
    }
    return moves;
}

// -1 for an empty set.
static double largest(const std::vector<double>& x) {
    return x.empty() ? -1 : *std::max_element(x.begin(), x.end());
}

static double median(std::vector<double> x) {
    if (x.empty()) return -1;
    const size_t n = x.size(), m = n / 2;
    std::nth_element(x.begin(), x.begin() + m, x.end());
    if (n % 2) return x[m];
    return (x[m] + *std::max_element(x.begin(), x.begin() + m)) / 2;
}

Decision AutoCapture::evaluate(const Detection& d, const Settings& actual, double tMs,
                               const Coverage& coverage) {
    const BoardSpec& board = detector_.spec();
    const int total = board.totalCorners();

    ++evaluated_;
    if (!haveSettings_) { reference_ = actual; haveSettings_ = true; }

    Decision dec;
    dec.tMs = tMs;
    dec.corners = (int)d.corners.size();

    // Gate 1 inputs. A complete board is also the only kind that can serve
    // as the next frame's motion reference.
    const bool full = d.complete(board);
    if (full) dec.fitPx = detector_.layoutResidualPx(d);
    const bool gate1 = full && dec.fitPx >= 0 && dec.fitPx < cfg_.maxFitPx;

    // Everything below is measured for any frame that passes gate 1, so the
    // live view can show it, but only decided in gate order.
    const bool hadPrev = havePrev_;
    std::vector<char> seen(total, 0);
    if (gate1) {
        if (hadPrev) {
            std::vector<double> moves = cornerMoves(d, prev_, prevSeen_);
            dec.motionMedPx = median(moves);
            dec.motionMaxPx = largest(moves);
        }
        dec.bin = poseBin(d, board, imageSize_, cfg_);
        dec.newCells = coverage.emptyCellsHit(d.corners);
        dec.binUnfilled = dec.bin.valid &&
                          binCount(dec.bin.tilt, dec.bin.band) < cfg_.minPerBin;
        if (haveAccept_) {
            std::vector<char> all(total, 1);
            dec.movePx = largest(cornerMoves(d, lastAccept_, all));
        }
    }

    // This frame becomes the next frame's reference, or clears it.
    havePrev_ = gate1;
    if (gate1) {
        prev_.assign(total, {});
        prevSeen_.assign(total, 0);
        for (size_t i = 0; i < d.cornerIds.size(); ++i) {
            prev_[d.cornerIds[i]] = d.corners[i];
            prevSeen_[d.cornerIds[i]] = 1;
        }
    }

    auto decide = [&](Verdict v) {
        dec.verdict = v;
        ++tally_[(int)v];
        return dec;
    };

    if (!full)  return decide(Verdict::NotFull);
    if (!gate1) return decide(Verdict::BadFit);

    if (!hadPrev || dec.motionMaxPx < 0) return decide(Verdict::NoReference);
    if (dec.motionMedPx >= cfg_.stillPx || dec.motionMaxPx >= cfg_.stillMaxPx)
        return decide(Verdict::Moving);

    if (dec.newCells.empty() && !dec.binUnfilled) return decide(Verdict::Redundant);

    if (!settingsMatch(actual)) return decide(Verdict::SettingsChanged);

    if (haveAccept_ && tMs - lastAcceptMs_ < cfg_.minIntervalMs)
        return decide(Verdict::TooSoon);

    if (haveAccept_ && cfg_.minMovePx > 0 && dec.movePx < cfg_.minMovePx)
        return decide(Verdict::SamePose);

    return decide(Verdict::Accept);
}

void AutoCapture::commit(const Decision& dec, const Detection& d, Coverage& coverage) {
    if (!dec.accepted() || !dec.bin.valid)
        throw std::logic_error("AutoCapture::commit: frame was not accepted");

    const BoardSpec& board = detector_.spec();
    coverage.add(d, board);
    ++bins_[dec.bin.band * kTiltBins + dec.bin.tilt];

    haveAccept_ = true;
    lastAcceptMs_ = dec.tMs;
    lastAccept_.assign(board.totalCorners(), {});
    for (size_t i = 0; i < d.cornerIds.size(); ++i) lastAccept_[d.cornerIds[i]] = d.corners[i];
}

int AutoCapture::binsShort() const {
    int n = 0;
    for (int c : bins_) if (c < cfg_.minPerBin) ++n;
    return n;
}

bool AutoCapture::complete(const Coverage& coverage) const {
    return coverage.emptyCells() == 0 && binsShort() == 0;
}

std::string AutoCapture::summaryJson(const std::string& in) const {
    std::ostringstream os;
    os << "{\n" << in << "  \"config\": " << cfg_.toJson(in + "  ") << ",\n";

    os << in << "  \"pose_bins\": {";
    for (int b = 0; b < kBands; ++b)
        for (int t = 0; t < kTiltBins; ++t)
            os << (b || t ? ", " : "") << "\"" << bandName(b) << "/" << tiltBinName(t)
               << "\": " << binCount(t, b);
    os << "},\n";

    os << in << "  \"evaluated\": " << evaluated_ << ",\n";
    os << in << "  \"verdicts\": {";
    for (int v = 0; v < kVerdictCount; ++v)
        os << (v ? ", " : "") << "\"" << verdictName((Verdict)v) << "\": " << tally_[v];
    os << "},\n";

    os << in << "  \"reference_settings\": {\"exposure\": " << reference_.exposure
       << ", \"gain\": " << reference_.gain
       << ", \"intensity\": " << reference_.intensity
       << ", \"frame_rate\": " << reference_.frameRate
       << ", \"ir_filter\": " << (reference_.irFilter ? "true" : "false") << "}\n";
    os << in << "}";
    return os.str();
}

}  // namespace mocap
