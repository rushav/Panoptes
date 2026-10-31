// Automatic frame selection for continuous capture.
//
// The operator moves the board slowly through the volume and this decides,
// frame by frame, which views to keep. It replaces the operator's judgement
// at the space bar, so every rule it applies is explicit, named, and recorded
// in session.json with the session.
//
// WHY STILLNESS IS A GATE
//
// The grayscale operating point exposes for 8133 us. At ~2 mm/px at 1.5 m a
// board moving 20 cm/s smears a corner by ~0.8 px during one exposure, 15x
// the 0.054 px static jitter (FINDINGS 4). A space-bar operator holds the
// board still before pressing; a continuous capture has to check for it.
// Blur is not visible in the corner count or the fit residual, so it is
// measured directly: how far each corner moved since the previous frame.
// That is a proxy for motion DURING the exposure, not a measurement of it.
//
// A frame is accepted only if, in this order:
//   1. full board: every corner found, fit residual below max_fit_px
//   2. still: since the previous frame, which must itself have passed
//      gate 1, the MEDIAN corner moved less than still_px and the LARGEST
//      move is under still_max_px. The median measures the board; the max
//      only catches a gross outlier. A max-only test at still_px tripped on
//      the jitter tail of 88 corners with the board static.
//   3. novel: fills an empty coverage cell, or lands in a pose bin holding
//      fewer than min_per_bin accepts
//   4. camera settings equal those at the first evaluated frame
//   5. at least min_interval_ms since the last accept
//   6. some corner moved at least min_move_px since the last accept
// The first gate that fails names the rejection.
//
// Reusable beyond intrinsics: the gates take corners and a board, not a
// camera, so the wand/extrinsics stage can run the same rules.

#pragma once

#include "camera.h"            // Settings
#include "coverage.h"
#include "detect.h"

#include <opencv2/core.hpp>
#include <array>
#include <string>
#include <vector>

namespace mocap {

// Loaded from config/autocapture.yaml. No defaults: every field must be
// present in the file, so a session never runs on a silently assumed value.
struct AutoConfig {
    std::string source;
    double maxFitPx      = 0;
    double stillPx       = 0;     // median corner move
    double stillMaxPx    = 0;     // largest corner move
    int    minIntervalMs = 0;
    double minMovePx     = 0;
    double flatDeg       = 0;
    double nearSquarePx  = 0;
    double farSquarePx   = 0;
    double approxFocalPx = 0;
    int    minPerBin     = 0;

    std::string toJson(const std::string& indent) const;
};

// Throws std::runtime_error naming the file and field on any missing or
// out-of-range value.
AutoConfig loadAutoConfig(const std::string& path);

inline const char* kAutoConfig = "config/autocapture.yaml";

// Why a frame was or was not kept. Order matches the gate order.
enum class Verdict {
    Accept,
    NotFull,          // gate 1: corners missing
    BadFit,           // gate 1: fit residual too high, or not computable
    NoReference,      // gate 2: previous frame did not pass gate 1
    Moving,           // gate 2: median >= still_px, or max >= still_max_px
    Redundant,        // gate 3: no empty cell, no unfilled bin
    SettingsChanged,  // gate 4
    TooSoon,          // gate 5
    SamePose,         // gate 6
};
constexpr int kVerdictCount = 9;

// Stable snake_case names, used in session.json and on screen.
const char* verdictName(Verdict v);

// Pose bins: 5 tilt directions x 3 distance bands.
//
// Tilt is measured from the board normal n in camera coordinates, flipped
// to face the camera (n.z < 0):
//   tilt about x = atan2(n.y, -n.z)   positive: board faces image-down
//   tilt about y = atan2(n.x, -n.z)   positive: board faces image-right
// "flat" when the angle between n and the view axis is under flat_deg;
// otherwise the quadrant of the two signs. A board tilted about one axis
// only sits on a quadrant boundary and its bin follows the sign of the
// small other angle.
enum TiltBin { Flat, XpYp, XpYn, XnYp, XnYn };
enum Band    { Near, Mid, Far };
constexpr int kTiltBins = 5;
constexpr int kBands    = 3;
const char* tiltBinName(int t);   // "flat", "+x+y", ...
const char* bandName(int b);      // "near", "mid", "far"

struct PoseBin {
    bool   valid = false;
    int    tilt  = Flat;
    int    band  = Mid;
    double tiltXDeg = 0, tiltYDeg = 0, tiltDeg = 0;
    double squarePx = 0;          // apparent size used for the band
    std::string name() const;     // "mid/+x-y"
};

// Pose bin of a full board. Needs every corner of the board; the tilt uses
// a rough pinhole camera (approx_focal_px, centre at image centre).
PoseBin poseBin(const Detection& d, const BoardSpec& board, cv::Size imageSize,
                const AutoConfig& cfg);

// Apparent size of one board square, px, along the board direction that
// tilt foreshortens least (the tilt axis). Close to f * square / distance at
// any tilt, unlike a spacing along a board axis, which shrinks under
// compound tilt. 0 if no neighbouring corners were found.
double apparentSquarePx(const Detection& d, const BoardSpec& board);

// One board square along the board's x (h) and y (v) directions, as image
// vectors: the component-wise median over neighbouring corner pairs. Works
// on a partial board. False if no neighbouring pair exists along either.
bool boardSteps(const Detection& d, const BoardSpec& board, cv::Point2d& h, cv::Point2d& v);

// Apparent in-plane rotation of the board: the image angle of h, degrees,
// atan2(h.y, h.x). Image y points down, so positive is clockwise on screen.
// NaN if boardSteps() fails.
double boardRollDeg(const Detection& d, const BoardSpec& board);

struct Decision {
    Verdict verdict = Verdict::NotFull;
    bool accepted() const { return verdict == Verdict::Accept; }

    int    corners  = 0;
    double fitPx    = -1;     // -1: not computed
    double motionMedPx = -1;  // median corner move since previous frame; -1: none
    double motionMaxPx = -1;  // largest corner move since previous frame; -1: none
    double movePx   = -1;     // largest corner move since last accept; -1: none
    PoseBin bin;
    std::vector<int> newCells;   // empty coverage cells this frame fills
    bool   binUnfilled = false;  // its pose bin held fewer than min_per_bin
    double tMs = 0;

    // "cells 3,8 + bin near/flat", for status lines and session.json.
    std::string acceptReason() const;
    // One-line description of the verdict with the value that decided it.
    std::string describe(const AutoConfig& cfg) const;
    // Per-frame JSON fields for session.json, without braces.
    std::string jsonFields() const;
};

class AutoCapture {
public:
    AutoCapture(const AutoConfig& cfg, const Detector& detector, cv::Size imageSize);

    // Run the gates on one new camera frame. tMs is a monotonic timestamp.
    // Updates the previous-frame reference and the rejection tally, but not
    // coverage or bins: call commit() for an accepted frame you keep.
    Decision evaluate(const Detection& d, const Settings& actual, double tMs,
                      const Coverage& coverage);

    // Record an accepted frame: adds it to coverage and its pose bin.
    void commit(const Decision& dec, const Detection& d, Coverage& coverage);

    // Gate 4's reference, taken from the first evaluated frame.
    bool hasReference() const { return haveSettings_; }
    const Settings& referenceSettings() const { return reference_; }
    bool settingsMatch(const Settings& s) const;
    std::string settingsDiff(const Settings& s) const;   // "exposure 8133 -> 7133"

    // Session end: zero empty cells and every bin at min_per_bin.
    bool complete(const Coverage& coverage) const;
    int  binsShort() const;                              // bins under min_per_bin

    int binCount(int tilt, int band) const { return bins_[band * kTiltBins + tilt]; }
    int tally(Verdict v) const { return tally_[(int)v]; }
    int evaluated() const { return evaluated_; }

    // {"bins": ..., "tally": ..., ...} for session.json.
    std::string summaryJson(const std::string& indent) const;

    const AutoConfig& config() const { return cfg_; }

private:
    AutoConfig cfg_;
    const Detector& detector_;
    cv::Size imageSize_;

    bool haveSettings_ = false;
    Settings reference_;

    // Previous frame, if it passed gate 1, keyed by corner ID.
    bool havePrev_ = false;
    std::vector<cv::Point2f> prev_;
    std::vector<char> prevSeen_;

    bool haveAccept_ = false;
    double lastAcceptMs_ = 0;
    std::vector<cv::Point2f> lastAccept_;

    std::array<int, kTiltBins * kBands> bins_{};
    std::array<int, kVerdictCount> tally_{};
    int evaluated_ = 0;
};

}  // namespace mocap
