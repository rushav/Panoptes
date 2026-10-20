// Capture coverage tracking: the acceptance criterion for calibration.
//
// WHY THIS EXISTS
//
// RMS reprojection error cannot tell you whether a calibration is good.
// RMS measures fit quality over the data you collected. If board poses
// cluster mid-frame, you are only measuring error where lens distortion
// is mild, and the distortion coefficients are then EXTRAPOLATING at the
// frame edges rather than being measured there. The metric looks clean
// precisely because the data is narrow.
//
// Measured against synthetic data with known ground truth:
//
//   set                empty cells   RMS        fx (truth 780)   edge error
//   centre-clustered   18/30         0.262 px   781.3            55.96 px
//   edge-covered        0/30         0.223 px   780.0            26.82 px
//
// The badly-sampled set had BETTER RMS, recovered focal length to 0.2%,
// and had fx/fy agreeing to 0.013% - while being 10x worse at the frame
// edge. So neither RMS nor fx/fy agreement detects bad spatial coverage.
//
// A second, independent failure: all-fronto-parallel views make focal
// length and distance mutually ambiguous. That one DOES show up as fx
// drifting from fy, but it is a different problem with a different fix,
// so tilt is tracked separately.
//
// Coverage visible during capture can be filled while the board is still
// in your hands. Coverage discovered afterward means recapturing.

#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace mocap {

struct Detection;
struct BoardSpec;

struct PoseEstimate {
    bool   valid    = false;
    double tiltDeg  = 0.0;    // angle away from fronto-parallel
    double distanceMM = 0.0;
};

class Coverage {
public:
    // gridX x gridY cells over a frame of imageSize.
    Coverage(cv::Size imageSize, int gridX = 5, int gridY = 6);

    // Record an accepted capture. Returns the pose estimated for it.
    PoseEstimate add(const Detection& d, const BoardSpec& board);

    void removeLast();      // undo a capture
    void clear();

    int captureCount() const { return (int)poses_.size(); }

    // Cells with zero corners. This is the acceptance criterion:
    // stop capturing when it reaches zero.
    int emptyCells() const;
    int totalCells() const { return gridX_ * gridY_; }

    // Fraction of captures with tilt beyond 20 degrees. Below roughly
    // 0.3 the set is too fronto-parallel and focal length will be
    // poorly constrained.
    double tiltedFraction(double thresholdDeg = 20.0) const;

    const std::vector<PoseEstimate>& poses() const { return poses_; }

    // Overlay the grid and a tilt histogram onto an image, for live view.
    void draw(cv::Mat& display) const;

private:
    cv::Size imageSize_;
    int gridX_, gridY_;

    std::vector<int> cellCounts_;              // corners landed per cell
    std::vector<std::vector<int>> perCapture_; // so removeLast can undo
    std::vector<PoseEstimate> poses_;

    int cellIndex(const cv::Point2f& p) const;
};

}  // namespace mocap