// Session output: captured frames plus the provenance to interpret them.
//
// Every calibration output carries versions, timestamp, source session,
// board geometry, and the camera settings each frame was taken under.
// The rig will be moved and recalibrated repeatedly, so a folder of
// PNGs with no record of how they were taken is nearly worthless.

#pragma once

#include "camera.h"
#include "detect.h"

#include <opencv2/core.hpp>
#include <string>

namespace mocap {

class Session {
public:
    // Creates <root>/intrinsics_<serial>_<timestamp>/
    Session(const std::string& root, int serial, const BoardSpec& board);

    // Writes frame_NNNN.png and records its settings and corner count.
    void record(const cv::Mat& gray, const Detection& d, const Settings& actual);

    void removeLast();

    // Writes session.json. Safe to call repeatedly.
    void flush() const;

    const std::string& path() const { return path_; }
    int count() const { return count_; }

private:
    std::string path_;
    int serial_;
    BoardSpec board_;
    int count_ = 0;
    std::string json_;      // accumulated per-frame entries
};

}  // namespace mocap