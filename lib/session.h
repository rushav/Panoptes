// Session output: captured frames plus the provenance to interpret them.
//
// Every calibration output carries versions, timestamp, source session,
// the full board record (config/charuco_board.yaml, measured sizes
// included), and the camera settings each frame was taken under.
// The rig will be moved and recalibrated repeatedly, so a folder of
// PNGs with no record of how they were taken is nearly worthless.

#pragma once

#include "camera.h"
#include "detect.h"

#include <opencv2/core.hpp>
#include <string>
#include <utility>
#include <vector>

namespace mocap {

class Session {
public:
    // Creates <root>/intrinsics_<serial>_<timestamp>/
    Session(const std::string& root, int serial, const BoardSpec& board);

    // Writes frame_NNNN.png and records its settings and corner count.
    // `extraFields`, if given, is appended inside the frame's JSON object
    // as further "key": value pairs (no braces, no leading comma).
    void record(const cv::Mat& gray, const Detection& d, const Settings& actual,
                const std::string& extraFields = "");

    // A top-level "key": <json> entry in session.json, written after the
    // board record. Setting a key again replaces it.
    void setExtra(const std::string& key, const std::string& json);

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
    std::vector<std::pair<std::string, std::string>> extras_;
};

}  // namespace mocap