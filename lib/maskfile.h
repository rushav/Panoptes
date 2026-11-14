// Reading what src/07_mask wrote: config/mask.yaml's camera block and the
// saved per-camera masks (config/masks/<serial>.json). OpenCV only, no SDK.
//
// 07_mask has its own copies of these readers and is the only writer; a
// change to the file format has to be made in both places.

#pragma once

#include "camera.h"

#include <opencv2/core.hpp>
#include <string>

namespace mocap {

// The object-mode settings masks are learned and applied under, and where
// the masks are kept. Throws std::runtime_error on a missing file or key.
struct MaskConfig {
    Settings camera;          // mode Object, emptyFrames on
    std::string masksDir;
};
MaskConfig loadMaskConfig(const std::string& configDir);

struct MaskFile {
    bool ok = false;
    std::string error;        // why not, when !ok
    int serial = 0;
    int cellPx = 0, cols = 0, rows = 0;
    Settings settings;        // what it was learned under
    std::string learnedAt;
    cv::Mat cells;            // CV_8U rows x cols, 1 = blocked
    int cellsSet = 0;
};
MaskFile loadMaskFile(const std::string& path);

// "" if `now` matches the settings the mask was learned under on exposure,
// gain, intensity, threshold and frame rate; otherwise the differences.
std::string maskSettingsDiff(const Settings& learned, const Settings& now);

}  // namespace mocap
