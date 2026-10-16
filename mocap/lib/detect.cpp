#include "detect.h"

#include <opencv2/imgproc.hpp>

namespace mocap {

Detector::Detector(const BoardSpec& spec) : spec_(spec) {
    dict_  = cv::aruco::getPredefinedDictionary(spec.dictionary);
    board_ = cv::aruco::CharucoBoard::create(
        spec.squaresX, spec.squaresY, spec.squareMM, spec.markerMM, dict_);

    // No setLegacyPattern here: OpenCV 4.6's board layout already IS the
    // legacy one. Only the 5.x Python solver needs that flag set
    // explicitly. Same physical board, two code paths.
}

Detection Detector::detect(const cv::Mat& gray) const {
    Detection d;
    if (gray.empty()) return d;

    // Stage 1: locate the ArUco markers. Each encodes an ID, so partial
    // views are usable - visible markers identify themselves.
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::detectMarkers(gray, dict_, markerCorners, markerIds);
    d.markersFound = (int)markerIds.size();
    if (markerIds.empty()) return d;

    // Stage 2: from known marker IDs, work out where the checkerboard
    // corners must be, and refine each to sub-pixel precision.
    cv::aruco::interpolateCornersCharuco(
        markerCorners, markerIds, gray, board_, d.corners, d.cornerIds);

    return d;
}

cv::Mat Detector::annotate(const cv::Mat& gray, const Detection& d) const {
    cv::Mat out;
    cv::cvtColor(gray, out, cv::COLOR_GRAY2BGR);

    if (!d.cornerIds.empty())
        cv::aruco::drawDetectedCornersCharuco(
            out, d.corners, d.cornerIds, cv::Scalar(0, 0, 255));

    return out;
}

// Markers are small, bright and round. Window glare is irregular; a
// distant speck is usually noise. Area and roundness come from the
// camera itself - this is the 2D object filter the SDK leaves to us.
bool plausibleMarker(const Blob& b, float minArea, float maxArea, float minRound) {
    return b.area >= minArea && b.area <= maxArea && b.roundness >= minRound;
}

}  // namespace mocap
