// ChArUco board detection.
//
// ChArUco is a checkerboard with ArUco markers in the white squares.
// Detection runs in two stages: find the markers, which carry IDs and
// so identify themselves even in partial views; then use those IDs to
// locate the checkerboard corners and refine each to sub-pixel
// precision by reading the gray gradient across its black-white edges.
//
// Calibration consumes the CORNERS. Markers are scaffolding that says
// which corner is which.

#pragma once

#include "camera.h"            // Blob

#include <opencv2/core.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>

namespace mocap {

struct BoardSpec {
    int   squaresX   = 9;
    int   squaresY   = 6;
    float squareMM   = 90.0f;    // measured after printing, not nominal
    float markerMM   = 67.0f;
    int   dictionary = cv::aruco::DICT_5X5_1000;

    int totalMarkers() const { return (squaresX * squaresY) / 2; }
    int totalCorners() const { return (squaresX - 1) * (squaresY - 1); }
};

struct Detection {
    std::vector<cv::Point2f> corners;   // sub-pixel checkerboard corners
    std::vector<int>         cornerIds;
    int markersFound = 0;

    bool complete(const BoardSpec& b) const {
        return (int)corners.size() == b.totalCorners();
    }
};

class Detector {
public:
    explicit Detector(const BoardSpec& spec);

    Detection detect(const cv::Mat& gray) const;

    // Draw markers and corners onto a colour copy, for live view.
    cv::Mat annotate(const cv::Mat& gray, const Detection& d) const;

    const BoardSpec& spec() const { return spec_; }

private:
    BoardSpec spec_;
    cv::Ptr<cv::aruco::Dictionary>    dict_;
    cv::Ptr<cv::aruco::CharucoBoard>  board_;
};

// Markers are small, bright and round. Window glare is irregular and a
// distant speck is usually noise, so area and roundness - both measured
// by the camera itself - are the 2D object filter the SDK leaves to us.
bool plausibleMarker(const Blob& b, float minArea, float maxArea, float minRound);

}  // namespace mocap