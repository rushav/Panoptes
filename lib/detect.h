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
#include <string>
#include <vector>

namespace mocap {

// Board geometry, loaded from config/charuco_board.yaml. There are no
// defaults: a board that was not loaded from a file is not a board.
struct BoardSpec {
    std::string source;             // the yaml it was loaded from
    std::string name;
    int   squaresX   = 0;           // cols
    int   squaresY   = 0;           // rows
    float squareMM   = 0;           // measured with calipers
    float markerMM   = 0;           // measured with calipers
    float squareNominalMM = 0;      // 0 = not recorded
    float markerNominalMM = 0;
    std::string dictionaryName;     // e.g. "DICT_5X5_100"
    int   dictionary = -1;
    bool  legacyPattern = false;    // OpenCV >= 4.7 meaning; see detect.cpp

    // provenance, free text
    std::string vendor, orderDate, material, patternMM, plateMM,
                measuredBy, measuredDate;

    int totalMarkers() const { return (squaresX * squaresY) / 2; }
    int totalCorners() const { return (squaresX - 1) * (squaresY - 1); }

    // One line naming the exact board, for startup output.
    std::string describe() const;

    // The full record as a JSON object, for session.json.
    std::string toJson(const std::string& indent) const;
};

// Throws std::runtime_error naming the file and field on any missing or
// malformed value, including a blank measured size.
BoardSpec loadBoardSpec(const std::string& path);

// Default board file, relative to the repo root the tools are run from.
inline const char* kBoardConfig = "config/charuco_board.yaml";

struct Detection {
    std::vector<cv::Point2f> corners;   // sub-pixel checkerboard corners
    std::vector<int>         cornerIds;
    int markersFound = 0;
    std::vector<int>                       markerIds;
    std::vector<std::vector<cv::Point2f>>  markerCorners;

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

    // Largest distance, in px, between the detected corners and a
    // homography fitted from their board coordinates. A few px (lens
    // distortion) when corner IDs are right; tens to hundreds when the
    // layout is wrong and IDs are attached to the wrong corners, which a
    // corner COUNT alone does not reveal. -1 with fewer than 6 corners.
    double layoutResidualPx(const Detection& d) const;

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