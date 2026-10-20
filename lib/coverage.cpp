#include "coverage.h"
#include "detect.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace mocap {

Coverage::Coverage(cv::Size imageSize, int gridX, int gridY)
    : imageSize_(imageSize), gridX_(gridX), gridY_(gridY),
      cellCounts_(gridX * gridY, 0) {}

int Coverage::cellIndex(const cv::Point2f& p) const {
    int cx = std::clamp(int(p.x * gridX_ / imageSize_.width),  0, gridX_ - 1);
    int cy = std::clamp(int(p.y * gridY_ / imageSize_.height), 0, gridY_ - 1);
    return cy * gridX_ + cx;
}

PoseEstimate Coverage::add(const Detection& d, const BoardSpec& board) {
    // Tally which cells this capture's corners fell into, keeping the
    // per-capture record so the tally can be undone.
    std::vector<int> touched;
    for (auto& c : d.corners) {
        int i = cellIndex(c);
        ++cellCounts_[i];
        touched.push_back(i);
    }
    perCapture_.push_back(touched);

    // Estimate board pose to measure tilt. This needs intrinsics, which
    // we do not have yet - that is what we are calibrating. A rough
    // guess is fine here: we only want the tilt ANGLE for diversity
    // scoring, not an accurate pose. Errors of a few percent in focal
    // length shift the angle negligibly.
    PoseEstimate pose;
    if ((int)d.corners.size() >= 6) {
        // Approximate intrinsics, from the prior rig. We only need the
        // tilt ANGLE for diversity scoring, and a few percent error in
        // focal length barely moves it.
        double f = 780.0;
        cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
        K.at<double>(0, 0) = f;
        K.at<double>(1, 1) = f;
        K.at<double>(0, 2) = imageSize_.width  / 2.0;
        K.at<double>(1, 2) = imageSize_.height / 2.0;

        // Build the 3D positions of the detected corners in board
        // coordinates. Corner id -> (row, col) on the interior grid.
        int cornersX = board.squaresX - 1;
        std::vector<cv::Point3f> objectPoints;
        for (int id : d.cornerIds) {
            int row = id / cornersX, col = id % cornersX;
            objectPoints.emplace_back((col + 1) * board.squareMM,
                                      (row + 1) * board.squareMM, 0.0f);
        }

        cv::Mat rvec, tvec;
        if (cv::solvePnP(objectPoints, d.corners, K, cv::Mat(), rvec, tvec)) {
            cv::Mat R;
            cv::Rodrigues(rvec, R);

            // The board's normal is its local z axis. Fronto-parallel
            // means that normal points straight back at the camera, so
            // R(2,2) is its cosine against the view direction.
            double cosTilt = std::abs(R.at<double>(2,2));
            pose.tiltDeg = std::acos(std::clamp(cosTilt, 0.0, 1.0)) * 180.0 / CV_PI;
            pose.distanceMM = cv::norm(tvec);
            pose.valid = true;
        }
    }

    poses_.push_back(pose);
    return pose;
}

void Coverage::removeLast() {
    if (perCapture_.empty()) return;
    for (int i : perCapture_.back()) --cellCounts_[i];
    perCapture_.pop_back();
    poses_.pop_back();
}

void Coverage::clear() {
    std::fill(cellCounts_.begin(), cellCounts_.end(), 0);
    perCapture_.clear();
    poses_.clear();
}

int Coverage::emptyCells() const {
    return (int)std::count(cellCounts_.begin(), cellCounts_.end(), 0);
}

double Coverage::tiltedFraction(double thresholdDeg) const {
    if (poses_.empty()) return 0.0;
    int n = 0;
    for (auto& p : poses_) if (p.valid && p.tiltDeg > thresholdDeg) ++n;
    return double(n) / poses_.size();
}

void Coverage::draw(cv::Mat& display) const {
    int cw = display.cols / gridX_, ch = display.rows / gridY_;

    int busiest = std::max(1, *std::max_element(cellCounts_.begin(), cellCounts_.end()));

    for (int y = 0; y < gridY_; ++y) {
        for (int x = 0; x < gridX_; ++x) {
            cv::Rect cell(x * cw, y * ch, cw, ch);
            int count = cellCounts_[y * gridX_ + x];

            // Empty cells are outlined red - they are the thing to fix.
            // Filled cells get a green tint scaled by how well sampled.
            if (count == 0) {
                cv::rectangle(display, cell, cv::Scalar(0, 0, 255), 2);
            } else {
                double strength = double(count) / busiest;
                cv::Mat roi = display(cell);
                cv::Mat tint(roi.size(), roi.type(), cv::Scalar(0, 180, 0));
                cv::addWeighted(roi, 1.0 - 0.25 * strength, tint, 0.25 * strength, 0, roi);
                cv::rectangle(display, cell, cv::Scalar(60, 60, 60), 1);
            }
        }
    }

    // Tilt histogram, bottom left: 0-60 degrees in 10 degree bins.
    const int bins = 6, binW = 24, histH = 80;
    std::vector<int> hist(bins, 0);
    for (auto& p : poses_)
        if (p.valid) ++hist[std::clamp(int(p.tiltDeg / 10.0), 0, bins - 1)];

    int tallest = std::max(1, *std::max_element(hist.begin(), hist.end()));
    int baseX = 20, baseY = display.rows - 30;

    for (int i = 0; i < bins; ++i) {
        int h = hist[i] * histH / tallest;
        // Bins below 20 degrees are the fronto-parallel ones - mark amber.
        cv::Scalar colour = (i < 2) ? cv::Scalar(0, 180, 255) : cv::Scalar(0, 220, 0);
        cv::rectangle(display,
                      cv::Rect(baseX + i * binW, baseY - h, binW - 3, h),
                      colour, cv::FILLED);
    }
    cv::putText(display, "tilt 0-60 deg", cv::Point(baseX, baseY + 18),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
}

}  // namespace mocap