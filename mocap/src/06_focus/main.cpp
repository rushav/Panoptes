// Focus assist: live variance-of-Laplacian with peak hold.
//
// The Laplacian is a second-derivative operator - it responds to how
// sharply intensity changes between neighbouring pixels. A sharp image
// has strong edge transitions and high variance; defocus smears those
// transitions and the variance collapses.
//
// IMPORTANT: this number is NOT comparable between cameras or between
// scenes. It depends on content, contrast and brightness. It is only
// meaningful for finding the PEAK on one camera with the scene held
// still. Do not treat it as a quality threshold.
//
// Focus with the IR bandpass filter IN - a flat glass plate in the
// optical path shifts the focal plane, so focusing without it would
// not transfer to the operating configuration.

#include "../../lib/camera.h"

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <deque>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) { printf("usage: focus <serial>\n"); return 1; }
    int serial = std::stoi(argv[1]);

    mocap::Camera cam(serial);
    mocap::Settings s;              // filter in, IR 15, gain 7, 30 Hz
    cam.start(s);

    printf("camera %d - twist the lens until the number peaks, then lock\n", serial);
    printf("  r  reset peak        q  quit\n\n");

    double peak = 0.0;
    std::deque<double> history;
    const size_t HISTORY = 200;

    while (true) {
        cv::Mat gray = cam.grab();
        if (gray.empty()) { cv::waitKey(5); continue; }

        // Measure on the centre half only. Put the board there - this
        // avoids the frame edges, where lens softness is a property of
        // the lens rather than of focus.
        cv::Rect roi(gray.cols / 4, gray.rows / 4, gray.cols / 2, gray.rows / 2);

        cv::Mat lap, mean, stddev;
        cv::Laplacian(gray(roi), lap, CV_64F);
        cv::meanStdDev(lap, mean, stddev);
        double vol = stddev.at<double>(0) * stddev.at<double>(0);

        if (vol > peak) peak = vol;
        history.push_back(vol);
        if (history.size() > HISTORY) history.pop_front();

        cv::Mat display;
        cv::cvtColor(gray, display, cv::COLOR_GRAY2BGR);
        cv::rectangle(display, roi, cv::Scalar(90, 90, 90), 1);

        char line[128];
        snprintf(line, sizeof(line), "%.0f", vol);
        cv::putText(display, line, cv::Point(30, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 2.4, cv::Scalar(0, 0, 0), 9);
        cv::putText(display, line, cv::Point(30, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 2.4, cv::Scalar(0, 255, 0), 4);

        // Peak hold matters: you twist past the maximum and need to know
        // where it was to come back to it.
        snprintf(line, sizeof(line), "peak %.0f   (%.0f%% of peak)",
                 peak, peak > 0 ? vol / peak * 100.0 : 0.0);
        cv::putText(display, line, cv::Point(30, 130),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(220, 220, 220), 2);

        snprintf(line, sizeof(line), "cam %d   mean %.1f", serial, cv::mean(gray)[0]);
        cv::putText(display, line, cv::Point(30, 165),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(180, 180, 180), 2);

        // Recent history as a strip, so the direction of travel is
        // visible while your hands are on the lens.
        int baseY = display.rows - 40, h = 120;
        double top = std::max(peak, 1.0);
        for (size_t i = 1; i < history.size(); ++i) {
            int x0 = 30 + int(i - 1) * 4, x1 = 30 + int(i) * 4;
            int y0 = baseY - int(history[i - 1] / top * h);
            int y1 = baseY - int(history[i]     / top * h);
            cv::line(display, {x0, y0}, {x1, y1}, cv::Scalar(0, 220, 0), 2);
        }
        cv::line(display, {30, baseY - h}, {30 + int(HISTORY) * 4, baseY - h},
                 cv::Scalar(80, 80, 80), 1);

        cv::Mat small;
        cv::resize(display, small, cv::Size(), 0.7, 0.7);
        cv::imshow("focus", small);

        int key = cv::waitKey(1) & 0xFF;
        if (key != 255) printf("key %d\n", key);        
        if (key == 'q') break;
        if (key == 'r') { peak = 0.0; history.clear(); }
    }
    return 0;
}