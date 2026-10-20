// Intrinsics capture: live view with detection, coverage tracking, and
// capture to a session folder.
//
// Acceptance criterion is ZERO EMPTY CELLS, not RMS. See coverage.h.

#include "../../lib/camera.h"
#include "../../lib/detect.h"
#include "../../lib/coverage.h"
#include "../../lib/session.h"

#include <opencv2/opencv.hpp>
#include <cstdio>

int main(int argc, char** argv) {
    int serial = (argc > 1) ? std::stoi(argv[1]) : 33661;

    mocap::BoardSpec board;                 // defaults: 9x6, 90mm, 67mm
    mocap::Detector detector(board);

    mocap::Camera cam(serial);
    mocap::Settings settings;
    cam.start(settings);

    cv::Size imageSize(cam.width(), cam.height());
    mocap::Coverage coverage(imageSize);

    std::string root = std::string(getenv("HOME")) + "/panoptes-data/sessions";
    mocap::Session session(root, serial, board);

    printf("%s  %dx%d\n", cam.name().c_str(), cam.width(), cam.height());
    printf("session: %s\n\n", session.path().c_str());
    printf("  space  capture      u  undo last\n");
    printf("  + -    exposure     [ ]  gain\n");
    printf("  q      quit\n\n");

    while (true) {
        cv::Mat gray = cam.grab();
        if (gray.empty()) { cv::waitKey(5); continue; }

        mocap::Detection d = detector.detect(gray);
        cv::Mat display = detector.annotate(gray, d);
        coverage.draw(display);

        mocap::Settings actual = cam.actual();

        char line1[200], line2[200];
        snprintf(line1, sizeof(line1),
                 "corners %2zu/%d   captures %d   empty cells %d/%d",
                 d.corners.size(), board.totalCorners(),
                 coverage.captureCount(),
                 coverage.emptyCells(), coverage.totalCells());
        snprintf(line2, sizeof(line2),
                 "exp %d/%d  gain %d  tilted %.0f%%",
                 actual.exposure, cam.maxExposure(), actual.gain,
                 coverage.tiltedFraction() * 100.0);

        // Green once coverage is complete - the actual stopping signal.
        cv::Scalar colour = coverage.emptyCells() == 0
                          ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255);

        cv::putText(display, line1, cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, colour, 2);
        cv::putText(display, line2, cv::Point(20, 75),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200, 200, 200), 2);

        cv::Mat small;
        cv::resize(display, small, cv::Size(), 0.7, 0.7);
        cv::imshow("intrinsics capture", small);

        int key = cv::waitKey(1);
        if (key == 'q') break;

        if (key == ' ') {
            // Only accept complete boards. A partial view still carries
            // valid corners, but mixing partial and full captures makes
            // per-view error harder to read.
            if (!d.complete(board)) {
                printf("skipped: only %zu/%d corners\n",
                       d.corners.size(), board.totalCorners());
            } else {
                auto pose = coverage.add(d, board);
                session.record(gray, d, actual);
                printf("captured %d  tilt %.0f deg  dist %.2f m  empty cells %d\n",
                       session.count(), pose.tiltDeg, pose.distanceMM / 1000.0,
                       coverage.emptyCells());
            }
        }

        if (key == 'u' && session.count() > 0) {
            coverage.removeLast();
            session.removeLast();
            printf("undid last, %d remain\n", session.count());
        }

        if (key == '+' || key == '=') {
            settings.exposure = std::min(settings.exposure + 1000, cam.maxExposure());
            cam.apply(settings);
        }
        if (key == '-') {
            settings.exposure = std::max(settings.exposure - 1000, 10);
            cam.apply(settings);
        }
        if (key == ']') { settings.gain = std::min(settings.gain + 1, 7); cam.apply(settings); }
        if (key == '[') { settings.gain = std::max(settings.gain - 1, 0); cam.apply(settings); }
    }

    session.flush();
    printf("\n%d frames in %s\n", session.count(), session.path().c_str());
    printf("empty cells: %d/%d   tilted: %.0f%%\n",
           coverage.emptyCells(), coverage.totalCells(),
           coverage.tiltedFraction() * 100.0);
    return 0;
}