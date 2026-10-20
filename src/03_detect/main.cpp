// Stage 3: stream live and detect the ChArUco board, reporting marker
// and corner counts continuously so camera settings can be swept.
//
// ChArUco is a hybrid target: a checkerboard with ArUco markers in the
// white squares. Detection runs in two stages.
//
//   1. detectMarkers finds the ArUco squares. Each marker's pattern
//      encodes an ID, so the detector knows WHICH marker it found. That
//      is what makes partial views usable - cover half the board and the
//      visible markers still identify themselves.
//
//   2. interpolateCornersCharuco uses those known IDs to work out where
//      the checkerboard corners must be, then refines each to sub-pixel
//      precision by reading the gray gradient across the black-white edge.
//
// Calibration uses the CORNERS, not the markers. Markers are scaffolding
// that says which corner is which. A 9x6 board has 27 markers, 40 corners.

#include <cameralibrary.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <cstdio>
#include <string>

using namespace CameraLibrary;

// Board geometry. Squares/markers in mm, measured after printing.
const int   SQUARES_X = 9, SQUARES_Y = 6;
const float SQUARE_MM = 90.0f, MARKER_MM = 67.0f;

int main(int argc, char** argv) {
    int wanted   = (argc > 1) ? std::stoi(argv[1]) : 33661;
    int exposure = (argc > 2) ? std::stoi(argv[2]) : 8133;
    int gain     = (argc > 3) ? std::stoi(argv[3]) : 7;
    int fps      = (argc > 4) ? std::stoi(argv[4]) : 30;

    // OpenCV 4.6 API. Note there is no setLegacyPattern call here -
    // 4.6's board layout already IS the legacy one. Only the 5.x Python
    // solver needs that flag. Same board, two code paths.
    auto dict  = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    auto board = cv::aruco::CharucoBoard::create(
        SQUARES_X, SQUARES_Y, SQUARE_MM, MARKER_MM, dict);

    CameraManager::X().WaitForInitialization();

    size_t previous = 0; int stable = 0;
    for (int i = 0; i < 40 && stable < 6; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        CameraList l;
        if (l.Count() == previous && l.Count() > 0) ++stable;
        else { stable = 0; previous = l.Count(); }
    }

    std::shared_ptr<Camera> camera;
    CameraList list;
    for (int i = 0; i < list.Count(); ++i) {
        if (list[i].Serial() == wanted) {
            camera = CameraManager::X().GetCamera(list[i].UID());
            break;
        }
    }
    if (!camera) { printf("camera %d not found\n", wanted); return 1; }

    camera->SetVideoType(Core::GrayscaleMode);
    camera->Start();
    camera->SetVideoType(Core::GrayscaleMode);

    camera->SetExposure(exposure);
    camera->SetIntensity(15);
    camera->SetFrameRate(fps);
        camera->SetImagerGain(static_cast<eImagerGain>(gain));

    printf("streaming: exposure %d, gain %d, %d Hz\n", exposure, gain, fps);
    printf("press q in the image window to quit, s to save the frame\n\n");

    int frames = 0;
    while (true) {
        std::shared_ptr<const Frame> frame;
        while (auto f = camera->NextFrame()) frame = f;
        if (!frame) { std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }
        if (frame->FrameType() != Core::GrayscaleMode) continue;

        const unsigned char* pixels = frame->GrayscaleData(*camera);
        if (!pixels) continue;

        int w = camera->PhysicalPixelWidth(), h = camera->PhysicalPixelHeight();
        if (w != 1280 || h != 1024) {
            printf("unexpected dimensions %d x %d\n", w, h);
            break;
        }

        cv::Mat gray(h, w, CV_8UC1, (void*)pixels);

        // Stage 1: find the ArUco markers and their IDs.
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::aruco::detectMarkers(gray, dict, markerCorners, markerIds);

        // Stage 2: from those, locate and refine the checkerboard corners.
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        if (!markerIds.empty()) {
            cv::aruco::interpolateCornersCharuco(
                markerCorners, markerIds, gray, board,
                charucoCorners, charucoIds);
        }

        // Draw on a colour copy so detections are visible.
        cv::Mat display;
        cv::cvtColor(gray, display, cv::COLOR_GRAY2BGR);
        if (!markerIds.empty())
            cv::aruco::drawDetectedMarkers(display, markerCorners, markerIds);
        if (!charucoIds.empty())
            cv::aruco::drawDetectedCornersCharuco(display, charucoCorners, charucoIds,
                                                  cv::Scalar(0, 0, 255));

        double meanBrightness = cv::mean(gray)[0];

        char status[200];
        snprintf(status, sizeof(status),
                 "markers %2zu/27   corners %2zu/40   mean %.1f",
                 markerIds.size(), charucoIds.size(), meanBrightness);
        cv::putText(display, status, cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 255, 0), 2);

        // Print once a second rather than every frame.
        if (++frames % 8 == 0) printf("%s\n", status);

        cv::Mat small;
        cv::resize(display, small, cv::Size(), 0.7, 0.7);
        cv::imshow("detect", small);

        int key = cv::waitKey(1);
        if (key == 'q') break;
        if (key == 's') { cv::imwrite("detect_frame.png", gray); printf("saved\n"); }
    }

    CameraManager::X().Shutdown();
    return 0;
}