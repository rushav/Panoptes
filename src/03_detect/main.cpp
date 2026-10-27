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
// that says which corner is which.
//
// The board comes from config/charuco_board.yaml through lib/detect, the
// same path 04_capture uses. --legacy / --no-legacy override the file's
// legacy_pattern for layout tests. A full corner count alone does not prove
// the layout is right; `fit` (lib/detect layoutResidualPx) does.

#include "../../lib/detect.h"

#include <cameralibrary.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

using namespace CameraLibrary;

int main(int argc, char** argv) {
    std::string boardPath = mocap::kBoardConfig;
    int legacy = -1;                        // -1: as the board file says
    std::vector<int> pos;
    for (int i = 1; i < argc; ++i) {
        if      (!strcmp(argv[i], "--legacy"))    legacy = 1;
        else if (!strcmp(argv[i], "--no-legacy")) legacy = 0;
        else if (!strcmp(argv[i], "--board") && i + 1 < argc) boardPath = argv[++i];
        else pos.push_back(std::stoi(argv[i]));
    }
    int wanted   = (pos.size() > 0) ? pos[0] : 33661;
    int exposure = (pos.size() > 1) ? pos[1] : 8133;
    int gain     = (pos.size() > 2) ? pos[2] : 7;
    int fps      = (pos.size() > 3) ? pos[3] : 30;

    mocap::BoardSpec spec;
    try {
        spec = mocap::loadBoardSpec(boardPath);
    } catch (const std::exception& e) {
        fprintf(stderr, "%s\n", e.what());
        return 1;
    }
    bool fromFile = spec.legacyPattern;
    if (legacy >= 0) spec.legacyPattern = legacy == 1;
    printf("%s\n", spec.describe().c_str());
    if (legacy >= 0)
        printf("legacy_pattern %s by %s (file says %s)\n",
               spec.legacyPattern ? "true" : "false",
               legacy ? "--legacy" : "--no-legacy", fromFile ? "true" : "false");
    printf("\n");
    mocap::Detector detector(spec);
    const int maxId = spec.totalMarkers() - 1;

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

        // Both stages (markers, then corners) run in lib/detect.
        mocap::Detection d = detector.detect(gray);

        // Draw on a colour copy so detections are visible.
        cv::Mat display = detector.annotate(gray, d);
        if (!d.markerIds.empty())
            cv::aruco::drawDetectedMarkers(display, d.markerCorners, d.markerIds);

        // IDs beyond the board's range are misreads or another target.
        int outOfRange = 0, highest = -1;
        for (int id : d.markerIds) {
            if (id > maxId) ++outOfRange;
            highest = std::max(highest, id);
        }

        double meanBrightness = cv::mean(gray)[0];

        char status[200];
        snprintf(status, sizeof(status),
                 "markers %2d/%d   corners %2zu/%d   ids>%d %d (max %d)   fit %.1f px   mean %.1f",
                 d.markersFound, spec.totalMarkers(), d.corners.size(),
                 spec.totalCorners(), maxId, outOfRange, highest,
                 detector.layoutResidualPx(d), meanBrightness);
        cv::putText(display, status, cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
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