// Stage 2: open one camera, stream grayscale, save a frame as PNG.
//
// Grayscale mode (not object mode) because calibration needs the actual
// checkerboard picture, not a list of bright blobs.
//
// This SDK hands back std::shared_ptr for cameras and frames, so buffers
// are freed automatically when the last reference goes away. No manual
// Release() calls.

#include <cameralibrary.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <cstdio>
#include <string>

using namespace CameraLibrary;

int main(int argc, char** argv) {
    int wanted = (argc > 1) ? std::stoi(argv[1]) : 33661;

    CameraManager::X().WaitForInitialization();

    // Staggered discovery: wait until the device count stops growing
    // rather than trusting the first "ready" signal.
    size_t previous = 0; int stable = 0;
    for (int i = 0; i < 40 && stable < 6; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        CameraList list;
        if (list.Count() == previous && list.Count() > 0) ++stable;
        else { stable = 0; previous = list.Count(); }
    }

    // Find our camera by serial and open it.
    std::shared_ptr<Camera> camera;
    CameraList list;
    for (int i = 0; i < list.Count(); ++i) {
        if (list[i].Serial() == wanted) {
            camera = CameraManager::X().GetCamera(list[i].UID());
            break;
        }
    }
    if (!camera) { printf("camera %d not found\n", wanted); return 1; }

    // PhysicalPixelWidth/Height are the pixel counts. ImagerWidth/Height
    // are doubles - physical sensor size in mm. Width/Height describe a
    // region-of-interest crop, not the full sensor.
    printf("opened %s, %d x %d\n", camera->Name(),
           camera->PhysicalPixelWidth(), camera->PhysicalPixelHeight());

    // Ask for grayscale, then start streaming.
    camera->SetVideoType(Core::GrayscaleMode);
    camera->Start();

    // SetVideoType is asynchronous: the camera reports the new mode
    // immediately, but frames already queued still carry the old one.
    // Reassert after Start(), then verify per-frame below.
    camera->SetVideoType(Core::GrayscaleMode);

    camera->SetExposure(8133);
    camera->SetIntensity(15);      // IR LED ring, 0-15
    camera->SetFrameRate(30);

    int saved = 0, discarded = 0;
    for (int attempt = 0; attempt < 300 && saved < 1; ++attempt) {

        // NextFrame() returns the OLDEST queued frame. Called once per
        // loop you fall steadily behind, so drain and keep the newest.
        // Reassigning `frame` drops the previous reference, freeing it.
        // Note the const: NextFrame() hands back a read-only Frame.
        std::shared_ptr<const Frame> frame;
        while (auto f = camera->NextFrame()) {
            frame = f;
        }
        if (!frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        // Trust nothing until the mode is confirmed. GrayscaleData() on
        // an object-mode frame returns null.
        if (frame->FrameType() != Core::GrayscaleMode) {
            ++discarded;
            continue;
        }

        // GrayscaleData needs the camera: decoding depends on resolution
        // and mode, which the frame alone doesn't carry.
        const unsigned char* pixels = frame->GrayscaleData(*camera);
        if (!pixels) continue;

        int w = camera->PhysicalPixelWidth(), h = camera->PhysicalPixelHeight();

        // Guard before building the Mat. A wrong size here segfaults;
        // a slightly wrong one would silently produce a skewed image,
        // which is worse.
        if (w != 1280 || h != 1024) {
            printf("unexpected dimensions %d x %d - refusing to build Mat\n", w, h);
            break;
        }

        // Wrap the SDK's buffer in a Mat. CV_8UC1 = 8-bit, 1 channel.
        // This does NOT copy - the Mat points at SDK memory, valid only
        // while `frame` is alive. Clone it if you keep it past the loop.
        cv::Mat img(h, w, CV_8UC1, (void*)pixels);

        printf("frame %d: %dx%d, mean brightness %.1f (discarded %d stale)\n",
               frame->FrameID(), w, h, cv::mean(img)[0], discarded);

        cv::imwrite("frame.png", img);
        ++saved;
    }

    CameraManager::X().Shutdown();
    printf(saved ? "\nwrote frame.png\n" : "\nno grayscale frames arrived\n");
    return saved ? 0 : 1;
}