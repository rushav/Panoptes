// Object-mode acquisition diagnostic. ONE camera, console only, no window.
//
// Answers the questions an fps counter cannot. A tile viewer showing
// "0.0 fps, 0 blobs" conflates three different failures:
//
//   the camera is sending nothing at all
//   the camera is sending frames that carry no objects
//   the camera is sending grayscale frames while we ask for objects
//
// So this prints, for every frame actually received: the frame ID (gaps
// are dropped frames, which an fps average hides), the FrameType the
// frame carried, its ObjectCount, and the camera's own readback of
// threshold and exposure - not what we asked for. "polls" is how many
// times we asked and got nothing, which at 30 Hz should be most of them.
//
// --sweep steps the threshold across the range the CAMERA reports and
// records the peak ObjectCount at each step. "zero blobs at every
// threshold" is a claim worth having data behind.
//
// Usage: objdiag <serial> [exposure] [threshold] [gain] [fps] [--sweep]

#include "../../lib/camera.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

using clk = std::chrono::steady_clock;

static double secs(clk::time_point a, clk::time_point b) {
    return std::chrono::duration<double>(b - a).count();
}

static const char* typeName(const mocap::Camera::FrameStats& f) {
    return f.object ? "Object" : "NOT-OBJECT";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("usage: objdiag <serial> [exposure] [threshold] [gain] [fps] [--sweep]\n");
        return 1;
    }
    bool sweep = false;
    for (int i = 1; i < argc; ++i)
        if (!strcmp(argv[i], "--sweep")) sweep = true;

    const int serial = std::stoi(argv[1]);

    mocap::Settings s;
    s.mode        = mocap::Mode::Object;
    s.exposure    = (argc > 2 && argv[2][0] != '-') ? std::stoi(argv[2]) : 500;
    s.threshold   = (argc > 3 && argv[3][0] != '-') ? std::stoi(argv[3]) : 200;
    s.gain        = (argc > 4 && argv[4][0] != '-') ? std::stoi(argv[4]) : 0;
    s.frameRate   = (argc > 5 && argv[5][0] != '-') ? std::stoi(argv[5]) : 30;
    s.intensity   = 15;
    s.emptyFrames = true;   // the whole point: see the cadence even with no markers

    mocap::Camera cam(serial);
    printf("\ncamera %d\n", serial);

    cam.start(s);

    // The mode change is asynchronous, so anything read immediately
    // after start() describes the camera mid-transition.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    printf("\n--- SDK state after start() ---\n%s", cam.diagnostics().c_str());

    const int tMin = cam.minThreshold(), tMax = cam.maxThreshold();
    printf("\nasked: exposure %d  threshold %d  gain %d  %d Hz\n",
           s.exposure, s.threshold, s.gain, s.frameRate);
    printf("camera says: exposure %d (ceiling %d)  threshold %d (range %d-%d)\n",
           cam.actual().exposure, cam.maxExposure(),
           cam.actual().threshold, tMin, tMax);
    if (s.threshold < tMin || s.threshold > tMax)
        printf("  *** requested threshold is OUTSIDE the camera's range ***\n");

    // -----------------------------------------------------------------
    // Threshold sweep
    // -----------------------------------------------------------------
    if (sweep) {
        printf("\n--- threshold sweep, %d-%d, 1.5 s each ---\n", tMin, tMax);
        printf("thresh  readback  frames  empty  peak_objects\n");

        const int steps = 12;
        for (int i = 0; i <= steps; ++i) {
            int t = tMin + (tMax - tMin) * i / steps;
            cam.setThreshold(t);

            // Discard whatever is already queued at the old threshold.
            auto settle = clk::now();
            while (secs(settle, clk::now()) < 0.3) cam.grabBlobs();

            int frames = 0, empty = 0, peak = 0;
            auto t0 = clk::now();
            while (secs(t0, clk::now()) < 1.5) {
                cam.grabBlobs();
                auto f = cam.lastFrame();
                if (!f.received) continue;
                ++frames;
                if (f.objects == 0) ++empty;
                if (f.objects > peak) peak = f.objects;
            }
            printf("%6d  %8d  %6d  %5d  %12d\n",
                   t, cam.actual().threshold, frames, empty, peak);
            fflush(stdout);
        }
        printf("\nsweep done\n");
        return 0;
    }

    // -----------------------------------------------------------------
    // Per-frame log
    // -----------------------------------------------------------------
    printf("\n--- per-frame log, Ctrl-C to stop ---\n");
    printf("   t(s)   frame#    dF  type         objects  thresh  expos  polls\n");

    auto t0 = clk::now();
    auto lastSummary = t0;
    int lastId = -1, polls = 0, received = 0;

    while (true) {
        auto blobs = cam.grabBlobs();
        auto f = cam.lastFrame();

        if (!f.received) {
            ++polls;
            // Yield rather than spin: a tight NextFrame loop starves the
            // SDK's own receive thread and can itself look like 0 fps.
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else {
            ++received;
            int dF = (lastId < 0) ? 0 : f.frameId - lastId;
            lastId = f.frameId;

            auto a = cam.actual();
            printf("%7.3f  %7d  %4d  %-11s  %7d  %6d  %5d  %5d\n",
                   secs(t0, clk::now()), f.frameId, dF, typeName(f),
                   f.objects, a.threshold, a.exposure, polls);
            polls = 0;

            // Blob geometry, when there is any. Area and roundness are
            // what separates a marker from a glare streak.
            for (auto& b : blobs)
                printf("           blob  x %7.1f  y %7.1f  area %7.1f  round %.3f\n",
                       b.x, b.y, b.area, b.roundness);
            fflush(stdout);
        }

        if (secs(lastSummary, clk::now()) >= 5.0) {
            printf("  [5 s: %d frames received]\n", received);
            if (received == 0)
                printf("  [no frames at all - not a threshold problem]\n");
            received = 0;
            lastSummary = clk::now();
            fflush(stdout);
        }
    }
    return 0;
}
