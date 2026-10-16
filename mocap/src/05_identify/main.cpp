// Open every camera at once and show a labeled tile grid: identify which
// physical unit is which serial, and check marker visibility across the
// volume.
//
// OBJECT MODE is what the tracking system actually runs on - the camera
// thresholds onboard and returns blob centroids, not pixels. It is also
// what makes 10-camera operation possible: grayscale is ~1.3 MB/frame
// and one camera alone was bandwidth-bound at 8 fps.
//
// Keys: g grayscale, o object, + - threshold, [ ] exposure, d dump, q quit.
//
// Usage: identify [fps] [exposure] [threshold] [gain] [--size WxH]
//   --size overrides the detected monitor resolution (for testing layouts).

#include "../../lib/camera.h"

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <string>
#include <memory>
#include <thread>
#include <vector>

using clk = std::chrono::steady_clock;

constexpr int kSensorW = 1280, kSensorH = 1024;    // blob coordinate space
const cv::Size kFallbackScreen(1920, 1080);

// Primary monitor via xrandr. The X screen spans every monitor, so its size
// is wrong on a multi-head desk. Empty rect if xrandr is unavailable.
cv::Rect primaryMonitor() {
    FILE* p = popen("xrandr --listmonitors 2>/dev/null", "r");
    if (!p) return {};
    cv::Rect first, primary;
    char line[256], name[64];
    int w, h, x, y;
    // e.g. " 0: +*DP-0 2560/340x1600/220+0+0  DP-0"  ('*' marks primary)
    while (fgets(line, sizeof(line), p))
        if (sscanf(line, " %*d: %63s %d/%*dx%d/%*d+%d+%d", name, &w, &h, &x, &y) == 5) {
            if (first.empty()) first = {x, y, w, h};
            if (strchr(name, '*')) primary = {x, y, w, h};
        }
    pclose(p);
    return primary.empty() ? first : primary;
}

struct Layout {
    cv::Size canvas;
    int tileW, tileH, hudH;
    cv::Point origin;   // top-left of the tile grid, centred in the canvas
    float u;            // 1% of tile height; every decoration is sized in u
};

// Largest sensor-aspect tiles that fit `area` above a HUD band.
Layout makeLayout(cv::Size area, int cols, int rows) {
    Layout L;
    L.hudH = std::max(16, area.height / 30);
    // Snap width to a multiple of 5 (for 1280x1024) so tileH is exact and
    // blob positions are not stretched by rounding.
    const int step = kSensorW / std::gcd(kSensorW, kSensorH);
    int w = std::min(area.width / cols,
                     (area.height - L.hudH) / rows * kSensorW / kSensorH);
    L.tileW = std::max(step, w - w % step);
    L.tileH = L.tileW * kSensorH / kSensorW;
    // A window dragged smaller than one step per tile still gets a full
    // grid; it just overflows and the window scales it.
    L.canvas = {std::max(area.width, cols * L.tileW),
                std::max(area.height, rows * L.tileH + L.hudH)};
    L.origin = {(L.canvas.width - cols * L.tileW) / 2,
                (L.canvas.height - L.hudH - rows * L.tileH) / 2};
    L.u = L.tileH / 100.0f;
    return L;
}

// Font scale that renders SIMPLEX text `px` pixels tall.
double fontScale(float px, int thickness) {
    return cv::getFontScaleFromHeight(cv::FONT_HERSHEY_SIMPLEX,
                                      std::max(1, int(px)), thickness);
}

struct View {
    std::unique_ptr<mocap::Camera> cam;
    int serial;
    cv::Mat latest;                     // grayscale mode
    std::vector<mocap::Blob> blobs;     // object mode
    int frames = 0;
    double fps = 0.0;
    clk::time_point lastTick;
};

int main(int argc, char** argv) {
    cv::Size forced;
    std::vector<int> pos;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--size") {
            if (i + 1 >= argc ||
                sscanf(argv[++i], "%dx%d", &forced.width, &forced.height) != 2 ||
                forced.width <= 0 || forced.height <= 0) {
                printf("--size wants WxH, e.g. --size 1920x1080\n");
                return 1;
            }
        } else {
            pos.push_back(std::stoi(argv[i]));
        }
    }
    auto arg = [&](size_t i, int def) { return i < pos.size() ? pos[i] : def; };

    auto found = mocap::discover();
    printf("opening %zu camera(s)\n", found.size());

    // Overridable from the command line so a settings sweep does not
    // need a rebuild. Object mode wants a SHORT exposure - markers are
    // retroreflective and far brighter than the scene - but 10 us is the
    // floor, and the floor is not automatically a working value.
    mocap::Settings s;
    s.mode        = mocap::Mode::Object;
    s.frameRate   = arg(0, 30);
    s.exposure    = arg(1, 500);
    s.threshold   = arg(2, 200);
    s.gain        = arg(3, 0);
    s.intensity   = 15;
    s.emptyFrames = true;   // keep the cadence visible when nothing is lit

    std::vector<View> views;
    for (auto& info : found) {
        if (!info.initialized) {
            printf("  %d not initialized, skipping\n", info.serial);
            continue;
        }
        try {
            View v;
            v.cam = std::make_unique<mocap::Camera>(info.serial);
            v.serial = info.serial;
            v.cam->start(s);
            v.lastTick = clk::now();
            views.push_back(std::move(v));
        } catch (const std::exception& e) {
            printf("  %d failed: %s\n", info.serial, e.what());
        }
    }

    if (views.empty()) { printf("no cameras opened\n"); return 1; }

    // Let the mode change settle, then report what each camera is ACTUALLY
    // delivering. Requested mode and delivered mode can differ.
    for (int i = 0; i < 40; ++i) {
        for (auto& v : views) v.cam->grabBlobs();
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    // Every number here is a READBACK from the camera, not a repeat of
    // what was asked. Printing the request tells you only that the
    // struct was filled in.
    printf("  threshold range %d-%d, exposure ceiling %d\n",
           views[0].cam->minThreshold(), views[0].cam->maxThreshold(),
           views[0].cam->maxExposure());
    for (auto& v : views) {
        auto a = v.cam->actual();
        printf("  %d: %-10s exposure %d (asked %d)  gain %d  threshold %d (asked %d)\n",
               v.serial,
               !v.cam->sawAnyFrame() ? "NO FRAMES"
               : v.cam->modeIsObject() ? "object" : "WRONG MODE",
               a.exposure, s.exposure, a.gain, a.threshold, s.threshold);
    }

    // 'o' returns to the exposure this was launched with, not to a
    // hardcoded floor that may be the reason nothing was visible.
    const int objectExposure = s.exposure;

    const int cols = 5;
    const int rows = (int(views.size()) + cols - 1) / cols;

    cv::Rect mon = !forced.empty() ? cv::Rect({0, 0}, forced) : primaryMonitor();
    if (mon.empty()) mon = cv::Rect({0, 0}, kFallbackScreen);
    // resizeWindow sets the image area; the title bar and desktop panels
    // sit outside it, so hold back a slice of the height for them.
    cv::Size winSize(mon.width, mon.height * 94 / 100);

    // FREERATIO: the image fills the window exactly, so the window's image
    // rect is the canvas size we should render - no letterboxing to undo.
    const char* win = "all cameras";
    cv::namedWindow(win, cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO);
    cv::resizeWindow(win, winSize);
    cv::moveWindow(win, mon.x, mon.y);
    Layout L = makeLayout(winSize, cols, rows);
    printf("monitor %dx%d, tiles %dx%d\n", mon.width, mon.height, L.tileW, L.tileH);

    printf("\n%zu streaming.\n", views.size());
    printf("  g grayscale   o object   + - threshold   [ ] exposure   d dump   q quit\n\n");

    auto lastReport = clk::now();

    while (true) {
        // Re-lay out when the user drags the window. Compared against the
        // last window size, not the canvas, since the canvas can be larger.
        cv::Rect wr = cv::getWindowImageRect(win);
        if (wr.width > 0 && wr.height > 0 && wr.size() != winSize) {
            winSize = wr.size();
            L = makeLayout(winSize, cols, rows);
        }
        const float u = L.u;

        cv::Mat grid(L.canvas, CV_8UC3, cv::Scalar(16, 16, 16));
        int totalBlobs = 0;

        for (size_t i = 0; i < views.size(); ++i) {
            View& v = views[i];

            // gotFrame() distinguishes "no frame ready" from "a frame with
            // nothing in it". Without it, empty polls overwrite the last
            // good blob list and get counted as frames - which inflates fps
            // to the loop rate and makes the display strobe.
            if (s.mode == mocap::Mode::Object) {
                auto blobs = v.cam->grabBlobs();
                if (v.cam->gotFrame() && v.cam->modeIsObject()) {
                    v.blobs = blobs;
                    ++v.frames;
                }
            } else {
                cv::Mat img = v.cam->grab();
                if (v.cam->gotFrame() && !img.empty()) {
                    v.latest = img;
                    ++v.frames;
                }
            }

            auto now = clk::now();
            double dt = std::chrono::duration<double>(now - v.lastTick).count();
            if (dt >= 1.0) {
                v.fps = v.frames / dt;
                v.frames = 0;
                v.lastTick = now;
            }

            int r = int(i) / cols, c = int(i) % cols;
            cv::Rect cell(L.origin.x + c * L.tileW, L.origin.y + r * L.tileH,
                          L.tileW, L.tileH);

            if (s.mode == mocap::Mode::Object) {
                // Centroids drawn on black - what the tracker sees. Radius
                // scaled by reported area, so a barely-resolved distant
                // marker looks different from a near one. Deliberately
                // exaggerated (~4x true size) so single-pixel blobs show.
                cv::rectangle(grid, cell, cv::Scalar(0, 0, 0), cv::FILLED);
                for (auto& b : v.blobs) {
                    int x = cell.x + int(b.x * L.tileW / kSensorW);
                    int y = cell.y + int(b.y * L.tileH / kSensorH);
                    int rad = int(std::clamp(0.4f * u * std::sqrt(b.area), 0.8f * u, 4.0f * u));
                    cv::circle(grid, {x, y}, rad, cv::Scalar(255, 255, 255), cv::FILLED);
                }
                totalBlobs += int(v.blobs.size());
            } else if (!v.latest.empty()) {
                cv::Mat small, colour;
                cv::resize(v.latest, small, cell.size(), 0, 0, cv::INTER_AREA);
                cv::cvtColor(small, colour, cv::COLOR_GRAY2BGR);
                colour.copyTo(grid(cell));
            }

            char label[64];
            snprintf(label, sizeof(label), "%d", v.serial);
            const int thick = std::max(1, int(std::lround(0.8f * u)));
            const double serialScale = fontScale(9 * u, thick);
            const cv::Point serialAt(cell.x + int(4 * u), cell.y + int(13 * u));
            cv::putText(grid, label, serialAt, cv::FONT_HERSHEY_SIMPLEX,
                        serialScale, cv::Scalar(0, 0, 0), thick * 2 + 1);
            cv::putText(grid, label, serialAt, cv::FONT_HERSHEY_SIMPLEX,
                        serialScale, cv::Scalar(0, 255, 0), thick);

            if (s.mode == mocap::Mode::Object)
                // fps and blob count are independent failures now that
                // empty frames arrive: 30 fps with 0 blobs is a lighting
                // or threshold problem, 0 fps is an acquisition problem.
                snprintf(label, sizeof(label), "%.1f fps  %zu blobs",
                         v.fps, v.blobs.size());
            else
                snprintf(label, sizeof(label), "%.1f fps", v.fps);
            const int statThick = std::max(1, int(std::lround(0.4f * u)));
            cv::putText(grid, label, cv::Point(cell.x + int(4 * u), cell.y + int(23 * u)),
                        cv::FONT_HERSHEY_SIMPLEX, fontScale(4.5f * u, statThick),
                        cv::Scalar(200, 200, 200), statThick);

            cv::rectangle(grid, cell, cv::Scalar(70, 70, 70), 1);
        }

        char hud[128];
        snprintf(hud, sizeof(hud), "%s   exposure %d   threshold %d",
                 s.mode == mocap::Mode::Object ? "object" : "grayscale",
                 s.exposure, s.threshold);
        const int hudThick = std::max(1, L.hudH / 20);
        cv::putText(grid, hud,
                    cv::Point(L.origin.x + int(4 * u), grid.rows - L.hudH * 3 / 10),
                    cv::FONT_HERSHEY_SIMPLEX, fontScale(L.hudH * 0.5f, hudThick),
                    cv::Scalar(255, 200, 0), hudThick);

        cv::imshow(win, grid);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q') break;

        if (key == 'd') {
            printf("\n--- blob dump ---\n");
            for (auto& v : views) {
                for (auto& b : v.blobs)
                    printf("cam %d  x %7.1f  y %7.1f  area %7.1f  round %.3f\n",
                           v.serial, b.x, b.y, b.area, b.roundness);
            }
        }

        // Mode switches go through apply(); threshold and exposure use the
        // single-setting calls, because re-applying everything mid-stream
        // stalls delivery.
        if (key == 'g' && s.mode != mocap::Mode::Grayscale) {
            s.mode = mocap::Mode::Grayscale;
            s.exposure = 8133;
            s.gain = 7;
            for (auto& v : views) v.cam->apply(s);
            printf("grayscale mode\n");
        }
        if (key == 'o' && s.mode != mocap::Mode::Object) {
            s.mode = mocap::Mode::Object;
            s.exposure = objectExposure;
            s.gain = 0;
            for (auto& v : views) v.cam->apply(s);
            printf("object mode\n");
        }
        if (key == '+' || key == '=') {
            s.threshold = std::min(s.threshold + 10, views[0].cam->maxThreshold());
            for (auto& v : views) v.cam->setThreshold(s.threshold);
            printf("threshold %d\n", s.threshold);
        }
        if (key == '-') {
            s.threshold = std::max(s.threshold - 10, views[0].cam->minThreshold());
            for (auto& v : views) v.cam->setThreshold(s.threshold);
            printf("threshold %d\n", s.threshold);
        }
        if (key == ']') {
            s.exposure = std::min(s.exposure + 100, 33133);
            for (auto& v : views) v.cam->setExposure(s.exposure);
            printf("exposure %d\n", s.exposure);
        }
        if (key == '[') {
            s.exposure = std::max(s.exposure - 100, 10);
            for (auto& v : views) v.cam->setExposure(s.exposure);
            printf("exposure %d\n", s.exposure);
        }

        auto now = clk::now();
        if (std::chrono::duration<double>(now - lastReport).count() >= 1.0) {
            double total = 0;
            for (auto& v : views) total += v.fps;
            printf("total %.1f fps, %d blobs\n", total, totalBlobs);
            lastReport = now;
        }
    }
    return 0;
}