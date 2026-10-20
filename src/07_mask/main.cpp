// Per-camera blocking masks: learn static IR interference (other cameras'
// illumination rings), apply it on-camera, verify, persist, and test that
// the working volume is still covered.
//
// A MASK IS PERMANENT BLINDNESS for that image region. A marker passing
// behind it disappears from that camera. So masks are kept as tight as the
// glare allows, and they are valid only while the camera does not move:
// bumped or re-aimed means relearn. --coverage is the acceptance test.
//
//   mask              learn (EMPTY volume) -> apply -> verify -> save
//   mask --load       apply saved masks -> verify
//   mask --show       apply saved masks; tile view with masks tinted
//   mask --coverage   apply saved masks; walk one marker through the volume
//   mask --probe      measure SDK mask behaviour on this hardware
//   mask --clear      remove masks from every camera
//   --config DIR      config directory (default: config/ in the source tree)

#include "../../lib/camera.h"

#include <opencv2/opencv.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using clk = std::chrono::steady_clock;

static double secs(clk::time_point t0) {
    return std::chrono::duration<double>(clk::now() - t0).count();
}

// ---------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------

struct Config {
    fs::path dir;
    mocap::Settings cam;
    double startDelay, learnS, persistFrac, matchR, warnCoverage, verifyS;
    int dilate;
    int minObservers, staleMs, logMs, nearCells;
    int tileW, grayExposure, grayGain;
    fs::path masks, logs;
};

static Config loadConfig(const fs::path& dir) {
    cv::FileStorage f((dir / "mask.yaml").string(), cv::FileStorage::READ);
    if (!f.isOpened()) throw std::runtime_error("cannot read " + (dir / "mask.yaml").string());
    // A missing key reads as 0 and would silently mean "no margin" or
    // "zero-second window". Refuse instead.
    auto need = [&](const char* sec, const char* key) {
        cv::FileNode n = f[sec][key];
        if (n.empty()) throw std::runtime_error(std::string("mask.yaml: missing ") + sec + "." + key);
        return n;
    };
    Config c;
    c.dir = dir;
    c.cam.mode        = mocap::Mode::Object;
    c.cam.exposure    = int(need("camera", "exposure_us"));
    c.cam.gain        = int(need("camera", "gain"));
    c.cam.intensity   = int(need("camera", "intensity"));
    c.cam.threshold   = int(need("camera", "threshold"));
    c.cam.frameRate   = int(need("camera", "frame_rate_hz"));
    c.cam.irFilter    = int(need("camera", "ir_filter")) != 0;
    c.cam.emptyFrames = true;  // a frame with no blobs must still count as a frame
    c.startDelay   = double(need("learn", "start_delay_s"));
    c.learnS       = double(need("learn", "window_s"));
    c.persistFrac  = double(need("learn", "persist_fraction"));
    c.matchR       = double(need("learn", "match_radius_px"));
    c.dilate       = int(need("learn", "dilate_cells"));
    c.warnCoverage = double(need("learn", "warn_coverage"));
    c.verifyS      = double(need("verify", "window_s"));
    c.minObservers = int(need("coverage", "min_observers"));
    c.staleMs      = int(need("coverage", "stale_ms"));
    c.logMs        = int(need("coverage", "log_interval_ms"));
    c.nearCells    = int(need("coverage", "near_mask_cells"));
    c.tileW        = int(need("show", "tile_width_px"));
    c.grayExposure = int(need("show", "grayscale_exposure_us"));
    c.grayGain     = int(need("show", "grayscale_gain"));
    c.masks = dir / std::string(need("paths", "masks"));
    c.logs  = dir / std::string(need("paths", "logs"));
    return c;
}

// ---------------------------------------------------------------------
// Cameras
// ---------------------------------------------------------------------

struct View {
    std::unique_ptr<mocap::Camera> cam;
    int serial = 0, imgW = 0, imgH = 0;
    mocap::Camera::MaskGrid grid;
    cv::Mat mask;                   // CV_8U grid.rows x grid.cols, empty = none
    std::vector<mocap::Blob> blobs; // newest received frame
    cv::Mat gray;
    clk::time_point lastFrame;

    bool masked(float x, float y, int margin = 0) const {
        if (mask.empty()) return false;
        int cx = int(x) / grid.cellPx, cy = int(y) / grid.cellPx;
        for (int dy = -margin; dy <= margin; ++dy)
            for (int dx = -margin; dx <= margin; ++dx) {
                int u = cx + dx, w = cy + dy;
                if (u >= 0 && w >= 0 && u < mask.cols && w < mask.rows && mask.at<uchar>(w, u))
                    return true;
            }
        return false;
    }
};

static void settle(std::vector<View>& views, int ms) {
    auto t0 = clk::now();
    while (secs(t0) * 1000 < ms) {
        for (auto& v : views) v.cam->grabBlobs();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

static std::vector<View> openAll(const mocap::Settings& s) {
    std::vector<View> views;
    for (auto& info : mocap::discover()) {
        if (!info.initialized) { printf("  %d not initialized, skipping\n", info.serial); continue; }
        try {
            View v;
            v.cam = std::make_unique<mocap::Camera>(info.serial);
            v.serial = info.serial;
            v.cam->start(s);
            v.imgW = v.cam->width();
            v.imgH = v.cam->height();
            v.grid = v.cam->maskGrid();
            views.push_back(std::move(v));
        } catch (const std::exception& e) {
            printf("  %d failed: %s\n", info.serial, e.what());
        }
    }
    if (views.empty()) throw std::runtime_error("no cameras opened");
    settle(views, 1000);

    printf("\n  serial  mode        exposure gain thresh  mask grid\n");
    for (auto& v : views) {
        auto a = v.cam->actual();
        printf("  %6d  %-10s  %8d %4d %6d  %dx%d cells of %d px (sensor %dx%d)\n",
               v.serial,
               !v.cam->sawAnyFrame() ? "NO FRAMES" : v.cam->modeIsObject() ? "object" : "WRONG MODE",
               a.exposure, a.gain, a.threshold,
               v.grid.cols, v.grid.rows, v.grid.cellPx, v.imgW, v.imgH);
        // The cell -> pixel mapping assumes cols*cellPx covers the sensor
        // exactly or with one partial cell. Anything else means the grid is
        // not what we think it is, and a mask built on it lands elsewhere.
        if (v.grid.cellPx <= 0 ||
            v.grid.cols != (v.imgW + v.grid.cellPx - 1) / v.grid.cellPx ||
            v.grid.rows != (v.imgH + v.grid.cellPx - 1) / v.grid.cellPx)
            throw std::runtime_error("camera " + std::to_string(v.serial) +
                                     ": mask grid does not tile the sensor; refusing to mask");
    }
    printf("\n");
    return views;
}

// Poll every camera for `seconds`, calling fn(view) once per RECEIVED
// object-mode frame with v.blobs set. Empty polls are not frames.
template <class F>
static void record(std::vector<View>& views, double seconds, const char* what, F fn) {
    auto t0 = clk::now();
    int shown = -1;
    for (double t; (t = secs(t0)) < seconds;) {
        if (int(t) != shown) { shown = int(t); printf("\r  %s %3d / %.0f s", what, shown, seconds); fflush(stdout); }
        for (auto& v : views) {
            auto b = v.cam->grabBlobs();
            if (v.cam->gotFrame() && v.cam->modeIsObject()) { v.blobs = std::move(b); fn(v); }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("\r  %s done%24s\n", what, "");
}

// ---------------------------------------------------------------------
// Learning: which blob positions are static
// ---------------------------------------------------------------------

// One bright source tracked across frames. Matching on position rather
// than on mask cells, because a centroid jittering across a cell border
// would otherwise split one persistent source into two that each fall
// under the threshold.
struct Source {
    double sx = 0, sy = 0;
    int n = 0, hits = 0, lastFrame = -1;
    float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f, maxArea = 0;
    float mx() const { return float(sx / n); }
    float my() const { return float(sy / n); }
};

struct Tally {
    int frames = 0, framesWithBlobs = 0;
    long blobs = 0;
    std::vector<Source> sources;

    void add(const std::vector<mocap::Blob>& bs, double r) {
        for (auto& b : bs) {
            Source* best = nullptr;
            double bestD = r * r;
            for (auto& s : sources) {
                double dx = s.mx() - b.x, dy = s.my() - b.y, d = dx * dx + dy * dy;
                if (d <= bestD) { bestD = d; best = &s; }
            }
            if (!best) { sources.emplace_back(); best = &sources.back(); }
            Source& s = *best;
            s.sx += b.x; s.sy += b.y; ++s.n;
            s.x0 = std::min(s.x0, b.x); s.x1 = std::max(s.x1, b.x);
            s.y0 = std::min(s.y0, b.y); s.y1 = std::max(s.y1, b.y);
            s.maxArea = std::max(s.maxArea, b.area);
            if (s.lastFrame != frames) { ++s.hits; s.lastFrame = frames; }
        }
        blobs += long(bs.size());
        if (!bs.empty()) ++framesWithBlobs;
        ++frames;
    }
    std::vector<const Source*> persistent(double frac) const {
        std::vector<const Source*> p;
        for (auto& s : sources)
            if (frames > 0 && s.hits > frac * frames) p.push_back(&s);
        return p;
    }
    double perFrame() const { return frames ? double(blobs) / frames : 0; }
};

// Cover each source's full observed extent plus its blob radius, then
// dilate. Pixel -> cell is integer division by cellPx, the same mapping
// the SDK uses internally (CalcMaskSpace, IsSoftwareMasked).
static cv::Mat buildMask(const std::vector<const Source*>& ps, const View& v, int dilate) {
    const int g = v.grid.cellPx;
    cv::Mat m(v.grid.rows, v.grid.cols, CV_8UC1, cv::Scalar(0));
    auto cell = [g](float px, int n) { return std::clamp(int(std::floor(px / g)), 0, n - 1); };
    for (auto* s : ps) {
        float rad = std::sqrt(s->maxArea / float(CV_PI));
        int cx0 = cell(s->x0 - rad, m.cols), cx1 = cell(s->x1 + rad, m.cols);
        int cy0 = cell(s->y0 - rad, m.rows), cy1 = cell(s->y1 + rad, m.rows);
        m(cv::Rect(cx0, cy0, cx1 - cx0 + 1, cy1 - cy0 + 1)).setTo(1);
    }
    if (dilate > 0)
        cv::dilate(m, m, cv::getStructuringElement(cv::MORPH_RECT, {2 * dilate + 1, 2 * dilate + 1}));
    return m;
}

static double coverage(const cv::Mat& m) {
    return m.empty() ? 0.0 : double(cv::countNonZero(m)) / double(m.total());
}

// Persistent sources in `after` that sit on a masked cell. With the mask
// committed these should be impossible; one means the mask is not where
// we think it is (mapping) or never reached the camera (commit).
static int leaks(const Tally& after, const View& v, double frac) {
    int n = 0;
    for (auto* s : after.persistent(frac)) if (v.masked(s->mx(), s->my())) ++n;
    return n;
}

// ---------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------

static std::string isoNow(std::time_t t) {
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", std::localtime(&t));
    return buf;
}

static fs::path maskPath(const Config& c, int serial) {
    return c.masks / (std::to_string(serial) + ".json");
}

static void saveMask(const Config& c, const View& v, const Tally& learn,
                     int persistent, const Tally& verify) {
    fs::create_directories(c.masks);
    auto a = v.cam->actual();
    std::time_t now = std::time(nullptr);
    cv::FileStorage f(maskPath(c, v.serial).string(),
                      cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    f << "serial" << v.serial
      << "learned_at" << isoNow(now)
      << "learned_unix" << double(now)
      << "note" << "Valid only while this camera has not moved. Bumped or re-aimed: relearn."
      << "grid" << "{" << "cell_px" << v.grid.cellPx << "cols" << v.grid.cols
                       << "rows" << v.grid.rows << "image_w" << v.imgW << "image_h" << v.imgH << "}"
      << "settings" << "{" << "exposure_us" << a.exposure << "gain" << a.gain
                           << "intensity" << a.intensity << "threshold" << a.threshold
                           << "frame_rate_hz" << a.frameRate << "ir_filter" << int(a.irFilter) << "}"
      << "learn" << "{" << "window_s" << c.learnS << "frames" << learn.frames
                        << "persist_fraction" << c.persistFrac << "match_radius_px" << c.matchR
                        << "dilate_cells" << c.dilate << "persistent_sources" << persistent << "}"
      << "verify" << "{" << "frames" << verify.frames << "blobs_per_frame" << verify.perFrame()
                         << "residual_persistent" << int(verify.persistent(c.persistFrac).size()) << "}"
      << "versions" << "{" << "camera_sdk" << mocap::sdkVersion() << "opencv" << CV_VERSION << "}"
      << "cells_set" << cv::countNonZero(v.mask)
      << "coverage" << coverage(v.mask);
    f << "cells" << "[";
    for (int y = 0; y < v.mask.rows; ++y)
        for (int x = 0; x < v.mask.cols; ++x)
            if (v.mask.at<uchar>(y, x)) f << "[:" << x << y << "]";
    f << "]";
}

// Load into v.mask. Returns false (mask left empty) if absent or unusable.
static bool loadMask(const Config& c, View& v) {
    fs::path p = maskPath(c, v.serial);
    if (!fs::exists(p)) { printf("  %d: no saved mask (%s)\n", v.serial, p.c_str()); return false; }
    cv::FileStorage f(p.string(), cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    cv::FileNode g = f["grid"];
    // Grid mismatch is fatal for this camera: the same cell indices would
    // cover different pixels.
    if (int(g["cell_px"]) != v.grid.cellPx || int(g["cols"]) != v.grid.cols ||
        int(g["rows"]) != v.grid.rows) {
        printf("  %d: REFUSED - saved grid %dx%d/%dpx, camera %dx%d/%dpx\n", v.serial,
               int(g["cols"]), int(g["rows"]), int(g["cell_px"]),
               v.grid.cols, v.grid.rows, v.grid.cellPx);
        return false;
    }
    cv::Mat m(v.grid.rows, v.grid.cols, CV_8UC1, cv::Scalar(0));
    for (auto n : f["cells"]) {
        int x = int(n[0]), y = int(n[1]);
        if (x < 0 || y < 0 || x >= m.cols || y >= m.rows) {
            printf("  %d: REFUSED - cell (%d,%d) outside grid\n", v.serial, x, y);
            return false;
        }
        m.at<uchar>(y, x) = 1;
    }
    v.mask = m;

    double age = std::difftime(std::time(nullptr), std::time_t(double(f["learned_unix"]))) / 86400;
    printf("  %d: %d cells (%.1f%%), learned %s (%.1f days ago)\n", v.serial,
           cv::countNonZero(m), 100 * coverage(m), std::string(f["learned_at"]).c_str(), age);

    cv::FileNode s = f["settings"];
    auto a = v.cam->actual();
    std::string diff;
    auto cmp = [&](const char* k, int now) {
        if (int(s[k]) != now) diff += "  " + std::string(k) + " " + std::to_string(int(s[k])) +
                                      " -> " + std::to_string(now);
    };
    cmp("exposure_us", a.exposure); cmp("gain", a.gain); cmp("intensity", a.intensity);
    cmp("threshold", a.threshold);  cmp("frame_rate_hz", a.frameRate);
    cmp("ir_filter", int(a.irFilter));
    if (!diff.empty())
        printf("    WARNING settings differ from learn time:%s\n"
               "    glare that became a blob then may not now, and vice versa - relearn\n", diff.c_str());
    return true;
}

// ---------------------------------------------------------------------
// Reports
// ---------------------------------------------------------------------

static void reportLearn(const std::vector<View>& views, const std::map<int, Tally>& t, const Config& c) {
    printf("\n  serial  frames  blobs/frame  persistent  cells set  of frame\n");
    for (auto& v : views) {
        auto& tl = t.at(v.serial);
        double cov = coverage(v.mask);
        printf("  %6d  %6d  %11.2f  %10zu  %9d  %7.1f%%%s\n", v.serial, tl.frames, tl.perFrame(),
               tl.persistent(c.persistFrac).size(), cv::countNonZero(v.mask), 100 * cov,
               cov > c.warnCoverage ? "  <-- TOO MUCH" : "");
        if (tl.frames == 0) printf("          NO FRAMES during learn - this camera is not masked\n");
    }
    for (auto& v : views)
        if (coverage(v.mask) > c.warnCoverage)
            printf("\n  %d masks over %.0f%% of its frame. That is a mounting problem, not\n"
                   "  normal cross-camera glare: it is looking straight at other rings.\n"
                   "  Downtilt or re-aim it and relearn rather than blinding it.\n",
                   v.serial, 100 * c.warnCoverage);
}

static void reportVerify(const std::vector<View>& views, const std::map<int, Tally>* before,
                         const std::map<int, Tally>& after, const Config& c) {
    printf("\n  serial   %s after: blobs/frame  persistent  leaks\n", before ? "before: blobs/frame  persistent " : "");
    bool clean = true;
    for (auto& v : views) {
        auto& a = after.at(v.serial);
        if (before) {
            auto& b = before->at(v.serial);
            printf("  %6d   %19.2f  %10zu ", v.serial, b.perFrame(), b.persistent(c.persistFrac).size());
        } else printf("  %6d  ", v.serial);
        int lk = leaks(a, v, c.persistFrac);
        size_t res = a.persistent(c.persistFrac).size();
        printf("  %18.2f  %10zu  %5d%s\n", a.perFrame(), res, lk, a.frames ? "" : "  NO FRAMES");
        if (lk) printf("          LEAK: static blobs on masked cells - mask not applied where intended\n");
        clean &= res == 0 && a.frames > 0;
    }
    printf(clean ? "\n  residual: none. Volume reads empty on every camera.\n"
                 : "\n  residual persistent blobs remain. With an empty volume that is new\n"
                   "  interference, or a camera that moved since learning: relearn it.\n");
}

// ---------------------------------------------------------------------
// Tile display, shared by --show and --coverage
// ---------------------------------------------------------------------

static void drawTile(cv::Mat& canvas, cv::Rect r, const View& v, bool grayscale,
                     const std::string& status) {
    cv::Mat tile = canvas(r);
    tile.setTo(cv::Scalar(0, 0, 0));
    if (grayscale && !v.gray.empty()) {
        cv::Mat small;
        cv::resize(v.gray, small, r.size(), 0, 0, cv::INTER_AREA);
        cv::cvtColor(small, tile, cv::COLOR_GRAY2BGR);
    }
    // Tint masked cells by mapping each tile pixel back to a sensor pixel
    // and then to its cell - the same path a blob takes, so the tint is
    // exactly where the camera is blind.
    if (!v.mask.empty())
        for (int ty = 0; ty < r.height; ++ty) {
            int cy = (ty * v.imgH / r.height) / v.grid.cellPx;
            auto* row = tile.ptr<cv::Vec3b>(ty);
            for (int tx = 0; tx < r.width; ++tx) {
                int cx = (tx * v.imgW / r.width) / v.grid.cellPx;
                if (v.mask.at<uchar>(cy, cx)) row[tx] = row[tx] / 2 + cv::Vec3b(0, 0, 110);
            }
        }
    const int rad = std::max(2, r.width / 120);
    for (auto& b : v.blobs) {
        cv::Point p(int(b.x * r.width / v.imgW), int(b.y * r.height / v.imgH));
        // A blob on a masked cell should not exist; draw it loud.
        if (v.masked(b.x, b.y)) cv::circle(tile, p, rad * 3, cv::Scalar(0, 0, 255), 2);
        cv::circle(tile, p, rad, cv::Scalar(255, 255, 255), cv::FILLED);
    }
    const double fs = r.width / 400.0;
    cv::putText(tile, std::to_string(v.serial), {int(10 * fs), int(30 * fs)},
                cv::FONT_HERSHEY_SIMPLEX, 0.9 * fs, cv::Scalar(0, 255, 0), std::max(1, int(2 * fs)));
    cv::putText(tile, status, {int(10 * fs), int(52 * fs)},
                cv::FONT_HERSHEY_SIMPLEX, 0.5 * fs, cv::Scalar(200, 200, 200), 1);
    cv::rectangle(canvas, r, cv::Scalar(70, 70, 70), 1);
}

struct Grid {
    int cols = 5, tileW, tileH, hudH;
    cv::Mat canvas;
    Grid(const std::vector<View>& views, int tw) : tileW(tw) {
        tileH = tw * views[0].imgH / views[0].imgW;
        hudH = tileH / 6;
        int rows = (int(views.size()) + cols - 1) / cols;
        canvas = cv::Mat(rows * tileH + hudH, cols * tileW, CV_8UC3);
    }
    cv::Rect cell(size_t i) const { return {int(i) % cols * tileW, int(i) / cols * tileH, tileW, tileH}; }
    void hud(const std::string& s, cv::Scalar colour) {
        cv::Mat band = canvas(cv::Rect(0, canvas.rows - hudH, canvas.cols, hudH));
        band.setTo(cv::Scalar(16, 16, 16));
        cv::putText(band, s, {10, hudH * 7 / 10}, cv::FONT_HERSHEY_SIMPLEX, hudH / 45.0, colour,
                    std::max(1, hudH / 25));
    }
};

static const char* kWin = "masks";

static void openWindow(const Grid& g) {
    cv::namedWindow(kWin, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(kWin, g.canvas.cols, g.canvas.rows);
}

// ---------------------------------------------------------------------
// Modes
// ---------------------------------------------------------------------

static void applyLoaded(std::vector<View>& views, const Config& c) {
    printf("loading masks from %s\n", c.masks.c_str());
    for (auto& v : views) {
        if (loadMask(c, v)) v.cam->setMask(v.mask);
        else { v.mask.release(); v.cam->clearMask(); printf("    %d runs UNMASKED\n", v.serial); }
    }
    settle(views, 500);
}

static void verify(std::vector<View>& views, const Config& c, const std::map<int, Tally>* before) {
    std::map<int, Tally> after;
    for (auto& v : views) after[v.serial];
    record(views, c.verifyS, "verify", [&](View& v) { after[v.serial].add(v.blobs, c.matchR); });
    reportVerify(views, before, after, c);
}

static void learn(std::vector<View>& views, const Config& c) {
    // A mask retained from an earlier session would hide the very glare
    // we are trying to find, and the SDK cannot tell us one is there.
    for (auto& v : views) { v.cam->clearMask(); v.mask.release(); }
    settle(views, 500);

    printf("LEARN: the volume must be EMPTY of markers - anything reflective in\n"
           "view for the whole window gets masked, and that is permanent blindness.\n");
    for (int i = int(c.startDelay); i > 0; --i) {
        printf("\r  starting in %d s ", i); fflush(stdout);
        settle(views, 1000);
    }
    printf("\n");

    std::map<int, Tally> before;
    for (auto& v : views) before[v.serial];
    record(views, c.learnS, "learn", [&](View& v) { before[v.serial].add(v.blobs, c.matchR); });

    std::map<int, int> nPersistent;
    for (auto& v : views) {
        auto p = before[v.serial].persistent(c.persistFrac);
        nPersistent[v.serial] = int(p.size());
        if (before[v.serial].frames > 0) v.mask = buildMask(p, v, c.dilate);
    }
    reportLearn(views, before, c);

    printf("\napplying\n");
    for (auto& v : views) if (!v.mask.empty()) v.cam->setMask(v.mask);
    settle(views, 500);

    std::map<int, Tally> after;
    for (auto& v : views) after[v.serial];
    record(views, c.verifyS, "verify", [&](View& v) { after[v.serial].add(v.blobs, c.matchR); });
    reportVerify(views, &before, after, c);

    for (auto& v : views)
        if (!v.mask.empty()) saveMask(c, v, before[v.serial], nPersistent[v.serial], after[v.serial]);
    printf("\nsaved to %s\n"
           "Masks are valid only while no camera moves. Bumped or re-aimed: relearn.\n"
           "Acceptance test: mask --coverage, walking one marker through the volume.\n",
           c.masks.c_str());
}

// Measure what the SDK documentation does not say. Run with an empty
// volume; each step reports per camera how many of the learned static
// sources are still visible.
static void probe(std::vector<View>& views, const Config& c) {
    const double w = 3.0;
    printf("PROBE  camera SDK %s, %zu cameras, empty volume required\n", mocap::sdkVersion().c_str(),
           views.size());
    for (auto& v : views)
        printf("  %d  grid %dx%d of %d px   SDK enabled flag at start: %s\n", v.serial,
               v.grid.cols, v.grid.rows, v.grid.cellPx, v.cam->maskEnabled() ? "on" : "off");

    auto pass = [&](const char* what) {
        std::map<int, Tally> t;
        for (auto& v : views) t[v.serial];
        record(views, w, what, [&](View& v) { t[v.serial].add(v.blobs, c.matchR); });
        return t;
    };
    // How many of `ref`'s persistent sources are still persistent now.
    auto visible = [&](const Tally& ref, const Tally& now) {
        int n = 0;
        auto cur = now.persistent(c.persistFrac);
        for (auto* s : ref.persistent(c.persistFrac))
            for (auto* q : cur)
                if (std::hypot(s->mx() - q->mx(), s->my() - q->my()) <= c.matchR) { ++n; break; }
        return n;
    };

    // 1. Before touching the mask: if the camera kept one from a previous
    //    process (or across a power cycle), sources are missing here that
    //    reappear in step 2.
    auto untouched = pass("1 untouched");
    for (auto& v : views) v.cam->clearMask();
    settle(views, 500);
    auto cleared = pass("2 cleared  ");

    for (auto& v : views)
        v.mask = buildMask(cleared[v.serial].persistent(c.persistFrac), v, c.dilate);

    // 3-5. Stage, enable, commit - separately, to see which one makes the
    //      camera act on the mask.
    bool roundTrip = true;
    for (auto& v : views) {
        v.cam->stageMask(v.mask);
        cv::Mat back = v.cam->stagedMask();
        roundTrip &= cv::countNonZero(back != v.mask) == 0;
    }
    settle(views, 300);
    auto staged = pass("3 staged   ");
    for (auto& v : views) v.cam->enableMask(true);
    settle(views, 300);
    auto enabled = pass("4 enabled  ");
    for (auto& v : views) v.cam->commitMask();
    settle(views, 300);
    auto committed = pass("5 committed");
    // 6. Is enable a live toggle on a committed mask? --show relies on it.
    for (auto& v : views) v.cam->enableMask(false);
    settle(views, 300);
    auto disabled = pass("6 disabled ");
    for (auto& v : views) v.cam->enableMask(true);
    settle(views, 300);
    auto reenabled = pass("7 enabled  ");

    printf("\n  static sources visible at each step (of those found in step 2)\n");
    printf("  serial  cells  untouched cleared staged enabled committed disabled re-enabled leaks\n");
    for (auto& v : views) {
        auto& ref = cleared[v.serial];
        printf("  %6d  %5d  %9d %7zu %6d %7d %9d %8d %10d %5d\n", v.serial, cv::countNonZero(v.mask),
               visible(ref, untouched[v.serial]), ref.persistent(c.persistFrac).size(),
               visible(ref, staged[v.serial]), visible(ref, enabled[v.serial]),
               visible(ref, committed[v.serial]), visible(ref, disabled[v.serial]),
               visible(ref, reenabled[v.serial]), leaks(reenabled[v.serial], v, c.persistFrac));
    }
    printf("\n  staged readback matches what was staged: %s\n", roundTrip ? "yes" : "NO");
    printf("\n  Reading it:\n"
           "   commit needed     staged/enabled ~= cleared, committed ~= 0\n"
           "   enable is live    disabled ~= cleared, re-enabled ~= 0\n"
           "   mapping correct   committed ~= 0 and leaks = 0\n"
           "   retained mask     untouched < cleared. The mask is LEFT APPLIED on exit:\n"
           "                     rerun --probe to test a process restart, then power-cycle\n"
           "                     the cameras and rerun to test a camera restart.\n");
}

static void show(std::vector<View>& views, const Config& c, mocap::Settings s) {
    applyLoaded(views, c);
    Grid g(views, c.tileW);
    openWindow(g);
    bool maskOn = true;
    printf("\n  m mask on/off   g grayscale   o object   q quit\n");
    while (true) {
        for (auto& v : views) {
            if (s.mode == mocap::Mode::Object) {
                auto b = v.cam->grabBlobs();
                if (v.cam->gotFrame() && v.cam->modeIsObject()) v.blobs = std::move(b);
            } else {
                cv::Mat img = v.cam->grab();
                if (v.cam->gotFrame() && !img.empty()) v.gray = img;
            }
        }
        for (size_t i = 0; i < views.size(); ++i) {
            auto& v = views[i];
            char st[96];
            snprintf(st, sizeof(st), "%.1f%% masked  %zu blobs", 100 * coverage(v.mask), v.blobs.size());
            drawTile(g.canvas, g.cell(i), v, s.mode == mocap::Mode::Grayscale, st);
        }
        g.hud(std::string(maskOn ? "mask ON" : "mask OFF - showing what it hides") +
              (s.mode == mocap::Mode::Grayscale ? "   grayscale" : "   object") +
              "   red = masked, ringed blob = on a masked cell",
              maskOn ? cv::Scalar(255, 200, 0) : cv::Scalar(0, 140, 255));
        cv::imshow(kWin, g.canvas);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q') break;
        if (key == 'm') {
            maskOn = !maskOn;
            for (auto& v : views) if (!v.mask.empty()) v.cam->enableMask(maskOn);
        }
        if (key == 'g' && s.mode != mocap::Mode::Grayscale) {
            s.mode = mocap::Mode::Grayscale; s.exposure = c.grayExposure; s.gain = c.grayGain;
            for (auto& v : views) { v.cam->apply(s); v.blobs.clear(); }
        }
        if (key == 'o' && s.mode != mocap::Mode::Object) {
            s = c.cam;
            for (auto& v : views) { v.cam->apply(s); v.gray.release(); }
        }
    }
    if (!maskOn) for (auto& v : views) if (!v.mask.empty()) v.cam->enableMask(true);
}

// Acceptance test: one marker walked through the volume with masks on.
// There is no 3D yet, so "where" is reported as time since start plus
// each observing camera's image coordinates - enough to find the spot
// again, and to see which camera's mask it was behind.
static void coverageWalk(std::vector<View>& views, const Config& c) {
    applyLoaded(views, c);
    fs::create_directories(c.logs);
    std::time_t now = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    fs::path logPath = c.logs / (std::string("coverage_") + stamp + ".csv");
    std::ofstream log(logPath);
    log << "t_s,observers";
    for (auto& v : views) log << ",x_" << v.serial << ",y_" << v.serial;
    log << "\n";

    struct Track {
        bool seen = false;
        cv::Point2f last{-1, -1};
        int losses = 0, nearMask = 0;
    };
    std::map<int, Track> tr;
    auto t0 = clk::now();
    for (auto& v : views) v.lastFrame = t0 - std::chrono::hours(1);

    bool started = false;
    int minObs = int(views.size()) + 1, prevObs = -1;
    double minT = 0, belowS = 0, lastT = 0, lastLog = -1;
    std::string minWhere;

    Grid g(views, c.tileW);
    openWindow(g);
    printf("\nCOVERAGE: walk ONE marker slowly through the whole volume, low to high,\n"
           "into the corners. q to finish.\n\n");

    while (true) {
        double t = secs(t0);
        for (auto& v : views) {
            auto b = v.cam->grabBlobs();
            if (v.cam->gotFrame() && v.cam->modeIsObject()) { v.blobs = std::move(b); v.lastFrame = clk::now(); }
        }
        // A camera observes the marker if its newest frame is fresh and
        // has a blob. More than one blob on a camera is a reflection or
        // residual glare; the first is taken as the marker.
        int obs = 0;
        for (auto& v : views) {
            bool fresh = std::chrono::duration<double, std::milli>(clk::now() - v.lastFrame).count() < c.staleMs;
            bool sees = fresh && !v.blobs.empty();
            Track& k = tr[v.serial];
            if (sees) { k.last = {v.blobs[0].x, v.blobs[0].y}; ++obs; }
            if (started && k.seen && !sees) {
                ++k.losses;
                if (v.masked(k.last.x, k.last.y, c.nearCells)) {
                    ++k.nearMask;
                    printf("  %6.1f s  %d lost marker at (%.0f, %.0f) NEXT TO ITS MASK\n",
                           t, v.serial, k.last.x, k.last.y);
                }
            }
            k.seen = sees;
        }
        if (!started && obs > 0) { started = true; printf("  marker seen, walk started\n"); }

        if (started) {
            if (obs < c.minObservers) belowS += t - lastT;
            if (obs < c.minObservers && prevObs >= c.minObservers)
                printf("  %6.1f s  WARNING only %d camera(s) see the marker\n", t, obs);
            if (obs < minObs) {
                minObs = obs; minT = t; minWhere.clear();
                for (auto& v : views) {
                    auto& k = tr[v.serial];
                    char buf[96];
                    if (k.seen) snprintf(buf, sizeof(buf), "    %d sees it at (%.0f, %.0f)\n", v.serial, k.last.x, k.last.y);
                    else if (k.last.x >= 0)
                        snprintf(buf, sizeof(buf), "    %d lost it, last at (%.0f, %.0f)%s\n", v.serial,
                                 k.last.x, k.last.y, v.masked(k.last.x, k.last.y, c.nearCells) ? " - next to its mask" : "");
                    else snprintf(buf, sizeof(buf), "    %d has not seen it\n", v.serial);
                    minWhere += buf;
                }
            }
            if (t - lastLog >= c.logMs / 1000.0) {
                lastLog = t;
                log << t << "," << obs;
                for (auto& v : views) {
                    auto& k = tr[v.serial];
                    if (k.seen) log << "," << k.last.x << "," << k.last.y; else log << ",,";
                }
                log << "\n";
            }
            prevObs = obs;
        }
        lastT = t;

        for (size_t i = 0; i < views.size(); ++i)
            drawTile(g.canvas, g.cell(i), views[i], false, tr[views[i].serial].seen ? "SEES" : "-");
        char hud[128];
        snprintf(hud, sizeof(hud), "%d camera(s) see the marker   minimum so far %s",
                 obs, started ? std::to_string(minObs).c_str() : "-");
        g.hud(hud, obs < c.minObservers ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 220, 0));
        cv::imshow(kWin, g.canvas);
        if ((cv::waitKey(1) & 0xFF) == 'q') break;
    }

    printf("\n--- coverage summary ---\n");
    if (!started) { printf("  marker never seen\n"); return; }
    printf("  walk %.0f s, below %d observers for %.1f s (%.0f%%)\n", lastT, c.minObservers, belowS,
           100 * belowS / std::max(lastT, 1e-9));
    printf("  minimum %d observer(s) at %.1f s:\n%s", minObs, minT, minWhere.c_str());
    printf("  losses per camera (next to mask / total):\n");
    for (auto& v : views)
        printf("    %d  %d / %d%s\n", v.serial, tr[v.serial].nearMask, tr[v.serial].losses,
               tr[v.serial].nearMask ? "  <-- mask is costing coverage" : "");
    printf("  log: %s\n", logPath.c_str());
    if (minObs < c.minObservers)
        printf("  FAIL: the volume has spots seen by fewer than %d cameras. If the losses\n"
               "  sit next to masks, tighten those masks (or re-aim) and relearn.\n", c.minObservers);
    else printf("  PASS\n");
}

int main(int argc, char** argv) {
    std::string mode = "learn";
    fs::path cfgDir = MOCAP_CONFIG_DIR;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) cfgDir = argv[++i];
        else if (a == "--load" || a == "--show" || a == "--coverage" || a == "--probe" || a == "--clear")
            mode = a.substr(2);
        else { printf("usage: mask [--load|--show|--coverage|--probe|--clear] [--config DIR]\n"); return 1; }
    }
    try {
        Config c = loadConfig(cfgDir);
        auto views = openAll(c.cam);
        if (mode == "learn")         learn(views, c);
        else if (mode == "load")     { applyLoaded(views, c); verify(views, c, nullptr); }
        else if (mode == "show")     show(views, c, c.cam);
        else if (mode == "coverage") coverageWalk(views, c);
        else if (mode == "probe")    probe(views, c);
        else if (mode == "clear")    { for (auto& v : views) v.cam->clearMask(); printf("masks cleared\n"); }
    } catch (const std::exception& e) {
        printf("error: %s\n", e.what());
        return 1;
    }
    return 0;
}
