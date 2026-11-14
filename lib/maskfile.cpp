#include "maskfile.h"

#include <filesystem>
#include <stdexcept>

namespace fs = std::filesystem;

namespace mocap {

MaskConfig loadMaskConfig(const std::string& configDir) {
    const std::string path = (fs::path(configDir) / "mask.yaml").string();
    cv::FileStorage f(path, cv::FileStorage::READ);
    if (!f.isOpened()) throw std::runtime_error("cannot read " + path);
    // A missing key reads as 0, which would silently mean exposure 0.
    auto need = [&](const char* sec, const char* key) {
        cv::FileNode n = f[sec][key];
        if (n.empty()) throw std::runtime_error(std::string("mask.yaml: missing ") + sec + "." + key);
        return n;
    };
    MaskConfig c;
    c.camera.mode        = Mode::Object;
    c.camera.exposure    = int(need("camera", "exposure_us"));
    c.camera.gain        = int(need("camera", "gain"));
    c.camera.intensity   = int(need("camera", "intensity"));
    c.camera.threshold   = int(need("camera", "threshold"));
    c.camera.frameRate   = int(need("camera", "frame_rate_hz"));
    c.camera.irFilter    = int(need("camera", "ir_filter")) != 0;
    c.camera.emptyFrames = true;
    c.masksDir = (fs::path(configDir) / std::string(need("paths", "masks"))).string();
    return c;
}

MaskFile loadMaskFile(const std::string& path) {
    MaskFile m;
    auto fail = [&](const std::string& why) { m.ok = false; m.error = why; return m; };
    if (!fs::exists(path)) return fail("no saved mask (" + path + ")");
    cv::FileStorage f;
    try {
        f.open(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    } catch (const cv::Exception& e) {
        return fail("unreadable: " + std::string(e.what()));
    }
    if (!f.isOpened()) return fail("cannot open " + path);

    cv::FileNode g = f["grid"], s = f["settings"], cells = f["cells"];
    if (g.empty() || s.empty() || f["serial"].empty() || cells.isNone())
        return fail("missing serial, grid, settings or cells");
    m.serial = int(f["serial"]);
    m.cellPx = int(g["cell_px"]);
    m.cols   = int(g["cols"]);
    m.rows   = int(g["rows"]);
    if (m.cellPx <= 0 || m.cols <= 0 || m.rows <= 0) return fail("grid has a non-positive size");
    m.settings.mode      = Mode::Object;
    m.settings.exposure  = int(s["exposure_us"]);
    m.settings.gain      = int(s["gain"]);
    m.settings.intensity = int(s["intensity"]);
    m.settings.threshold = int(s["threshold"]);
    m.settings.frameRate = int(s["frame_rate_hz"]);
    m.settings.irFilter  = int(s["ir_filter"]) != 0;
    if (!f["learned_at"].empty()) m.learnedAt = std::string(f["learned_at"]);

    m.cells = cv::Mat(m.rows, m.cols, CV_8UC1, cv::Scalar(0));
    for (auto n : cells) {
        int x = int(n[0]), y = int(n[1]);
        if (x < 0 || y < 0 || x >= m.cols || y >= m.rows)
            return fail("cell (" + std::to_string(x) + "," + std::to_string(y) + ") outside grid");
        m.cells.at<uchar>(y, x) = 1;
    }
    m.cellsSet = cv::countNonZero(m.cells);
    m.ok = true;
    return m;
}

std::string maskSettingsDiff(const Settings& learned, const Settings& now) {
    std::string d;
    auto cmp = [&](const char* k, int was, int is) {
        if (was != is) d += std::string(d.empty() ? "" : ", ") + k + " " + std::to_string(was) +
                            " -> " + std::to_string(is);
    };
    cmp("exposure_us", learned.exposure, now.exposure);
    cmp("gain", learned.gain, now.gain);
    cmp("intensity", learned.intensity, now.intensity);
    cmp("threshold", learned.threshold, now.threshold);
    cmp("frame_rate_hz", learned.frameRate, now.frameRate);
    return d;
}

}  // namespace mocap
