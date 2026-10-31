#include "capture_status.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace mocap {

std::string utcNowIso() {
    using namespace std::chrono;
    auto now = system_clock::now();
    std::time_t secs = system_clock::to_time_t(now);
    int ms = (int)(duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000);
    std::tm tm{};
    gmtime_r(&secs, &tm);
    char buf[40];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
    char out[48];
    std::snprintf(out, sizeof(out), "%s.%03dZ", buf, ms);
    return out;
}

// JSON number, or null for "not measured" (negative sentinel or NaN).
static std::string num(double v, int places, bool negativeIsNull = true) {
    if (std::isnan(v) || (negativeIsNull && v < 0)) return "null";
    std::ostringstream os;
    os << std::fixed << std::setprecision(places) << v;
    return os.str();
}

static std::string quoted(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out + "\"";
}

std::string captureStatusJson(const StatusMeta& meta, const Settings& actual,
                              const Detection& d, const BoardSpec& board,
                              const Decision& dec, const AutoCapture& ac,
                              const Coverage& coverage) {
    const AutoConfig& cfg = ac.config();
    std::ostringstream os;
    os << "{\n"
       << "  \"schema\": " << quoted(kStatusSchema) << ",\n"
       << "  \"timestamp_utc\": " << quoted(meta.utc) << ",\n"
       << "  \"t_ms\": " << num(meta.tMs, 1) << ",\n"
       << "  \"frame\": " << meta.frame << ",\n"
       << "  \"camera_serial\": " << meta.serial << ",\n"
       << "  \"dry_run\": " << (meta.dryRun ? "true" : "false") << ",\n"
       << "  \"session_path\": "
       << (meta.sessionPath.empty() ? std::string("null") : quoted(meta.sessionPath)) << ",\n";

    os << "  \"settings\": {\"exposure\": " << actual.exposure
       << ", \"gain\": " << actual.gain
       << ", \"intensity\": " << actual.intensity
       << ", \"frame_rate\": " << actual.frameRate
       << ", \"ir_filter\": " << (actual.irFilter ? "true" : "false") << "},\n";

    const cv::Size img = coverage.imageSize();
    os << "  \"image\": {\"width\": " << img.width << ", \"height\": " << img.height << "},\n";
    os << "  \"markers\": {\"found\": " << d.markersFound
       << ", \"total\": " << board.totalMarkers() << "},\n";
    os << "  \"corners\": {\"found\": " << d.corners.size()
       << ", \"total\": " << board.totalCorners()
       << ", \"complete\": " << (d.complete(board) ? "true" : "false") << "},\n";
    os << "  \"fit_px\": " << num(dec.fitPx, 3) << ",\n";

    // Board geometry from whatever corners were found. On a partial board
    // the centroid is of the visible part only; check corners.complete.
    if (d.corners.empty()) {
        os << "  \"board\": null,\n";
    } else {
        double sx = 0, sy = 0;
        float x0 = std::numeric_limits<float>::max(), y0 = x0, x1 = -x0, y1 = -x0;
        for (const auto& p : d.corners) {
            sx += p.x; sy += p.y;
            x0 = std::min(x0, p.x); y0 = std::min(y0, p.y);
            x1 = std::max(x1, p.x); y1 = std::max(y1, p.y);
        }
        const double n = (double)d.corners.size();
        const double sq = apparentSquarePx(d, board);
        os << "  \"board\": {\"centroid_px\": [" << num(sx / n, 2, false) << ", "
           << num(sy / n, 2, false) << "]"
           << ", \"bbox_px\": [" << num(x0, 2, false) << ", " << num(y0, 2, false) << ", "
           << num(x1, 2, false) << ", " << num(y1, 2, false) << "]"
           << ", \"square_px\": " << (sq > 0 ? num(sq, 3) : "null")
           << ", \"roll_deg\": " << num(boardRollDeg(d, board), 3, false) << "},\n";
    }

    // Gate 2 passed: the verdict got past "moving".
    const bool still = dec.verdict != Verdict::NotFull && dec.verdict != Verdict::BadFit &&
                       dec.verdict != Verdict::NoReference && dec.verdict != Verdict::Moving;
    os << "  \"motion\": {\"median_px\": " << num(dec.motionMedPx, 4)
       << ", \"max_px\": " << num(dec.motionMaxPx, 4)
       << ", \"still_px\": " << num(cfg.stillPx, 3)
       << ", \"still_max_px\": " << num(cfg.stillMaxPx, 3)
       << ", \"still\": " << (still ? "true" : "false") << "},\n";

    os << "  \"decision\": {\"verdict\": " << quoted(verdictName(dec.verdict))
       << ", \"reason\": " << quoted(dec.describe(cfg))
       << ", \"bin\": " << quoted(dec.bin.name()) << "},\n";

    os << "  \"coverage\": {\"grid\": {\"cols\": " << coverage.gridX()
       << ", \"rows\": " << coverage.gridY()
       << ", \"cell_width_px\": " << num((double)img.width / coverage.gridX(), 3)
       << ", \"cell_height_px\": " << num((double)img.height / coverage.gridY(), 3)
       << ", \"id\": \"row * cols + col\"}"
       << ", \"empty_cells\": [";
    auto empty = coverage.emptyCellIds();
    for (size_t i = 0; i < empty.size(); ++i) os << (i ? ", " : "") << empty[i];
    os << "], \"empty_count\": " << empty.size()
       << ", \"total_cells\": " << coverage.totalCells() << "},\n";

    os << "  \"pose_bins\": {\"min_per_bin\": " << cfg.minPerBin << ", \"counts\": {";
    for (int b = 0; b < kBands; ++b)
        for (int t = 0; t < kTiltBins; ++t)
            os << (b || t ? ", " : "") << "\"" << bandName(b) << "/" << tiltBinName(t)
               << "\": " << ac.binCount(t, b);
    os << "}, \"short\": " << ac.binsShort() << "},\n";

    os << "  \"accepted\": " << coverage.captureCount() << ",\n"
       << "  \"complete\": " << (ac.complete(coverage) ? "true" : "false") << "\n"
       << "}\n";
    return os.str();
}

void writeFileAtomic(const std::string& path, const std::string& text) {
    const std::string tmp = path + ".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        if (!out) throw std::runtime_error("status file: cannot write " + tmp);
        out << text;
        out.flush();
        if (!out) throw std::runtime_error("status file: write failed on " + tmp);
    }
    if (std::rename(tmp.c_str(), path.c_str()) != 0)
        throw std::runtime_error("status file: cannot rename " + tmp + " to " + path);
}

}  // namespace mocap
