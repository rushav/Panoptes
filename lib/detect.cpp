#include "detect.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mocap {

// ---------------------------------------------------------------- config

static int dictionaryFromName(const std::string& name) {
    using namespace cv::aruco;
    static const std::pair<const char*, int> table[] = {
        {"DICT_4X4_50", DICT_4X4_50},     {"DICT_4X4_100", DICT_4X4_100},
        {"DICT_4X4_250", DICT_4X4_250},   {"DICT_4X4_1000", DICT_4X4_1000},
        {"DICT_5X5_50", DICT_5X5_50},     {"DICT_5X5_100", DICT_5X5_100},
        {"DICT_5X5_250", DICT_5X5_250},   {"DICT_5X5_1000", DICT_5X5_1000},
        {"DICT_6X6_50", DICT_6X6_50},     {"DICT_6X6_100", DICT_6X6_100},
        {"DICT_6X6_250", DICT_6X6_250},   {"DICT_6X6_1000", DICT_6X6_1000},
        {"DICT_7X7_50", DICT_7X7_50},     {"DICT_7X7_100", DICT_7X7_100},
        {"DICT_7X7_250", DICT_7X7_250},   {"DICT_7X7_1000", DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", DICT_ARUCO_ORIGINAL},
    };
    for (auto& [n, v] : table)
        if (name == n) return v;
    return -1;
}

BoardSpec loadBoardSpec(const std::string& path) {
    cv::FileStorage fs;
    try {
        fs.open(path, cv::FileStorage::READ);
    } catch (const cv::Exception&) {}
    if (!fs.isOpened())
        throw std::runtime_error("board config: cannot open " + path +
                                 " (run from the repo root, or pass --board)");

    auto fail = [&](const std::string& what) {
        return std::runtime_error("board config " + path + ": " + what);
    };
    auto integer = [&](const cv::FileNode& n, const std::string& key) {
        if (!n.isInt()) throw fail(key + " is missing or not an integer");
        return (int)n;
    };
    auto number = [&](const cv::FileNode& n, const std::string& key) {
        if ((!n.isInt() && !n.isReal()) || (double)n <= 0)
            throw fail(key + " is missing, blank or not a positive number");
        return (float)(double)n;
    };
    auto text = [&](const cv::FileNode& n) {
        return n.isString() ? (std::string)n : std::string();
    };

    BoardSpec b;
    b.source   = path;
    b.name     = text(fs["name"]);
    b.squaresY = integer(fs["rows"], "rows");
    b.squaresX = integer(fs["cols"], "cols");
    if (b.squaresX < 2 || b.squaresY < 2) throw fail("rows and cols must be >= 2");

    // measured is required; nominal is a record only and never used in
    // its place.
    b.squareMM = number(fs["square_mm"]["measured"], "square_mm.measured");
    b.markerMM = number(fs["marker_mm"]["measured"], "marker_mm.measured");
    if (!fs["square_mm"]["nominal"].empty())
        b.squareNominalMM = number(fs["square_mm"]["nominal"], "square_mm.nominal");
    if (!fs["marker_mm"]["nominal"].empty())
        b.markerNominalMM = number(fs["marker_mm"]["nominal"], "marker_mm.nominal");
    if (b.markerMM >= b.squareMM) throw fail("marker_mm must be smaller than square_mm");

    b.dictionaryName = text(fs["dictionary"]);
    b.dictionary = dictionaryFromName(b.dictionaryName);
    if (b.dictionary < 0) throw fail("unknown dictionary '" + b.dictionaryName + "'");
    int dictSize = cv::aruco::getPredefinedDictionary(b.dictionary)->bytesList.rows;
    if (b.totalMarkers() > dictSize)
        throw fail(b.dictionaryName + " has too few markers for this board");

    // OpenCV 4.6 reads `false` as a string, 5.x as an int; accept both.
    cv::FileNode lp = fs["legacy_pattern"];
    std::string lps = text(lp);
    if (lp.isInt() && ((int)lp == 0 || (int)lp == 1)) b.legacyPattern = (int)lp == 1;
    else if (lps == "true" || lps == "false")         b.legacyPattern = lps == "true";
    else throw fail("legacy_pattern must be true or false");

    cv::FileNode pv = fs["provenance"];
    b.vendor       = text(pv["vendor"]);
    b.orderDate    = text(pv["order_date"]);
    b.material     = text(pv["material"]);
    b.patternMM    = text(pv["pattern_mm"]);
    b.plateMM      = text(pv["plate_mm"]);
    b.measuredBy   = text(pv["measured_by"]);
    b.measuredDate = text(pv["measured_date"]);
    return b;
}

static std::string mm(float v) {
    std::ostringstream os;
    os << v;
    return os.str();
}

std::string BoardSpec::describe() const {
    auto orBlank = [](const std::string& s) { return s.empty() ? "?" : s; };
    std::ostringstream os;
    os << "board " << name << " from " << source << ": "
       << squaresX << " cols x " << squaresY << " rows, square " << mm(squareMM)
       << " mm (nominal " << (squareNominalMM > 0 ? mm(squareNominalMM) : "?")
       << "), marker " << mm(markerMM)
       << " mm (nominal " << (markerNominalMM > 0 ? mm(markerNominalMM) : "?")
       << "), " << dictionaryName
       << ", legacy_pattern " << (legacyPattern ? "true" : "false")
       << " -> " << totalMarkers() << " markers (ids 0-" << totalMarkers() - 1
       << "), " << totalCorners() << " corners; vendor " << orBlank(vendor)
       << ", measured by " << orBlank(measuredBy) << " on " << orBlank(measuredDate);
    return os.str();
}

static std::string quoted(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        if ((unsigned char)c < 0x20) continue;
        out += c;
    }
    return out + "\"";
}

std::string BoardSpec::toJson(const std::string& in) const {
    auto nominal = [](float v) { return v > 0 ? mm(v) : std::string("null"); };
    std::ostringstream os;
    os << "{\n"
       << in << "  \"source\": " << quoted(source) << ",\n"
       << in << "  \"name\": " << quoted(name) << ",\n"
       << in << "  \"rows\": " << squaresY << ",\n"
       << in << "  \"cols\": " << squaresX << ",\n"
       << in << "  \"square_mm\": {\"nominal\": " << nominal(squareNominalMM)
             << ", \"measured\": " << mm(squareMM) << "},\n"
       << in << "  \"marker_mm\": {\"nominal\": " << nominal(markerNominalMM)
             << ", \"measured\": " << mm(markerMM) << "},\n"
       << in << "  \"dictionary\": " << quoted(dictionaryName) << ",\n"
       << in << "  \"legacy_pattern\": " << (legacyPattern ? "true" : "false") << ",\n"
       << in << "  \"provenance\": {\n"
       << in << "    \"vendor\": " << quoted(vendor) << ",\n"
       << in << "    \"order_date\": " << quoted(orderDate) << ",\n"
       << in << "    \"material\": " << quoted(material) << ",\n"
       << in << "    \"pattern_mm\": " << quoted(patternMM) << ",\n"
       << in << "    \"plate_mm\": " << quoted(plateMM) << ",\n"
       << in << "    \"measured_by\": " << quoted(measuredBy) << ",\n"
       << in << "    \"measured_date\": " << quoted(measuredDate) << "\n"
       << in << "  }\n"
       << in << "}";
    return os.str();
}

// ---------------------------------------------------------------- board

// OpenCV 4.6 has no setLegacyPattern, and its create() always builds what
// 4.7+ calls the NON-legacy layout. For an even row count the legacy layout
// differs (markers on the other colour of square), and 4.6 interpolation
// against the wrong layout still returns a full corner count, with IDs on
// the wrong corners. So place the markers here by the 4.7+ rule and
// recompute the nearest-marker tables that create() derived from them.
// Checked against 5.0's layout for 9x6 and 12x9, both flag values.
static void applyLayout(cv::aruco::CharucoBoard& board, const BoardSpec& spec) {
    const float s = spec.squareMM, m = spec.markerMM, inset = (s - m) / 2;
    const bool flip = spec.legacyPattern && spec.squaresY % 2 == 0;

    board.objPoints.clear();
    for (int y = 0; y < spec.squaresY; ++y) {
        for (int x = 0; x < spec.squaresX; ++x) {
            bool black = flip ? (y + 1) % 2 == x % 2 : y % 2 == x % 2;
            if (black) continue;
            float x0 = x * s + inset, y0 = y * s + inset;
            board.objPoints.push_back({{x0, y0, 0}, {x0 + m, y0, 0},
                                       {x0 + m, y0 + m, 0}, {x0, y0 + m, 0}});
        }
    }
    board.ids.resize(board.objPoints.size());
    for (size_t i = 0; i < board.ids.size(); ++i) board.ids[i] = (int)i;

    // Same algorithm as 4.6's private _getNearestMarkerCorners(): for each
    // corner, the markers whose centres are nearest (ties within 1% of a
    // square), and for each of those, its marker corner nearest to it.
    const double tie = std::pow(0.01 * s, 2);
    const size_t nCorners = board.chessboardCorners.size();
    board.nearestMarkerIdx.assign(nCorners, {});
    board.nearestMarkerCorners.assign(nCorners, {});
    for (size_t i = 0; i < nCorners; ++i) {
        const cv::Point3f c = board.chessboardCorners[i];
        auto& idx = board.nearestMarkerIdx[i];
        double minDist = -1;
        for (size_t j = 0; j < board.objPoints.size(); ++j) {
            cv::Point3f centre(0, 0, 0);
            for (auto& p : board.objPoints[j]) centre += p;
            centre /= 4.;
            cv::Point3f v = c - centre;
            double d = v.x * v.x + v.y * v.y;
            if (j == 0 || std::fabs(d - minDist) < tie) {
                idx.push_back((int)j);
                minDist = d;
            } else if (d < minDist) {
                idx.assign(1, (int)j);
                minDist = d;
            }
        }
        auto& corner = board.nearestMarkerCorners[i];
        corner.assign(idx.size(), 0);
        for (size_t j = 0; j < idx.size(); ++j) {
            double best = -1;
            for (int k = 0; k < 4; ++k) {
                cv::Point3f v = c - board.objPoints[idx[j]][k];
                double d = v.x * v.x + v.y * v.y;
                if (k == 0 || d < best) { best = d; corner[j] = k; }
            }
        }
    }
}

Detector::Detector(const BoardSpec& spec) : spec_(spec) {
    if (spec.squaresX < 2 || spec.squaresY < 2 || spec.dictionary < 0)
        throw std::runtime_error("Detector: board spec not loaded (use loadBoardSpec)");
    dict_  = cv::aruco::getPredefinedDictionary(spec.dictionary);
    board_ = cv::aruco::CharucoBoard::create(
        spec.squaresX, spec.squaresY, spec.squareMM, spec.markerMM, dict_);
    applyLayout(*board_, spec);
}

Detection Detector::detect(const cv::Mat& gray) const {
    Detection d;
    if (gray.empty()) return d;

    // Stage 1: locate the ArUco markers. Each encodes an ID, so partial
    // views are usable - visible markers identify themselves.
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::detectMarkers(gray, dict_, markerCorners, markerIds);
    d.markersFound = (int)markerIds.size();
    d.markerIds = markerIds;
    d.markerCorners = markerCorners;
    if (markerIds.empty()) return d;

    // Stage 2: from known marker IDs, work out where the checkerboard
    // corners must be, and refine each to sub-pixel precision.
    cv::aruco::interpolateCornersCharuco(
        markerCorners, markerIds, gray, board_, d.corners, d.cornerIds);

    return d;
}

double Detector::layoutResidualPx(const Detection& d) const {
    if (d.corners.size() < 6) return -1;
    std::vector<cv::Point2f> board;
    for (int id : d.cornerIds)
        board.emplace_back(board_->chessboardCorners[id].x,
                           board_->chessboardCorners[id].y);
    cv::Mat H = cv::findHomography(board, d.corners);
    if (H.empty()) return -1;
    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(board, projected, H);
    double worst = 0;
    for (size_t i = 0; i < projected.size(); ++i)
        worst = std::max(worst, (double)cv::norm(projected[i] - d.corners[i]));
    return worst;
}

cv::Mat Detector::annotate(const cv::Mat& gray, const Detection& d) const {
    cv::Mat out;
    cv::cvtColor(gray, out, cv::COLOR_GRAY2BGR);

    if (!d.cornerIds.empty())
        cv::aruco::drawDetectedCornersCharuco(
            out, d.corners, d.cornerIds, cv::Scalar(0, 0, 255));

    return out;
}

// Markers are small, bright and round. Window glare is irregular; a
// distant speck is usually noise. Area and roundness come from the
// camera itself - this is the 2D object filter the SDK leaves to us.
bool plausibleMarker(const Blob& b, float minArea, float maxArea, float minRound) {
    return b.area >= minArea && b.area <= maxArea && b.roundness >= minRound;
}

}  // namespace mocap
