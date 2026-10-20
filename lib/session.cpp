#include "session.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <filesystem>
#include <fstream>
#include <ctime>
#include <sstream>
#include <iomanip>

namespace fs = std::filesystem;

namespace mocap {

static std::string timestamp() {
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream os;
    os << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return os.str();
}

static std::string isoNow() {
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::gmtime(&t);
    std::ostringstream os;
    os << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    return os.str();
}

Session::Session(const std::string& root, int serial, const BoardSpec& board)
    : serial_(serial), board_(board) {
    path_ = root + "/intrinsics_" + std::to_string(serial) + "_" + timestamp();
    fs::create_directories(path_);
}

void Session::record(const cv::Mat& gray, const Detection& d, const Settings& a) {
    ++count_;

    std::ostringstream name;
    name << "frame_" << std::setw(4) << std::setfill('0') << count_ << ".png";
    cv::imwrite(path_ + "/" + name.str(), gray);

    std::ostringstream e;
    e << (count_ > 1 ? ",\n" : "")
      << "    {\"file\": \"" << name.str() << "\""
      << ", \"markers\": " << d.markersFound
      << ", \"corners\": " << d.corners.size()
      << ", \"exposure\": " << a.exposure
      << ", \"gain\": " << a.gain
      << ", \"intensity\": " << a.intensity
      << ", \"frame_rate\": " << a.frameRate
      << ", \"ir_filter\": " << (a.irFilter ? "true" : "false")
      << "}";
    json_ += e.str();

    flush();
}

void Session::removeLast() {
    if (count_ == 0) return;
    std::ostringstream name;
    name << "frame_" << std::setw(4) << std::setfill('0') << count_ << ".png";
    fs::remove(path_ + "/" + name.str());

    // Drop the last JSON entry.
    size_t cut = json_.rfind(",\n    {");
    json_ = (cut == std::string::npos) ? "" : json_.substr(0, cut);
    --count_;
    flush();
}

void Session::flush() const {
    std::ofstream out(path_ + "/session.json");
    out << "{\n"
        << "  \"camera_serial\": " << serial_ << ",\n"
        << "  \"created\": \"" << isoNow() << "\",\n"
        << "  \"opencv_version\": \"" << CV_VERSION << "\",\n"
        << "  \"board\": {\n"
        << "    \"squares_x\": " << board_.squaresX << ",\n"
        << "    \"squares_y\": " << board_.squaresY << ",\n"
        << "    \"square_mm\": " << board_.squareMM << ",\n"
        << "    \"marker_mm\": " << board_.markerMM << ",\n"
        << "    \"dictionary\": \"DICT_5X5_1000\"\n"
        << "  },\n"
        << "  \"frames\": [\n" << json_ << "\n  ]\n"
        << "}\n";
}

}  // namespace mocap