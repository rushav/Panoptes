// grab_frames - live preview + calibration capture
//
// Shows a live window per camera with ArUco detection overlaid.
//   SPACE = save current frames as PGM
//   +/-   = exposure up/down
//   q     = quit

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <map>
#include <filesystem>
#include <cstdlib>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "DeviceConnectionManager.h"
#include "cameralibrary.h"

using namespace CameraLibrary;
namespace fs = std::filesystem;

static bool write_pgm(const std::string& path, const cv::Mat& m)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f << "P5\n" << m.cols << " " << m.rows << "\n255\n";
    f.write(reinterpret_cast<const char*>(m.data),
            static_cast<std::streamsize>(m.cols) * m.rows);
    return f.good();
}

int main()
{
    fs::create_directories("captures");

    DeviceConnectionManager manager;
    std::cerr << "[info] waiting for devices...\n";
    manager.WaitForNewDevice(10000);
    for (int i = 0; i < 10; ++i) {
        const size_t before = manager.GetDevices().size();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (manager.GetDevices().size() == before) break;
    }

    auto devices = manager.GetDevices();
    if (devices.empty()) { std::cerr << "[error] no devices\n"; return 1; }

    int exposure = 6000;

    for (auto& d : devices) {
        if (!d) continue;
        std::cerr << "[info] " << d->Name() << " serial " << d->Serial() << "\n";
        d->SetVideoType(Core::GrayscaleMode);
        d->SetExposure(exposure);
        d->SetIntensity(15);
        d->SetImagerGain(static_cast<eImagerGain>(3));
        d->SetImagerGain(static_cast<eImagerGain>(3));
        d->Start();
        d->SetVideoType(Core::GrayscaleMode);
        cv::namedWindow(std::to_string(d->Serial()), cv::WINDOW_NORMAL);
        cv::resizeWindow(std::to_string(d->Serial()), 640, 512);
    }

    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);

    std::map<unsigned int, cv::Mat> latest;
    std::map<unsigned int, int> det;              // markers seen this frame
    std::map<unsigned int, std::vector<cv::Point2f>> lastSaved;  // corners at last save
    std::map<unsigned int, int> savedCount;
    std::map<unsigned int, std::vector<cv::Point2f>> curCorners;
    int shot = 0;
    bool autoMode = false;
    const int    kMinMarkers  = 12;     // trigger threshold for the driving camera
    const int    kSaveMarkers  = 4;      // any camera seeing this many gets saved too
    int          autoIndex     = 0;      // shared capture index across all cameras
    const double kMinMovePx   = 60.0;   // board must move this far since last save
    auto lastAuto = std::chrono::steady_clock::now();

    std::cerr << "[info] SPACE=capture  +/-=exposure  q=quit\n";

    while (true) {
        for (auto& cam : devices) {
            if (!cam) continue;

            std::shared_ptr<const Frame> frame, f;
            for (int i = 0; i < 64 && (f = manager.GetNextFrame(cam->Serial())); ++i)
                frame = f;
            if (!frame || frame->FrameType() != Core::GrayscaleMode) continue;

            const unsigned char* px =
                const_cast<Frame*>(frame.get())->GrayscaleData(*cam);
            if (!px) continue;

            cv::Mat img(frame->Height(), frame->Width(), CV_8UC1,
                        const_cast<unsigned char*>(px));
            latest[cam->Serial()] = img.clone();

            cv::Mat vis;
            cv::cvtColor(img, vis, cv::COLOR_GRAY2BGR);

            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids;
            cv::aruco::detectMarkers(img, dict, corners, ids);
            if (!ids.empty())
                cv::aruco::drawDetectedMarkers(vis, corners, ids);

            det[cam->Serial()] = (int)ids.size();
            std::vector<cv::Point2f> centres;
            for (const auto& c : corners) {
                cv::Point2f m(0,0);
                for (const auto& p : c) m += p;
                centres.push_back(m * 0.25f);
            }
            curCorners[cam->Serial()] = centres;

            std::ostringstream hud;
            hud << "markers=" << ids.size()
                << "  mean=" << std::fixed << std::setprecision(0) << cv::mean(img)[0]
                << "  exp=" << exposure
                << "  shots=" << shot;
            cv::putText(vis, hud.str(), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        ids.empty() ? cv::Scalar(0,0,255) : cv::Scalar(0,255,0), 2);

            cv::imshow(std::to_string(cam->Serial()), vis);
        }

        const int key = cv::waitKey(1) & 0xFF;

        if (key == 'q') break;

        if (key == 'a') {
            autoMode = !autoMode;
            std::cerr << "[mode] auto-capture " << (autoMode ? "ON" : "OFF") << "\n";
        }

        if (key == '+' || key == '=' || key == '-' || key == '_') {
            exposure += (key == '+' || key == '=') ? 1000 : -1000;
            exposure = std::max(100, std::min(exposure, 30000));
            for (auto& d : devices) if (d) d->SetExposure(exposure);
            for (auto& d : devices)
                if (d) std::cerr << "[exp] " << d->Serial() << " requested "
                                 << exposure << " actual " << d->Exposure() << "\n";
        }

        if (autoMode) {
            const auto now = std::chrono::steady_clock::now();
            if (now - lastAuto > std::chrono::milliseconds(400)) {

                // does any camera have a good, moved view?
                bool trigger = false;
                for (auto& cam : devices) {
                    if (!cam) continue;
                    const unsigned int sn = cam->Serial();
                    if (det[sn] < kMinMarkers) continue;

                    const auto& cur  = curCorners[sn];
                    const auto& prev = lastSaved[sn];
                    if (!prev.empty() && !cur.empty()) {
                        cv::Point2f a(0,0), b(0,0);
                        for (auto& p : cur)  a += p;  a *= 1.0f / cur.size();
                        for (auto& p : prev) b += p;  b *= 1.0f / prev.size();
                        if (cv::norm(a - b) < kMinMovePx) continue;
                    }
                    trigger = true;
                    lastSaved[sn] = cur;
                }

                if (trigger) {
                    std::ostringstream log;
                    log << "[auto] " << autoIndex << ":";
                    for (auto& cam : devices) {
                        if (!cam) continue;
                        const unsigned int sn = cam->Serial();
                        if (det[sn] < kSaveMarkers) continue;
                        auto it = latest.find(sn);
                        if (it == latest.end() || it->second.empty()) continue;

                        std::ostringstream name;
                        name << "captures/pose" << std::setw(3) << std::setfill('0')
                             << autoIndex << "_cam" << sn << ".pgm";
                        if (write_pgm(name.str(), it->second)) {
                            ++savedCount[sn];
                            log << "  " << sn << "(" << det[sn] << ")";
                        }
                    }
                    std::cerr << log.str() << "\n";
                    std::system("play -nq -t alsa synth 0.08 sine 880 >/dev/null 2>&1 &");
                    ++autoIndex;
                }
                lastAuto = now;
            }
        }

        if (key == ' ') {
            int saved = 0;
            for (auto& cam : devices) {
                if (!cam) continue;
                auto it = latest.find(cam->Serial());
                if (it == latest.end() || it->second.empty()) continue;

                std::ostringstream name;
                name << "captures/frame_" << std::setw(3) << std::setfill('0')
                     << shot << "_cam" << cam->Serial() << ".pgm";
                if (write_pgm(name.str(), it->second)) ++saved;
            }
            std::cerr << "[capture] " << shot << ": " << saved << " image(s)\n";
            ++shot;
        }
    }

    for (auto& d : devices) if (d) d->Stop();
    cv::destroyAllWindows();
    CameraManager::X().Shutdown();
    std::cerr << "[info] " << shot << " capture(s)\n";
    return 0;
}
