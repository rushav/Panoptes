// centroid_dump - PANOPTES acquisition layer prototype
//
// Connects to all detected OptiTrack cameras, puts them in Object mode,
// and prints per-frame 2D centroids to stdout as CSV.
//
// Output: serial,frame_id,timestamp,object_index,x,y,area,radius,roundness

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

#include "KeyPress.h"
#include "DeviceConnectionManager.h"
#include "cameralibrary.h"

using namespace CameraLibrary;

static std::atomic_bool g_running{true};
static void on_signal(int) { g_running.store(false); }

int main()
{
    std::signal(SIGINT,  &on_signal);
    std::signal(SIGTERM, &on_signal);

    DeviceConnectionManager manager;

    std::cerr << "[info] waiting for devices...\n";
    manager.WaitForNewDevice(10000);
    // devices appear staggered; keep waiting until the count stops growing
    for (int settle = 0; settle < 10; ++settle) {
        const size_t before = manager.GetDevices().size();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (manager.GetDevices().size() == before) break;
    }

    auto devices = manager.GetDevices();
    if (devices.empty()) {
        std::cerr << "[error] no devices found\n";
        CameraManager::X().Shutdown();
        return 1;
    }

    for (auto& dev : devices) {
        if (!dev) continue;
        std::cerr << "[info] " << dev->Name() << " serial " << dev->Serial() << "\n";

        if (!dev->IsVideoTypeSupported(Core::ObjectMode)) {
            std::cerr << "[warn] object mode unsupported on this device\n";
            continue;
        }
        dev->SetVideoType(Core::ObjectMode);
        dev->SetExposure(200);
        dev->SetThreshold(240);
        dev->SetIntensity(15);
        dev->Start();
    }

    std::cout << "serial,frame_id,timestamp,object_index,x,y,area,radius,roundness\n";
    std::cout << std::fixed << std::setprecision(4);

    while (g_running.load() && !keyPressed()) {
        for (auto& dev : devices) {
            if (!dev) continue;

            auto frame = manager.GetNextFrame(dev->Serial());
            if (!frame) continue;

            const int n = frame->ObjectCount();
            for (int i = 0; i < n; ++i) {
                const cObject* obj = frame->Object(i);
                if (!obj) continue;

                std::cout << dev->Serial()   << ','
                          << frame->FrameID() << ','
                          << frame->TimeStamp() << ','
                          << i                << ','
                          << obj->X()         << ','
                          << obj->Y()         << ','
                          << obj->Area()      << ','
                          << obj->Radius()    << ','
                          << obj->Roundness() << '\n';
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    std::cerr << "[info] stopping\n";
    for (auto& dev : devices) if (dev) dev->Stop();
    CameraManager::X().Shutdown();
    return 0;
}
