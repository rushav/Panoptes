// centroid_dump - PANOPTES acquisition layer
//
// Connects to all OptiTrack cameras, puts them in Object mode, and uses
// cModuleSync to deliver synchronized frame groups. Emits 2D centroids as CSV.
//
// Columns: frame_id,timestamp,time_spread,cam_count,serial,obj,x,y,area,radius,roundness

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

#include "KeyPress.h"
#include "DeviceConnectionManager.h"
#include "cameralibrary.h"
#include "modulesync.h"
#include "framegroup.h"

using namespace CameraLibrary;

static std::atomic_bool g_running{true};
static void on_signal(int) { g_running.store(false); }

// Blobs below this area are noise. See docs/ for the area histogram.
static constexpr float kMinArea = 30.0f;

int main()
{
    std::signal(SIGINT,  &on_signal);
    std::signal(SIGTERM, &on_signal);

    DeviceConnectionManager manager;

    std::cerr << "[info] waiting for devices...\n";
    manager.WaitForNewDevice(10000);
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

    cModuleSync* sync = cModuleSync::Create();
    sync->SetOptimization(cModuleSync::ForceCompleteDelivery);
    sync->SetAllowIncompleteGroups(true);

    for (auto& dev : devices) {
        if (!dev) continue;
        std::cerr << "[info] " << dev->Name() << " serial " << dev->Serial() << "\n";

        if (!dev->IsVideoTypeSupported(Core::ObjectMode)) {
            std::cerr << "[warn] object mode unsupported, skipping\n";
            continue;
        }
        dev->SetVideoType(Core::ObjectMode);
        dev->SetExposure(200);
        dev->SetThreshold(240);
        dev->SetIntensity(15);

        sync->AddCamera(dev);
        dev->Start();
    }

    std::cerr << "[info] " << sync->CameraCount() << " camera(s) in sync group\n";

    std::cout << "frame_id,timestamp,time_spread,cam_count,serial,obj,x,y,area,radius,roundness\n";
    std::cout << std::fixed << std::setprecision(4);

    auto last_report = std::chrono::steady_clock::now();

    while (g_running.load() && !keyPressed()) {

        auto group = sync->GetFrameGroup();
        if (!group) {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            continue;
        }

        const int    fid    = group->FrameID();
        const double ts     = group->TimeStamp();
        const double spread = group->TimeSpread();
        const int    ncam   = group->Count();

        // frame-level record: lets us measure delivery independent of detection
        std::cout << fid << ',' << ts << ',' << spread << ',' << ncam
                  << ",-1,-1,0,0,0,0,0\n";

        for (const auto& frame : group->GetAllFrames()) {
            if (!frame) continue;

            const int n = frame->ObjectCount();
            for (int i = 0; i < n; ++i) {
                const cObject* obj = frame->Object(i);
                if (!obj || obj->Area() < kMinArea) continue;

                std::cout << fid            << ','
                          << ts             << ','
                          << spread         << ','
                          << ncam           << ','
                          << frame->Serial() << ','
                          << i              << ','
                          << obj->X()       << ','
                          << obj->Y()       << ','
                          << obj->Area()    << ','
                          << obj->Radius()  << ','
                          << obj->Roundness() << '\n';
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_report > std::chrono::seconds(2)) {
            std::cerr << "[stat] delivery " << sync->FrameDeliveryRate()
                      << " Hz, mode "
                      << (sync->LastFrameGroupMode() == FrameGroup::Hardware ? "HARDWARE" : "software")
                      << ", spread " << sync->LastFrameGroupSpread() << " s\n";
            last_report = now;
        }
    }

    std::cerr << "[info] stopping\n";
    for (auto& dev : devices) if (dev) dev->Stop();
    sync->RemoveAllCameras();
    cModuleSync::Destroy(sync);
    CameraManager::X().Shutdown();
    return 0;
}
