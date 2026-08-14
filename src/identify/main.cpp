// identify - blink each camera's LED ring in turn so you can label them
#include <iostream>
#include <thread>
#include <chrono>
#include "DeviceConnectionManager.h"
#include "cameralibrary.h"

using namespace CameraLibrary;

int main()
{
    DeviceConnectionManager manager;
    std::cerr << "[info] waiting for devices...\n";
    manager.WaitForNewDevice(10000);
    for (int i = 0; i < 10; ++i) {
        const size_t before = manager.GetDevices().size();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (manager.GetDevices().size() == before) break;
    }

    auto devices = manager.GetDevices();
    std::cerr << "[info] " << devices.size() << " camera(s)\n\n";

    for (auto& d : devices) { if (d) { d->SetIntensity(0); d->Start(); } }

    for (auto& d : devices) {
        if (!d) continue;
        std::cerr << ">>> BLINKING serial " << d->Serial()
                  << "  (" << d->Name() << ")  -- watch for it, 10 s\n";
        for (int i = 0; i < 10; ++i) {
            d->SetIntensity(15);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            d->SetIntensity(0);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::cerr << "    done\n\n";
    }

    for (auto& d : devices) if (d) d->Stop();
    CameraManager::X().Shutdown();
    return 0;
}
