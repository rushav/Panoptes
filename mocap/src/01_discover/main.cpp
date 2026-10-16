// Stage 1: find cameras on the network and report what they are.
// No streaming, no images - just proving the SDK can see the hardware.

#include <cameralibrary.h>
#include <chrono>
#include <map>
#include <thread>
#include <cstdio>

using namespace CameraLibrary;

int main() {
    // Start the SDK's discovery service. It begins listening for the
    // UDP 13013 broadcasts we just watched with tcpdump.
    CameraManager::X().WaitForInitialization();

    // Discovery is staggered: cameras finish initializing at different
    // times. Rather than trusting the first "ready" signal, poll until
    // the count holds steady. Full enumeration takes 4-8 seconds cold.
    size_t previous = 0;
    int stable = 0;
    for (int i = 0; i < 120 && stable < 20; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        CameraList list;
        size_t count = list.Count();

        if (count == previous && count > 0) {
            ++stable;                 // unchanged - might be done
        } else {
            stable = 0;               // still growing - reset
            previous = count;
            printf("discovery: %zu device(s) so far\n", count);
        }
    }

    // The list can contain the same serial more than once. Collapse by
    // serial so each physical camera is reported once.
    CameraList list;
    std::map<int, int> unique;        // serial -> index in list
    for (int i = 0; i < list.Count(); ++i) {
        unique[list[i].Serial()] = i;
    }

    printf("\nsettled on %zu camera(s)\n\n", unique.size());
    printf("%-10s %-6s %-24s %s\n", "serial", "rev", "name", "state");

    for (auto& [serial, i] : unique) {
        printf("%-10d %-6d %-24s %s\n",
               list[i].Serial(),
               list[i].Revision(),
               list[i].Name(),
               list[i].State() == Initialized ? "Initialized" : "not ready");
    }

    CameraManager::X().Shutdown();
    return 0;
}