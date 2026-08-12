# Hardware Compatibility

## Confirmed working

| Component | Version |
|---|---|
| Camera | OptiTrack Prime 13W (legacy Prime series) |
| SDK | OptiTrack Camera SDK 3.5.0 Beta1 (Ubuntu) |
| OS | Ubuntu 24.04.4 LTS |
| Kernel | 7.0.0-28-generic |
| Switch | NETGEAR 24-port PoE+ |

**Result:** Prime 13W enumerates and streams via the Linux Camera SDK.
Both `InfoDump` and `CameraViewerApp` samples build and run.

This is not documented by the vendor — official Linux SDK notes reference the
X-series (PrimeX, VersaX, SlimX). Legacy Prime-series support was unverified
prior to this test.

## Build dependencies

    sudo apt install build-essential cmake libjpeg-dev qt6-base-dev

Samples require **Qt6**, not Qt5. Installing `qtbase5-dev` will not satisfy
the CMake `find_package(Qt6)` call.

Build scripts take the SDK root as an argument:

    ./linuxBuild.sh ../../

## Network

- Camera interface set to **Link-Local Only** (169.254.x.x)
- Separate interface (WiFi) for internet
- Cameras broadcast discovery on **UDP port 13013**

Verify the camera is on the wire before debugging the SDK:

    sudo tcpdump -i <iface> -n port 13013

Discovery broadcasts from 0.0.0.0 until the SDK initializes the camera; this is
normal.

## Notes

- The camera LED ring must be lit. If it is dark, the problem is PoE, not software.
- USB cameras are not supported by the Linux Camera SDK. GigE/PoE only.

## Multi-camera synchronization

Two Prime 13W cameras (serials 33661, 33275) on one PoE switch were verified
hardware-synchronized with no additional configuration:

- Both cameras report the **same frame IDs**
- `Frame::TimeStamp()` is **identical** across cameras for a shared frame ID
  (measured offset: 0.00000 s across all sampled frames)
- Frame rate 120 Hz (timestamp delta 0.0083 s)

No eSync device is required for frame-level alignment between Ethernet cameras.

### Device discovery is staggered

`WaitForNewDevice()` returns as soon as the *first* device initializes. With
multiple cameras this yields an incomplete device list. Poll `GetDevices()`
until the count stops growing before starting capture.

### Dropped frames

Frame IDs show gaps under a naive per-camera polling loop. Increasing socket
buffers helps:

    sudo sysctl -w net.core.rmem_max=26214400
    sudo sysctl -w net.core.rmem_default=26214400

The proper fix is the SDK's Synchronizer / frame-group API rather than polling
each camera independently.

## Frame group synchronization (cModuleSync)

`cModuleSync` delivers synchronized frame groups across cameras and eliminates
the frame loss seen when polling each camera independently.

Configuration used:

    sync->SetOptimization(cModuleSync::ForceCompleteDelivery);
    sync->SetAllowIncompleteGroups(true);

Measured with two Prime 13W cameras:

| Metric | Result |
|---|---|
| Delivery rate | 120 Hz sustained |
| `LastFrameGroupMode()` | **Hardware** |
| `LastFrameGroupSpread()` | 0 s |
| Frames delivered | 1621 / 1621 (**0% loss**) |

By comparison, a naive per-camera `GetNextFrame()` polling loop dropped frames
continuously.

Note: `Synchronizer` (synchronizer.h) is a per-camera frame pool, not the
multi-camera grouping mechanism. `cModuleSync` (modulesync.h) is the correct
class. No SDK sample demonstrates it.

## Static centroid noise floor

A stationary marker at ~1 m, Object mode, threshold 240:

- X spread ≈ 0.05 px peak-to-peak
- Y spread ≈ 0.04 px peak-to-peak

Blob area for a single marker at this range is ~46–56 px, so an area filter
threshold of 30 is appropriate. A threshold of 50 sits inside the marker's own
area distribution and causes intermittent rejection.
