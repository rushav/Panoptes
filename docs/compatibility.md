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
