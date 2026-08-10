# Command Reference

Running notes on paths and commands for this project. Updated as things change.

## Paths

| What | Where |
|---|---|
| Repo | `~/dev/panoptes` |
| OptiTrack SDK | `~/optitrack/CameraSDK` |
| SDK headers | `~/optitrack/CameraSDK/include` |
| SDK samples | `~/optitrack/CameraSDK/samples` |

## Run the SDK samples

Console enumeration test (fastest way to confirm cameras are seen):

    cd ~/optitrack/CameraSDK/samples/InfoDump
    sudo ./build/InfoDump

GUI viewer (live image, mode switching, camera controls):

    cd ~/optitrack/CameraSDK/samples/CameraViewerApp
    sudo ./build/CameraViewerApp

Both need `sudo` for raw network access.

## Rebuild a sample

Build scripts take the SDK root as an argument:

    cd ~/optitrack/CameraSDK/samples/<SampleName>
    chmod +x linuxBuild.sh
    ./linuxBuild.sh ../../

Binary lands in `./build/`.

## Dependencies

    sudo apt install build-essential cmake libjpeg-dev qt6-base-dev

Qt6 is required — `qtbase5-dev` will not satisfy `find_package(Qt6)`.

If cmake still can't find Qt6:

    export CMAKE_PREFIX_PATH=/usr/lib/x86_64-linux-gnu/cmake/Qt6

## Network

Camera interface: `enp6s0` (Realtek RTL8125 2.5G, built in)

Set to link-local:

    nmcli con mod "Wired connection 1" ipv4.method link-local
    nmcli con down "Wired connection 1" && nmcli con up "Wired connection 1"

Check the interface has a 169.254.x.x address:

    ip addr show enp6s0

Confirm cameras are broadcasting (discovery is UDP port 13013):

    sudo tcpdump -i enp6s0 -n port 13013

Turn WiFi off if the SDK seems to bind the wrong interface:

    nmcli radio wifi off

## Searching the SDK API

Find a symbol across headers:

    grep -rn "<symbol>" ~/optitrack/CameraSDK/include/

Read a header:

    sed -n '1,120p' ~/optitrack/CameraSDK/include/frame.h

Key headers:

| Header | Contains |
|---|---|
| `camera.h` | Camera control, settings, modes |
| `frame.h` | Per-frame data: `FrameID()`, `TimeStamp()`, `ObjectCount()`, `Object(i)` |
| `object.h` | Per-blob data: `X()`, `Y()`, `Area()`, `Radius()`, `Roundness()` |
| `cameratypes.h` | `sObjectModeSettings` (min/max diameter, margin, skew) |
| `cameramanager.h` | Device discovery and connection |

## Troubleshooting

| Symptom | Check |
|---|---|
| Camera LED ring dark | PoE power — not a software problem |
| `No devices detected` | LED ring lit? WiFi off? firewall? interface link-local? |
| No traffic on tcpdump | Cabling, switch port, PoE budget |
| Build fails on Qt6 | Install `qt6-base-dev`, set `CMAKE_PREFIX_PATH` |

## Git

    cd ~/dev/panoptes
    git add .
    git commit -m "message"
    git push
