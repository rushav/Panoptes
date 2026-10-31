# PANOPTES

A motion capture system for Linux, built from scratch, on ten OptiTrack Prime
13W cameras.

PANOPTES has one job: **ground truth for VESPR**. VESPR is a UW GIX capstone
drone that perches on infrastructure and estimates its own state using only
onboard sensors (an Intel RealSense D435i). To show that the onboard estimate
is accurate, something independent has to measure where the drone really was.
That is PANOPTES.

**Mocap data never enters the drone's control loop.** PANOPTES is a measuring
instrument. If it helped fly the drone, the onboard estimate would prove
nothing.

## Why build it instead of buying Motive

OptiTrack's own software, Motive, runs only on Windows. The vendor's Linux
Camera SDK provides three things: camera control, hardware frame sync, and 2D
blob centroids. It provides nothing above that. So everything that turns
centroids into a pose is built here:

- calibration
- triangulation
- marker labeling
- rigid-body pose
- streaming

The SDK is also thinly documented, so its behaviour is measured on this
hardware rather than assumed. Those measurements are a large part of this
repo.

## Hardware

| | |
|---|---|
| Cameras | 10 × OptiTrack Prime 13W (legacy Prime series, not PrimeX) |
| Imager | 1280 × 1024, mono, global shutter, up to 120 Hz |
| Optics | 82° × 70° field of view, manual focus; 850 nm IR ring; switchable 850 nm bandpass filter |
| Network | Gigabit Ethernet with PoE, link-local addressing, one host NIC dedicated to the cameras |
| Mounting | 4 cameras at 51.5 in, 6 at 119 in, all aimed at one point, around a 61 × 134 × 119 in volume |
| Calibration target | calib.io ChArUco board: 12 × 9 squares of 60 mm (nominal), 45 mm markers, `DICT_5X5_100`, on aluminium; defined in [config/charuco_board.yaml](config/charuco_board.yaml) with caliper-measured sizes |
| Host | Linux (Ubuntu 24.04), C++17, OpenCV 4.6 (C++) and 5.x (Python) |

Specifications: [docs/hardware/cameras.md](docs/hardware/cameras.md). Camera
positions and serials: [config/layout.yaml](config/layout.yaml).

## Current state

**Works today**

- **Camera discovery** that waits for all ten cameras. The SDK reports
  "ready" after the first one, so a naive wait misses most of them.
- **Grayscale streaming** from any camera.
- **ChArUco intrinsics capture** with a live coverage grid and tilt histogram.
  The acceptance criterion is zero empty grid cells, not RMS reprojection
  error. Every tool prints the board it loaded, and each session records it.
  Detection reports a fit residual as well as the corner count, because a
  wrong board layout still returns every corner. Intrinsics for one camera are solved in Python, and the solves are
  compared across sessions.
- **Ten-camera object mode**: every camera streaming blob centroids at once,
  in a tiled view labelled by serial.
- **IR blocking masks** that hide other cameras' illumination rings. Masks
  are learned per camera from an empty volume, applied on the camera,
  verified and saved, and there is a walk-through coverage test.
- **A synthetic triangulation accuracy study** of the mounted layout:
  - A DLT seed plus Levenberg-Marquardt triangulator, with a self-test.
  - The floor on error is a median of 0.077 mm at the perch.
  - **19% of the volume (the top of the room) cannot be triangulated** with
    the current aim.

**Not built yet**

- Frame synchronisation across cameras
- Marker labeling
- Extrinsic calibration (wand and L-frame)
- Rigid-body pose
- Streaming output

## Repository layout

```
lib/       C++ library: SDK wrapper, ChArUco detection, coverage, session output. No UI.
src/       One small tool per task (01_discover ... 07_mask), each its own CMake project
python/    Intrinsics solve, rig model, triangulation, synthetic accuracy study,
           Kinova Gen3 arm client: status, boxed Cartesian moves (arm.py),
           tool-to-image probe and arm-driven sessions (arm_capture.py),
           capture status reader
config/    Camera layout, mask and study settings, saved per-camera masks
docs/      Runbook, architecture, measured findings, hardware
analysis/  Accuracy study outputs (PNGs regenerated; JSON summary tracked)
logs/      Mask coverage-walk logs
```

Capture sessions are written to `$HOME/panoptes-data/sessions`, outside the
repo.

## Documentation

- **[docs/RUNBOOK.md](docs/RUNBOOK.md)**: a command card (§1), then one
  procedure per task with its own troubleshooting.
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**: how it works and why,
  stage by stage, in data order.
- **[docs/FINDINGS.md](docs/FINDINGS.md)**: key numbers, the rules learned,
  everything measured on this hardware, and the open items.

## Getting started

You need the OptiTrack Camera SDK at `~/optitrack/CameraSDK`, the system
packages and venv in [RUNBOOK §2](docs/RUNBOOK.md#2-environment-setup),
and the cameras on a dedicated NIC ([RUNBOOK §4](docs/RUNBOOK.md#4-network-discovery-and-object-mode)).
Then build and grant capabilities with the block in
[RUNBOOK §1.2](docs/RUNBOOK.md#12-build--setcap), and run:

```sh
src/01_discover/build/discover
```

The binaries need `cap_net_raw,cap_net_admin`, and **relinking removes it**,
so set it again after every build.
