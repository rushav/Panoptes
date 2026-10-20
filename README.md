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
| Calibration target | ChArUco board: 9 × 6 squares of 90 mm, 67 mm markers, `DICT_5X5_1000` |
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
  error. Intrinsics for one camera are solved in Python, and the solves are
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
python/    Intrinsics solve, rig model, triangulation, synthetic accuracy study
config/    Camera layout, mask and study settings, saved per-camera masks
docs/      Runbook, architecture, measured findings, hardware
analysis/  Accuracy study outputs (PNGs regenerated; JSON summary tracked)
logs/      Mask coverage-walk logs
```

Capture sessions are written to `$HOME/panoptes-data/sessions`, outside the
repo.

## Documentation

- **[docs/RUNBOOK.md](docs/RUNBOOK.md)**: set up from a clean clone, build,
  network, every tool's flags and keys, a calibration session, troubleshooting.
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**: how it works and why,
  stage by stage, in data order.
- **[docs/FINDINGS.md](docs/FINDINGS.md)**: everything measured on this
  hardware, and the open items.

## Getting started

You need the OptiTrack Camera SDK at `~/optitrack/CameraSDK`, the system
packages and venv in [RUNBOOK §1](docs/RUNBOOK.md#1-environment-setup-from-a-clean-clone),
and the cameras on a dedicated NIC ([RUNBOOK §2](docs/RUNBOOK.md#2-camera-network)).
Then:

```sh
for t in src/0*/; do cmake -S "$t" -B "$t/build" && cmake --build "$t/build" -j"$(nproc)" || break; done
# grant capabilities to each binary (RUNBOOK §3), then:
src/01_discover/build/discover
```

The binaries need `cap_net_raw,cap_net_admin`, and **relinking removes it**,
so set it again after every build.
