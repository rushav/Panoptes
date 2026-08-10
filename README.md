# PANOPTES

Open Linux motion capture — a vendor-independent 3D optical tracking stack for
OptiTrack cameras, built entirely on Linux without proprietary software.

## What this is

OptiTrack cameras normally require Motive, which is Windows-only. Motive handles
calibration, triangulation, marker labeling, and rigid-body solving — the entire
3D pipeline.

PANOPTES replaces that pipeline with an open Linux implementation. The vendor's
Linux Camera SDK provides camera control, hardware synchronization, and 2D marker
centroids. Everything above that layer is built from scratch:

- Multi-camera intrinsic and extrinsic calibration
- 3D triangulation from 2D observations
- Marker labeling and correspondence
- Rigid-body pose estimation and filtering
- ROS 2 publishing with hardware timestamps

## Status

Phase 1 — camera bring-up.

- [x] Single camera enumerates on Linux (Prime 13W, SDK 3.5.0 Beta1)
- [x] Live grayscale streaming via `CameraViewerApp`
- [ ] Object mode / 2D centroid output verified
- [ ] Multi-camera acquisition
- [ ] Calibration
- [ ] Triangulation and rigid-body solve
- [ ] ROS 2 output

See [docs/compatibility.md](docs/compatibility.md) for verified hardware and
software versions.

## Hardware

| Item | Notes |
|---|---|
| OptiTrack Prime 13W | GigE/PoE, 82° x 70° FOV, global shutter |
| PoE switch | Gigabit, power budget covering all cameras |
| Cat5e/Cat6 cables | One per camera, plus one switch-to-host |
| Linux host | x86_64 (ARM unsupported), gigabit Ethernet |

## Quick start

    sudo apt install build-essential cmake libjpeg-dev qt6-base-dev

Set the camera network interface to Link-Local Only, then build a sample:

    cd CameraSDK/samples/InfoDump
    ./linuxBuild.sh ../../
    sudo ./build/InfoDump

## License

MIT — see [LICENSE](LICENSE).
