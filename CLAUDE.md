# CLAUDE.md

## Purpose

PANOPTES is a motion capture system for Linux, built from scratch, on ten
OptiTrack Prime 13W cameras. Its only job is ground truth for VESPR, a UW GIX
capstone drone that perches using onboard-only state estimation (RealSense
D435i).

**Hard rule: mocap data never enters the drone's control loop.** It is a
measuring instrument. Never design, suggest or wire anything that feeds
PANOPTES output into VESPR's control or estimation.

Docs:

- [README.md](README.md): overview
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): how and why
- [docs/RUNBOOK.md](docs/RUNBOOK.md): commands
- [docs/FINDINGS.md](docs/FINDINGS.md): every measured result and the open
  items. It is the single home for measurements; other docs cite it.

## Working style

- **Never commit unless explicitly told to.** Rushav reviews the diff and
  commits. Leave changes in the working tree.
- **You cannot sudo.** When something needs root (`setcap`, `tcpdump`, `apt`),
  print the exact commands for Rushav to run. Don't claim a tool is runnable
  until that has been done.
- **In every summary, separate "verified by running" from "inferred from
  reading."** Say which commands you ran and what they printed.
- **Measure before assuming.** The SDK is thinly documented and several calls
  behave differently from their names.
  - Grep the SDK headers (`~/optitrack/CameraSDK/include`) before trusting a
    method name or signature.
  - Prefer a camera readback over an echo of what was requested.
- When a doc and the code disagree, the code wins. Report the disagreement
  rather than silently "fixing" either.

## Verified SDK and hardware behaviour: do not "fix" these away

Each rule is backed by a FINDINGS section. If code looks redundant or odd
around one of these, it is deliberate.

1. **Discovery is staggered and scales with camera count.** The SDK signals
   ready after the *first* camera. Poll `CameraList` every 250 ms until the
   count is unchanged for 5 s (20 polls), capped at 30 s. A 1.5 s window
   reported 5 of 10. (§8)
2. **`CameraList` can repeat a serial.** Collapse by serial. (§8)
3. **`SetVideoType` is asynchronous.** Set it, `Start()`, set it again, then
   apply settings. Check `FrameType()` on every frame; queued frames keep the
   old mode, and `GrayscaleData()` on an object frame is null. (§9)
4. **`NextFrame()` returns the OLDEST frame** as `std::shared_ptr<const Frame>`.
   Drain to the newest every loop. Freeing is by dropping the pointer; there
   is no manual `Release()`. (§9)
5. **`ImagerWidth()` / `ImagerHeight()` are `double`s, the sensor size in mm.**
   Pixel counts are `PhysicalPixelWidth()` / `PhysicalPixelHeight()`;
   `Width()` / `Height()` describe an ROI. (`lib/camera.cpp`, SDK header)
6. **`SetImagerGain` takes `eImagerGain`**, not `int`. Cast explicitly. (SDK
   header)
7. **The exposure ceiling is `floor(1e6/fps) − 200` µs.** Re-read
   `MaximumExposureValue()` after any frame-rate change, and set the rate
   before the exposure. (§1)
8. **The blocking mask is a grid of 640 × 512 cells of 2 px.** (§13)
   - `SetBitMaskPixel` takes **cells**.
   - `AddBlockingRectangle` takes **pixels**.
   - `SetBlockingMask` needs **exactly cols × rows bytes**; any other size
     silently clears the staged mask.
   - Nothing reaches the camera until `UpdateBlockingMask()`, which sends only
     chunks that differ from the SDK's belief. That belief starts empty in
     every process, so `Camera::setMask()` commits all-blocked and then the
     target.
   - `IsBlockingMaskEnabled()` is an SDK flag, not a camera readback.
9. **Masks are settings-specific and position-specific.** A mask learned at
   one exposure and threshold is wrong at another, and wrong once its camera
   moves. (§13)
10. **Change settings one at a time while streaming.** Re-sending every
    setting stalls delivery; `Camera::apply()` diffs on purpose. (§12)
11. **`SendEmptyFrames(true)` in object mode.** Without it, "no blobs" and "no
    frames" are indistinguishable. (§11)
12. **`setcap cap_net_raw,cap_net_admin+eip` is stripped on every relink.**
    Every rebuilt binary needs it again. Don't run tools under sudo. (§16)
13. **Two OpenCV versions, on purpose.** C++ uses system OpenCV 4.6; Python
    uses 5.x (`opencv-contrib-python`) in `.venv`. Don't unify them. (§16)
14. **`lib/detect` builds the board layout itself, by the OpenCV 4.7+
    rule.** OpenCV 4.6's `create()` always builds the non-legacy layout, and
    it mislabelled every corner of the old 9×6 board. `legacy_pattern` in
    `config/charuco_board.yaml` only matters for an even row count; the 12×9
    board is the same either way. Python (5.x) sets it with
    `setLegacyPattern()` from the same config and calibrates via
    `CharucoDetector.detectBoard` + `matchImagePoints` + `calibrateCamera`.
    **Never trust a corner count without the fit residual**
    (`layoutResidualPx`, `fit` in `03_detect`). A wrong layout still returns
    every corner, with a residual of ~200 px against ~1 px. (§5, §7, §17)
15. **RMS reprojection error is not an acceptance criterion. Zero empty
    coverage cells (5 × 6 grid) is.** All-fronto-parallel views are a
    separate failure; aim for 30% or more of captures tilted beyond 20°.
    (§5, §6)
16. **Grayscale delivery is bandwidth-bound at 6–8 fps** at any imager rate.
    It is not a bug. (§3)
17. **Calibrate with the IR filter IN and the ring at 15.** The filter shifts
    the focal plane. (§2)
18. **Gain is free at the grayscale operating point** (jitter 0.054 px at
    gain 7 vs 0.056 at gain 0). (§4)
19. **Starting a tool right after killing one can hang** while the old
    sockets are bound. Wait a couple of seconds. (§9)

## Conventions

- **`lib/`** has no UI code. Every SDK quirk is handled there, once.
- **`src/NN_name/`** tools are thin. Each is its own CMake project compiling
  the `lib/` sources it needs. `01`–`03` call the SDK directly; new tools
  must go through `lib/`.
- **Configs** live in `config/`. Lengths in `layout.yaml` and `study.yaml` are
  inches and are converted to mm only in `python/rig.py`. YAML is read with
  OpenCV `FileStorage` (`%YAML:1.0` header), and string values with trailing
  comments must be quoted.
- **Outputs.** `analysis/*.png` and `*.npz` are gitignored and regenerated;
  `analysis/accuracy_study.json` is tracked. `logs/coverage_*.csv` are
  currently tracked, not ignored. Capture sessions go to
  `$HOME/panoptes-data/sessions`, outside the repo.
- **Python** runs only via `.venv/bin/python`. Dependencies are pinned in
  `python/requirements.txt`.
- **No absolute paths in code or docs.** Use repo-relative paths, `$HOME`, or
  `~`. (`07_mask`'s `MOCAP_CONFIG_DIR` is generated at configure time and is
  an open item.)
- **Verify a change** with the build loop and `python/triangulate.py` (all 12
  checks must pass) as in RUNBOOK §3–4. Use a fresh build directory when the
  build configuration changes.

## Open items

Copied from [FINDINGS.md → Open items](docs/FINDINGS.md#open-items), which is
authoritative. Update both together.

1. **cx cross-session discrepancy:** 9.4 px across three sessions of camera
   33661. Run `bootstrap_intrinsics.py` to see whether sampling explains it.
2. **WRONG MODE false alarm:** `modeIsObject()` reflects the last *poll*, and
   an empty poll reads as "not object" (inferred from `lib/camera.cpp`).
3. **Tools don't report mask state:** masks stay on the cameras after
   `07_mask` exits. Other tools neither read nor clear them, so a masked
   camera looks like a detection bug.
4. **Mask tool's compile-time config path:** `MOCAP_CONFIG_DIR` breaks when
   the repo moves. Rebuild, or use `--config`.
5. **Frame sync** via `cModuleSync` (not `Synchronizer`): not built.
6. **Marker labeling:** not built.
7. **Extrinsics, wand and L-frame:** not built.
8. **Reference markers:** not built.
9. **Object-mode operating point** is unsettled: `identify`/`objdiag` use
   500/200, `mask.yaml` uses 10/40.
10. **Mask acceptance walk has no recorded PASS.** Every log has a minimum of
    0 observers.
11. **Inconsistencies found 2026-10-19:**
    - `02_stream` and `03_detect` use the short discovery window.
    - `05_identify` hardcodes the 33133 exposure cap.
    - `session.json` `created` is rewritten on every flush.
    - `cameras.yaml` and `cameras.md` list 6 cameras vs 10 in `layout.yaml`.
    - `PyYAML` is pinned but unused.
    - `layout.yaml` distortion is assumed, not fitted.
