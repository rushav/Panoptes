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

Each rule is backed by a FINDINGS section (§N below means FINDINGS). If code
looks redundant or odd around one of these, it is deliberate. **This list is
identical to [FINDINGS §2](docs/FINDINGS.md#2-rules-we-learned); change both
together, in the same change.**

1. **Discovery is staggered and scales with camera count.** The SDK signals
   ready after the *first* camera. Poll `CameraList` every 250 ms until the
   count is unchanged for 5 s (20 polls), capped at 30 s. A 1.5 s window
   reported 5 of 10. (§10)
2. **`CameraList` can repeat a serial.** Collapse by serial. (§10)
3. **`SetVideoType` is asynchronous.** Set it, `Start()`, set it again, then
   apply settings. Check `FrameType()` on every frame; queued frames keep the
   old mode, and `GrayscaleData()` on an object frame is null. (§11)
4. **`NextFrame()` returns the OLDEST frame** as `std::shared_ptr<const Frame>`.
   Drain to the newest every loop. Freeing is by dropping the pointer; there
   is no manual `Release()`. (§11)
5. **`ImagerWidth()` / `ImagerHeight()` are `double`s, the sensor size in mm.**
   Pixel counts are `PhysicalPixelWidth()` / `PhysicalPixelHeight()`;
   `Width()` / `Height()` describe an ROI. (`lib/camera.cpp`, SDK header)
6. **`SetImagerGain` takes `eImagerGain`**, not `int`. Cast explicitly. (SDK
   header)
7. **The exposure ceiling is `floor(1e6/fps) − 200` µs.** Re-read
   `MaximumExposureValue()` after any frame-rate change, and set the rate
   before the exposure. (§3)
8. **The blocking mask is a grid of 640 × 512 cells of 2 px.** (§15)
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
   moves. (§15)
10. **Change settings one at a time while streaming.** Re-sending every
    setting stalls delivery; `Camera::apply()` diffs on purpose. (§14)
11. **`SendEmptyFrames(true)` in object mode.** Without it, "no blobs" and "no
    frames" are indistinguishable. (§13)
12. **`setcap cap_net_raw,cap_net_admin+eip` is stripped on every relink.**
    Every rebuilt binary needs it again. Don't run tools under sudo. (§18)
13. **Two OpenCV versions, on purpose.** C++ uses system OpenCV 4.6; Python
    uses 5.x (`opencv-contrib-python`) in `.venv`. Don't unify them. (§18)
14. **`lib/detect` builds the board layout itself, by the OpenCV 4.7+
    rule.** OpenCV 4.6's `create()` always builds the non-legacy layout, and
    it mislabelled every corner of the old 9×6 board. `legacy_pattern` in
    `config/charuco_board.yaml` only matters for an even row count; the 12×9
    board is the same either way. Python (5.x) sets it with
    `setLegacyPattern()` from the same config and calibrates via
    `CharucoDetector.detectBoard` + `matchImagePoints` + `calibrateCamera`.
    **Never trust a corner count without the fit residual**
    (`layoutResidualPx`, `fit` in `03_detect`). A wrong layout still returns
    every corner, with a residual of ~200 px against ~1 px. (§7, §9, §19)
15. **RMS reprojection error is not an acceptance criterion. Zero empty
    coverage cells (5 × 6 grid) is.** All-fronto-parallel views are a
    separate failure; aim for 30% or more of captures tilted beyond 20°.
    (§7, §8)
16. **Grayscale delivery is bandwidth-bound at 6–8 fps** at any imager rate.
    It is not a bug. (§5)
17. **Calibrate with the IR filter IN and the ring at 15.** The filter shifts
    the focal plane. (§4)
18. **Gain is free at the grayscale operating point** (jitter 0.0543 px
    radial median at gain 7 vs 0.0600 px at gain 0, 2026-10-27). (§6)
19. **Starting a tool right after killing one can hang** while the old
    sockets are bound. Wait a couple of seconds. (§11)

## Keeping the docs current

A standing rule for every change.

- **Same change, matching doc.** Any change that adds or alters a command,
  flag, key, config field, measured number or verified behaviour updates the
  matching doc in the same change:
  - [RUNBOOK](docs/RUNBOOK.md): commands, flags, keys, config fields an
    operator sets, expected output. Both the §1 quick reference line and the
    task's procedure section.
  - [FINDINGS](docs/FINDINGS.md): measured numbers (the §1 key-numbers line
    and the topic section) and rules (§2, kept identical to the list above).
  - [ARCHITECTURE](docs/ARCHITECTURE.md): mechanism, design reasons and the
    files involved.
- **Name the sections.** Every summary lists which doc sections were updated,
  or says that none needed to be and why.
- **Never put history in RUNBOOK.** It is instructions only; history and
  past results go to FINDINGS.
- **Never put mechanism in FINDINGS.** How the code works goes to
  ARCHITECTURE, linked from FINDINGS.
- **Never duplicate the open-items list.** It lives only in
  [FINDINGS → Open items](docs/FINDINGS.md#open-items); everything else links
  to it.
- **Flags come from the source.** Copy every flag and default from the code's
  argument parsing, never from memory.
- **The build/setcap loop appears once,** in RUNBOOK §1.2.

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
- **Verify a change** with the build block in RUNBOOK §1.2 (all eight tools
  build, and `tests/build/test_autocapture` passes) and
  `python/triangulate.py` (all 12 checks must pass). Use a fresh build
  directory when the build configuration changes.

## Open items

The single list is [FINDINGS.md → Open items](docs/FINDINGS.md#open-items).
Don't copy it here.
