# Architecture

How PANOPTES works, stage by stage, in the order data flows through it. This
explains what each part does, why it was built that way, and where the code is.

- Measured numbers live in [FINDINGS.md](FINDINGS.md) and are cited here by
  section (§N).
- Commands and flags are in [RUNBOOK.md](RUNBOOK.md).
- Camera specifications are in [hardware/cameras.md](hardware/cameras.md).

---

## The boundary this system is built against

OptiTrack's Linux Camera SDK gives three things: camera control, hardware
frame sync, and 2D blob centroids. Everything above that is ours:
calibration, triangulation, marker labeling, rigid-body pose and streaming.
So the design question at every stage is the same: what does the SDK
actually do, measured on this hardware, and what must be built on top?

The SDK is thinly documented, and several of its calls behave differently
from their names (FINDINGS §10, §11, §15). The habit that runs through the
code is to **verify by measurement and read back what the camera reports**,
never trusting a request or a UI readback.

```
                 ┌──────────── built ────────────┐   ┌──── not built ─────┐
discover ─► open/start ─┬─► grayscale ─► ChArUco detect ─► coverage ─► session PNGs
                        │                                              │
                        │                                   python: fit intrinsics
                        │                                              │
                        └─► object mode ─► blob centroids ─► blocking masks
                                                   │
                                                   └─► frame sync ─► labeling ─► extrinsics
                                                       ─► triangulation* ─► rigid body ─► streaming

* triangulation exists, but only inside the synthetic accuracy study
```

---

## lib/ versus src/

| | Contents | Rule |
|---|---|---|
| `lib/` | `camera` (SDK wrapper), `detect` (ChArUco), `coverage` (acceptance grid), `autocapture` (frame selection), `capture_status` (live status file), `session` (capture output) | No UI code. Every SDK quirk is handled here, once. |
| `src/NN_name/` | One small program per task, each its own CMake project that compiles the `lib/` sources it needs | Thin: parse arguments, loop, draw, call `lib/`. |
| `python/` | Calibration solve, rig model, triangulation, synthetic study; Kinova arm client, status and boxed moves (`arm.py`, §9) | Separate runtime (OpenCV 5.x in `.venv`); shares only file formats with the C++ side. |
| `tests/` | Unit tests for `lib/` logic that needs no camera (`test_autocapture`) | Own CMake project; links OpenCV only, no SDK. |

**Why the split exists.** `lib/camera.cpp` carries the reason in a comment.
`grab()` (grayscale) and `grabBlobs()` (object mode) used to have separate
copies of the frame-fetch logic. The copies drifted: `grabBlobs()` never set
the "frame received" flag, so callers that checked it threw away every
object-mode frame. Fetching now happens in one place (`Camera::Impl::fetch()`), and the
same principle applies to discovery, mode changes and masks. A tool that
needs the SDK goes through `lib/` and inherits the fixes.

`lib/camera.h` keeps SDK types out of its header (a private `Impl` struct). A
tool that includes it compiles against OpenCV and the standard library only.

**The exception.** `src/01_discover`, `02_stream` and `03_detect` call the
SDK directly. They were the first steps of characterising the SDK, and
01–02 predate `lib/`. They duplicate SDK handling, and the duplicates have
drifted already: 02 and 03 still use the discovery window that undercounts
ten cameras (FINDINGS §10). That is the cost the split exists to avoid.

There is no top-level CMake project. Each tool builds on its own (RUNBOOK
§3).

---

## 1. Discovery and the SDK frame model

**Files:** `lib/camera.cpp` (`discover()`, `Camera::Camera`,
`Camera::Impl::fetch()`), `src/01_discover/main.cpp`.

**Discovery is a network operation.** The cameras are powered over PoE,
self-assign link-local addresses, and broadcast their presence on UDP 13013.
`CameraManager::X().WaitForInitialization()` starts the SDK's listener and
returns as soon as the *first* device is ready, not all of them (FINDINGS §10).
So `discover()` polls `CameraList` every 250 ms until the count has held for
20 polls (5 s), capped at 120 polls (30 s). The window scales with camera
count. A 1.5 s window, tuned for four cameras, read a plateau mid-enumeration
as "done" and reported 5 of 10. The list can repeat a serial, so results are
collapsed by serial. `discover()` keeps the **first** entry for a serial and
`01_discover` keeps the **last**; neither prefers the `Initialized` one.
`01_discover` uses the same 5 s window; `02_stream` and `03_detect` still
carry the old one (6 stable polls, 40-poll cap; FINDINGS §10).

`Camera`'s constructor calls `discover()` again before looking up its serial.
Because every call waits out the 5 s stability window, opening ten cameras
spends at least ~55 s in discovery (one call in the tool plus one per camera).
*Inferred from the code, not timed.*

**The frame model.** The SDK receives frames on its own threads and queues
them per camera. `NextFrame()` returns the **oldest** queued frame as a
`std::shared_ptr<const Frame>`. Calling it once per loop falls further behind
real time on every pass, so `fetch()` drains the queue and keeps the newest.
Dropping a `shared_ptr` frees that buffer; there is no `Release()`.
(`LatestFrame()` is the SDK's one-call equivalent. The drain loop was kept
because it is what the working tools already run on.)

`fetch()` records what actually arrived in `FrameStats`:

- whether a frame came at all
- whether its `FrameType()` was object mode
- its `ObjectCount()`
- its `FrameID()`, whose gaps are dropped frames that an fps average would hide

Callers use `gotFrame()` to tell "no frame ready yet" from "a frame carrying
nothing". Without that distinction, empty polls overwrite good data and get
counted as frames.

The record is reset on every fetch, including one that finds no frame. So
`modeIsObject()` describes the last *poll*, not the camera. That is the likely
cause of the `WRONG MODE` false alarm (FINDINGS, Open items #2).

`grab()` copies the pixels out (`clone()`), because SDK frame memory dies
with the frame. It refuses any size other than 1280×1024 rather than build a
skewed image.

All tools are single-threaded polling loops, and none registers SDK
callbacks. (`04_capture` adds one thread that only watches the loop and
prints if it stops turning; §5, stalls.) `07_objdiag` sleeps 1 ms between empty polls, because a tight
`NextFrame()` loop can starve the SDK's receive thread (FINDINGS §14).

---

## 2. Two operating modes

**Files:** `lib/camera.h` (`Settings`, `Mode`), `lib/camera.cpp`
(`start()`, `apply()`, `actual()`, `diagnostics()`),
`src/05_identify/main.cpp`, `src/07_objdiag/main.cpp`.

The camera can send two very different things, and the system needs both.

**Grayscale: for calibration.** This is the full 1280×1024 image. A
checkerboard is not a set of bright blobs, so calibration needs the picture.
Frames are ~1.3 MB, delivery is bandwidth-bound at 6–8 fps whatever the
imager rate (FINDINGS §5), and the frames are not synchronised across
cameras. The defaults in `Settings` are the measured working point for
ChArUco capture:

| Setting | Value | Why |
|---|---|---|
| IR filter | in | Calibrate in the same optical configuration you track in. A glass plate in the light path shifts the focal plane, so intrinsics measured without it would not apply (FINDINGS §4). |
| IR intensity | 15 (max) | With the filter in, the ring is the light source. With the ring off, the frame averages 6/255 and detects nothing (§4). |
| Gain | 7 | Gain is free here: corner jitter 0.0543 px at gain 7 against 0.0600 px at gain 0 (§6). |
| Exposure | 8133 µs | Gave every marker at ~1.5 m on the previous 9×6 board (§4). The ceiling at 30 Hz is 33133, so there is 4× headroom (§3). |
| Frame rate | 30 Hz | Delivery is capped at ~8 fps anyway, and a lower rate raises the exposure ceiling (§3). |

**Object mode: for tracking.** The camera thresholds on board and sends only
blob centroids, each with its own area and roundness. Frames are tiny, so all
ten cameras stream at once, and this is the mode the SDK synchronises across
cameras (unverified here, FINDINGS §13).

- **Exposure:** short, because retroreflective markers are far brighter than
  the scene.
- **Gain:** 0.
- **Threshold:** the brightness cutoff, kept inside the camera's own reported
  range (`MinimumThreshold()` / `MaximumThreshold()`, exposed as
  `Camera::minThreshold()` / `maxThreshold()`). A value outside it is said to
  be silently clamped or ignored (FINDINGS §13).
- **`SendEmptyFrames`:** on, so a frame with zero blobs still arrives. "Dark
  scene" and "dead stream" are then different observations.

The exact exposure and threshold are not settled. Two sets are in use
(FINDINGS §13).

**Changing mode is asynchronous.** `SetVideoType` takes effect on later
frames while already-queued frames keep the old type (FINDINGS §11).
`start()` sets the type, calls `Start()`, sets it again, then applies
settings, because settings applied while stopped are not reliably kept.
Every fetched frame's type is checked: `grab()` returns nothing for an
object frame, and `grabBlobs()` returns nothing for a grayscale one.

**Settings change one at a time.** Re-sending every setting mid-stream
stalls delivery (FINDINGS §14). `apply()` diffs against the last request and
sends only what changed; `setThreshold()` and `setExposure()` send one value
each, which is what `05_identify`'s keys use. `apply()` touches the video type
only on a real mode change. It sets the frame rate before the exposure, because
the rate sets the exposure ceiling (§3), and `maxExposure()` asks the SDK on
every call instead of caching, because a ceiling read before a rate change is
wrong. `actual()` reads values back from the camera rather than echoing the
request; the mode, IR filter state and `emptyFrames` are the exceptions and
echo it. `05_identify`'s `]` key ignores the ceiling and caps at a hardcoded
33133.

**Why the mode split shapes the future extrinsics design** (`src/notes.md`).
A board-based extrinsic calibration needs grayscale, which is not
synchronised. If the board moves between two cameras' exposures, no single
board pose fits both views. A wand of retroreflective markers works in
object mode, which is synchronised. That is why the plan is a wand plus an
L-frame (FINDINGS, Open items #7).

`lib/detect.h` provides `plausibleMarker()`, an area and roundness filter on
blobs. No tool calls it yet.

---

## 3. Blocking masks

**Files:** `lib/camera.h` / `lib/camera.cpp` (mask API), `src/07_mask/main.cpp`,
`src/07_mask/README.md`, `config/mask.yaml`, `config/masks/<serial>.json`,
`logs/coverage_*.csv`.

**The problem.** Ten cameras face into one volume, so each sees other
cameras' IR rings as bright, static blobs. In object mode those are
indistinguishable from markers.

**The SDK's answer, and its traps.** The camera can drop regions of its
image *before* blob detection. The mask is a grid of 2-px cells, 640×512 on
this hardware. The SDK's semantics had to be read out of the binary, and they
contain three traps (FINDINGS §15):

- `SetBitMaskPixel` takes cell coordinates.
- `SetBlockingMask` with the wrong buffer size silently clears.
- `UpdateBlockingMask()` sends only chunks that differ from what the SDK
  *believes* the camera holds, and in a new process that belief is "empty".

So `Camera::stageMask()` checks the buffer is exactly cols × rows before
calling the SDK, and `Camera::setMask()` validates the size, commits an
all-blocked mask, then commits the target. Every chunk differs from something, so every chunk is
sent, and the camera ends in the right state whatever it held before.

**Learn** (`mask`, empty volume):

1. Clear any retained mask, so it can't hide the glare being measured.
2. Count down `start_delay_s` (5 s) to let the operator leave.
3. Record `window_s` (30 s) of object frames from every camera.
4. Track bright sources by **position** (within `match_radius_px`, 4 px), not
   by mask cell. A centroid jittering across a cell border would otherwise
   split into two sources, each below the persistence threshold.
5. Mark as static any source seen in more than `persist_fraction` (0.5) of
   frames.
6. For each static source, mask its observed extent plus its blob radius, then
   dilate by `dilate_cells` (2).
7. Warn if a camera's mask covers more than `warn_coverage` (10%) of its
   frame. That means the camera is staring at other rings and should be
   re-aimed, not blinded.

**Apply and verify.** `setMask()` on every camera, then record `verify`
`window_s` (10 s) again and report two things per camera:

- **Residual persistent sources.** These should be zero with an empty volume.
- **Leaks.** A persistent source sitting on a masked cell is impossible if
  the mask reached the camera. A leak means the pixel-to-cell mapping or the
  commit is wrong.

**Persist.** Each camera's mask is saved as JSON with:

- the grid geometry, and the settings it was learned under
- the learn and verify statistics, and the SDK and OpenCV versions
- the masked cells

`--load` behaves as follows:

- It **refuses** a mask whose grid doesn't match the camera, because the same
  cell indices would cover different pixels.
- It **warns** if the current settings differ from the learn-time ones,
  because exposure and threshold decide which glare becomes a blob.
- It prints the mask's age, because a mask is valid only while its camera
  hasn't moved.

**Accept.** A mask is permanent blindness for that region: a marker passing
behind it disappears from that camera. So silencing interference is not
enough. `--coverage` has the operator walk one marker through the volume and
fails if any moment is seen by fewer than `min_observers` (3) cameras. Each
camera's losses are blamed on its mask when they happen within
`near_mask_cells` of it. Results, and the fact that no pass is recorded, are
in FINDINGS §15.

**Measure the SDK.** `--probe` steps through:

1. untouched
2. cleared
3. staged
4. enabled
5. committed
6. disabled
7. re-enabled

At each step it counts how many of the known static sources are still
visible. That separates "staging alone does nothing" from "enable is a live
toggle" and checks whether a camera retains a mask across processes.

---

## 4. Intrinsics capture

**Files:** `src/06_focus` (focus first), `src/03_detect` (settings sweep),
`src/04_capture/main.cpp`, `lib/detect.*`, `lib/coverage.*`,
`lib/session.*`.

**Focus first.** `06_focus` shows the variance of the Laplacian over the
centre half of the frame, with peak hold and a history strip. Twist the lens
past the peak and come back to it. Focus with the IR filter in, for the
same focal-shift reason as above. The number depends on the scene, so it
only finds the peak on one camera; it is not a quality threshold and is not
comparable between cameras.

**The target.** The board is a calib.io ChArUco board: 12×9 squares of
60 mm (nominal), 45 mm ArUco markers, `DICT_5X5_100`, giving 54 markers and
88 interior corners (FINDINGS §19).

- **One definition.** `config/charuco_board.yaml` defines the board:
  - rows and cols
  - nominal and caliper-measured sizes
  - dictionary
  - `legacy_pattern`
  - provenance
- **No defaults.** `loadBoardSpec()` (`lib/detect.cpp`) and `python/board.py`
  read that file, and `BoardSpec` has no defaults. A missing or blank
  measured size stops the tool, and nominal is never used in its place.
- **Override.** `03_detect` and `04_capture` take `--board FILE`;
  `config/charuco_board_old.yaml` keeps the retired 9×6 board for reproducing
  its sessions.
- **Printed.** Both tools print the exact board they loaded at startup, so a
  session can't run against the wrong board silently.

Detection is two-stage. First the markers are found; each carries an ID, so
partial views identify themselves. Then the corners between known markers
are interpolated and refined to sub-pixel precision. Calibration consumes
the corners; the markers only say which corner is which.

**The layout is built in `lib/detect`, not by OpenCV 4.6.** 4.6's
`CharucoBoard::create()` always builds what 4.7+ calls the non-legacy layout.
For an even row count the legacy layout differs, and 4.6 then still returns
every corner, with the IDs on the wrong ones. That is what happened with the
old board (FINDINGS §9). `applyLayout()` places the markers by the 4.7+ rule
for the configured `legacy_pattern` and recomputes 4.6's nearest-marker
tables. The result matches 5.x exactly (checked for 9×6 and 12×9, both flag
values; FINDINGS §9). The old board is kept in
`config/charuco_board_old.yaml` with `legacy_pattern: true`.

**The fit residual, not the corner count, says the detection is right.**
`Detector::layoutResidualPx()` fits a homography from the detected corners'
board coordinates to their image positions and returns the worst residual.
Correct labels give about 1 px; a wrong layout or swapped rows/cols gives
hundreds, with a full corner count (FINDINGS §7). `03_detect` shows it as
`fit`, next to the count of marker IDs beyond the board's range.

**The capture loop.** `04_capture` grabs, detects, draws the detection and
the coverage overlay, and waits for the operator.

- **Space accepts a capture only if every corner (88 on the 12×9 board) was
  found.** Partial
  views are valid data, but mixing them with full ones makes per-view error
  harder to read.
- **Each accepted capture tallies its corners into a 5×6 grid over the image**
  (`Coverage`).
- **Board tilt is estimated with `solvePnP` against a guessed f = 780 px.**
  Only the angle is needed, and a few percent of focal error barely moves it.
- **`u` undoes the last capture** in both the tally and the session folder.

**Acceptance: zero empty cells, not RMS.** This is the central calibration
decision, and the evidence is FINDINGS §7. RMS reprojection error is computed
only where observations exist, so it cannot see a region with no data. On
synthetic data with known truth:

- A centre-clustered set had **better** RMS (0.262 vs 0.223 px), recovered
  fx to 0.2% and fx/fy to 0.013%.
- Yet it was 10× worse across the frame and 2× worse at the edge.

Spatial coverage is what constrains the distortion model at the edges.
So the status line turns green at zero empty cells, and coverage is watched
*during* capture, while the board is still in hand.

A second, independent failure is that all-fronto-parallel views make focal
length and distance ambiguous, which shows up as fx drifting from fy
(FINDINGS §8). It needs a different fix, so tilt is tracked separately: the
tool shows the share of captures tilted more than 20° (aim for 30% or more)
and a tilt histogram. It shows the number; it does not warn below 30%.

**Start-up settings.** `04_capture` starts the camera with
`mocap::Settings` (exposure 8133 µs, gain 7, intensity 15, 30 fps, IR
filter in), overridden by `--exposure US`, `--gain N` (0–7) and
`--intensity N` (0–15). They are sent in `Camera::start()`, frame rate
first, before the first `grab()`. An exposure is checked twice: against
`floor(1e6/fps) − 200` while parsing, so nothing touches the camera, and
against `minExposure()`/`maxExposure()` (the SDK's
`Minimum/MaximumExposureValue()`) once the rate is set, which exits non-zero.
Then `actual()` is read back and printed as the `settings` line; a readback
that differs from the request prints a `WARNING` but doesn't stop the tool,
as the SDK may not report a new value at once. In auto mode gate 4's
reference is the first evaluated frame's readback, so the flags are the
baseline the `+ - [ ]` keys depart from.

**Session output** (`Session`). Each accepted capture is written as
`frame_NNNN.png`. `session.json` is rewritten after every change and records:

- camera serial, OpenCV version, and the full board record from the YAML
  (measured and nominal sizes, dictionary, `legacy_pattern`, provenance)
- for each frame: marker and corner counts, and the exposure, gain,
  intensity, frame rate and filter state it was taken under
- `baseline_settings`: the start-up settings, `source` (`flags` or
  `defaults`), `requested`, `readback`, `readback_matches` and the camera's
  `exposure_range`
- in auto mode only, the extra records in section 5

The rig will be moved and recalibrated repeatedly, and a folder of PNGs with
no record of how they were taken is nearly worthless. Sessions go to
`$HOME/panoptes-data/sessions`, outside the repo. `created` is rewritten on
every flush (FINDINGS, Open items).

---

## 5. Automatic frame selection

**Files:** `lib/autocapture.h`, `lib/autocapture.cpp`,
`config/autocapture.yaml`, `src/04_capture/main.cpp` (`runAuto`),
`lib/coverage.*` (`emptyCellsHit`), `lib/session.*` (`setExtra`, per-frame
extras), `tests/test_autocapture.cpp`.

**What it replaces.** In manual mode the operator holds the board still and
presses space. In `--auto` the operator moves the board slowly and
`AutoCapture` decides per frame. That replaces a human judgement, so every
rule is explicit, named, read from `config/autocapture.yaml` (no defaults:
`loadAutoConfig()` refuses a missing field), and copied into `session.json`.
The gates take corners and a `BoardSpec`, not a camera, so the
wand/extrinsics stage can reuse them.

**The gates,** in order. The first failure names the rejection:

1. **Full board.** All corners found, and `layoutResidualPx()` under
   `max_fit_px` (default 5: correct labels give about 1 px mid-frame and up
   to ~3 px at the edge; wrong labels ~200 px; FINDINGS §7, §9).
2. **Still.** Corner moves since the previous frame, matched by corner ID:
   the **median** must be under `still_px` and the **largest** under
   `still_max_px`. The previous frame must itself have passed gate 1, or the
   verdict is `no_reference`. The reference updates on every evaluated
   frame, accepted or not. `describe()` names which bound failed
   (`moving median ...` or `moving max ...`).
3. **Novel.** `Coverage::emptyCellsHit()` finds an empty cell, or the frame's
   pose bin holds fewer than `min_per_bin`. A bin counts as unfilled below
   `min_per_bin`, not only at zero; otherwise the session-end condition could
   never be met.
4. **Settings unchanged.** Exposure, gain, intensity, frame rate and filter
   equal the reference, taken from `Camera::actual()` at the first evaluated
   frame. The first four are camera readbacks; the filter is the request.
5. **Spacing in time.** `min_interval_ms` since the last accept.
6. **Spacing in position.** Some corner moved `min_move_px` since the last
   accept. Gate 5 alone lets a pose held longer than the interval be taken
   twice, which gate 3 allows while its bin is short. It also stops a corner
   jittering across a cell border from counting as a new cell.

Every value is measured for any frame that passes gate 1, so the live view
can show it, but the verdict is decided strictly in gate order. `evaluate()`
counts every frame's verdict; `commit()` adds an accepted frame to coverage
and its bin, and refuses one that was not accepted.

**Why stillness is a gate.** At 8133 µs, a board moving 20 cm/s at 1.5 m
(~2 mm/px) smears a corner by ~0.8 px, 15× the 0.054 px static jitter
(FINDINGS §6). Neither the corner count nor the fit residual shows blur.
Movement between consecutive frames stands in for motion during the
exposure; it is not a measurement of it.

**Why median plus max.** Gate 2 was a single test, the largest of 88 corner
moves against `still_px` 0.3. A static corner moves ~0.08 px between frames
(two draws of the 0.054 px jitter, FINDINGS §6), but the largest of 88 such
moves has a tail, and with the board static the operator saw HOLD STILL
intermittently. A board that is actually moving moves every corner, so the
median measures it and ignores the tail; 0.3 px is ~4× the static median.
The max is kept, at a looser `still_max_px` (1.5), only to reject a single
gross outlier or a board flexing about one edge, which the median would hide.

**Pose bins:** 5 tilt directions × 3 distance bands.

- **Tilt.** `solvePnP` with `approx_focal_px` (780) and the principal point at
  the image centre, as in `Coverage`. With the board normal n turned to face
  the camera (n.z < 0): tilt about x = atan2(n.y, −n.z), about y =
  atan2(n.x, −n.z). Under `flat_deg` (20, the coverage "tilted" threshold)
  the bin is `flat`; otherwise it is the quadrant of the two signs. `+x`
  means the board faces image-down, `+y` image-right. The unit test checks
  this against which image edge comes out shorter.
- **Distance band,** from apparent size. `apparentSquarePx()` takes the
  component-wise median horizontal and vertical corner-step vectors h and v,
  and returns the largest singular value of [h v]. That is the scale along
  the board direction tilt foreshortens least, which is the tilt axis. A
  spacing along a board axis doesn't work, because compound tilt shrinks
  both axes. At 1.25 m with 25°/25° tilt, the axis spacing read 33.9 px per
  square (the far band); the singular value reads 37.4–37.5 px (mid, as at
  0° tilt).

**Session end:** zero empty cells **and** every bin at `min_per_bin`, or the
operator's `q`.

**What it records** (auto only; a manual `session.json` is byte-identical to
before, apart from `created`):

- per accepted frame, an `auto` object: reason (`cells`, `bin`),
  `motion_median_px` and `motion_max_px`, distance moved since the last
  accept, fit, size, tilt, time. Sessions written before 2026-10-28 have a
  single `motion_px` (the max) instead.
- `autocapture`: the config, bin counts, every verdict's count, and the
  reference settings
- `autocapture_end`: why the session ended

- `stalls`: one object per stall (below), if any

Rejected frames are never written. `--auto-dry` runs the same loop with no
`Session` at all, because the `Session` constructor creates the folder.

**Stalls** (auto modes; `runAuto()` in `src/04_capture/main.cpp`,
`Camera::stateLine()` and `Camera::restart()` in `lib/camera.cpp`). A
grayscale stream can stop mid-session with no error (FINDINGS §11).
`grab()` then returns nothing, and a loop that only waits writes no status
and prints nothing, which looks like a hang. So the gap since the last frame
is timed:

- **1 s:** print `no frames from SERIAL for N s` with `stateLine()`
  (`State()` via `CameraStateText`, `IsCameraRunning()`, `IsDisconnected()`,
  `DataRate()`, `HasAvailableFrames()`, `AllocatedFrameCount()`, the last
  `FrameID` received; all SDK beliefs, not camera readbacks). Again every
  5 s.
- **5 s, then every 5 s until frames return:** `restart()` with the
  session's baseline settings (the start-up `Settings`, which `+ - [ ]`
  changes are discarded for). If the SDK still reports the camera
  `Initialized`: `Stop()`, 200 ms, drain the queue, then `start()` with
  `applied` cleared, so every setting is sent in rule order (rate before
  exposure). Otherwise the handle is re-acquired from `CameraList` by serial
  first; a serial that is absent or not `Initialized` is a failed attempt,
  retried in 5 s.
- **First frame back:** `AutoCapture::forgetPrevious()`, so it cannot pass
  the still gate against a frame from before the gap. After a restart the
  readback is compared with the baseline on up to 10 frames (the SDK may
  report a new value late); a mismatch prints a warning, and gate 4 already
  refuses every frame until the readback matches the reference.
- **Status file** during the stall: rewritten every 250 ms with `stalled`
  set and the last frame's fields.
- **`session.json` `stalls`**, one entry per stall: `last_frame_utc`,
  `end_utc`, `gap_s`, `recovered`, `frames_before`, `accepted_before`,
  `camera_state` (at 1 s), `restart_attempts` (each: `utc`, `after_s`,
  `sent`, `why`, `state_before`, `state_after`), and `verify` (`asked`,
  `readback`, `readback_matches`, `frames`, `note`; null without a restart).
  A stall still open at the end is written with `recovered: false`.

The session folder and every count carry on; nothing is renumbered.
`LoopWatchdog` is the other half: a thread that prints `main loop blocked
for N s in STEP` to stderr if the loop stops turning for 5 s, naming the
step (`grab`, `detect`, `record`, `Camera::restart`, `waitKey`, ...). A stall
inside an SDK call that never returns is not recoverable from the loop, but
it is no longer silent. On hardware (FINDINGS §11) the stalls seen so far
were the camera leaving the network (`Disconnected`, then absent from
`CameraList`); re-acquiring it by serial brought it back once. Whether
`Stop()`/`Start()` alone fixes a stall where the camera stays listed is
**unvalidated**.

**Live status file.** With `--status-file PATH`, `04_capture` rewrites one
JSON file after every evaluated frame, so another process can follow the
capture without sharing the camera (the arm probe, §9). The JSON is
built by `captureStatusJson()` in `lib/capture_status.cpp`.
`writeFileAtomic()` writes `PATH.tmp` and `rename()`s it over `PATH`. On
one filesystem a reader sees the old file or the new one, never part of
one. `python/capture_status.py` still retries an unparseable read once
(after 50 ms) before refusing, for a network filesystem or a hand edit. A
failed write prints once and the capture carries on; the file goes stale,
and readers treat a file older than 2 s as dead. The `schema` string is the
version: change it whenever a field changes meaning or is removed.

`panoptes.capture_status/2`. `null` means not measured on this frame. Schema
1 (before 2026-11-07) has no `stalled`, and its file stops changing during
a stall.

| Field | Type | Meaning |
|---|---|---|
| `schema` | string | `"panoptes.capture_status/2"` |
| `timestamp_utc` | string | wall clock at write, `YYYY-MM-DDTHH:MM:SS.mmmZ` |
| `t_ms` | number | monotonic ms since the capture started |
| `frame` | int | frames evaluated so far, 1-based; strictly increasing |
| `camera_serial` | int | |
| `dry_run` | bool | `--auto-dry` (nothing is being written) |
| `session_path` | string/null | capture's session folder; null in `--auto-dry` |
| `stalled` | object/null | null while frames arrive. After 1 s without one: `seconds` since the last frame, `restarts` (attempts in this stall), `camera_state` (`Camera::stateLine()`). While set, `frame` and every per-frame field below are the last frame's, stale |
| `settings` | object | `exposure`, `gain`, `intensity`, `frame_rate`, `ir_filter`: `Camera::actual()` |
| `image` | object | `width`, `height`, px |
| `markers` | object | `found`, `total` (54) |
| `corners` | object | `found`, `total` (88), `complete` |
| `fit_px` | number/null | layout fit residual (gate 1); null if not a full board |
| `board` | object/null | null with no corners. Otherwise from the corners found; on a partial board, the visible part only: |
| `board.centroid_px` | [x, y] | mean of the detected corners |
| `board.bbox_px` | [x0, y0, x1, y1] | corner extent; not the plate's |
| `board.square_px` | number/null | `apparentSquarePx()`: px per board square along the least foreshortened direction; larger is closer |
| `board.roll_deg` | number/null | `boardRollDeg()`: image angle of the board's x step, atan2(h.y, h.x); image y is down, so positive is clockwise on screen; wraps at ±180 |
| `motion` | object | `median_px`, `max_px` (null before a reference), `still_px`, `still_max_px`, and `still`: gate 2 passed on this frame |
| `decision` | object | `verdict` (`verdictName()`), `reason` (`Decision::describe()`), `bin` |
| `coverage` | object | `grid` {`cols` 5, `rows` 6, `cell_width_px`, `cell_height_px`, `id`: `"row * cols + col"`}, `empty_cells` (ids, ascending), `empty_count`, `total_cells` |
| `pose_bins` | object | `min_per_bin`, `counts` {"near/flat": n, ...: 15 bins}, `short` |
| `accepted` | int | frames accepted (`--auto-dry`: would have been) |
| `complete` | bool | session end condition met (0 empty cells, every bin at `min_per_bin`) |

`still` is true only for verdicts past gate 2 (`redundant`, `too_soon`,
`same_pose`, `settings_changed`, `accept`), never for `not_full`, `bad_fit`,
`no_reference` or `moving`. `full_and_still()` in the reader is false while
`stalled` is set. The fixture `tests/fixtures/capture_status.json`
is written by the C++ test (`PANOPTES_STATUS_FIXTURE=PATH
tests/build/test_autocapture`), so the Python reader is tested against the
writer's actual output.

**Status.** `tests/test_autocapture` checks the gates on synthetic corner
sets: the real board projected through a pinhole camera, with seeded noise.
**Unvalidated on real data** (FINDINGS, Open items). The validation rule is
in RUNBOOK §9.

---

## 6. Solve and the distortion model

**Files:** `python/compare_models.py`, `python/fit_intrinsics.py`,
`python/bootstrap_intrinsics.py`.

The solve runs in Python because it needs OpenCV 5.x, which can't share a
process with the C++ tools' OpenCV 4.6 (FINDINGS §18).

**Detection in 5.x.** `CharucoDetector.detectBoard()` finds the corners and
`board.matchImagePoints()` turns their IDs into 3D board coordinates;
`cv2.calibrateCamera` then solves. OpenCV 5 removed the old
`interpolateCornersCharuco` / `calibrateCameraCharuco` path. The board comes
from `python/board.py`. It uses the record in the session's `session.json`,
or `config/charuco_board_old.yaml` for sessions recorded before the board
config existed, after checking that it matches what the session recorded.
`setLegacyPattern()` is set from that record. A wrong value on an even-row
board makes 5.x silently find zero corners (FINDINGS §9). Views with fewer
than 6 corners are dropped.

**Choosing a model** (`compare_models.py`). Training RMS can't choose
between models: more coefficients always fit the training views better,
including by contorting where data is thin. So the script fits
Brown-Conrady (5), rational (8) and fisheye on a training split and judges
them on two checks the optimiser never saw:

- **Held-out reprojection error.** Each held-out view gets its own pose from
  the fitted intrinsics; the test is whether the lens model generalises.
- **Edge straightness.** Board rows are straight in reality, so after
  undistortion the corners in the outer 20% of the frame must lie on lines.

The split uses a fixed seed and reports how many grid cells the test set
covers. A split that put every edge view in training would make every model
look good. Brown-Conrady was chosen; the held-out numbers behind that choice
were not saved (FINDINGS §16).

**The final fit** (`fit_intrinsics.py`). This fits Brown-Conrady on every
frame and writes `intrinsics.json` into the session, with per-view error so
a bad capture can be spotted. Given several sessions, it prints the spread
across them. **That spread is the accuracy figure.** Held-out error says how
well the model fits its own data; it says nothing about whether a fresh
capture would land on the same numbers.

**The camera file** (`fit_intrinsics.py --write-camera`, `write_camera()`).
After the per-session fits, one more Brown-Conrady fit over every frame of
every session given, written to `config/intrinsics/<serial>.yaml`. That is
the per-camera result; before it, nothing per-camera was written outside
the session folders. The sessions must agree on camera serial, image size,
board spec and read-back camera settings, or nothing is written. The file
holds K and dist, the joint RMS and view count, `uncertainty_px` (the range
of the per-session fits, as the spread is the accuracy figure), each
session's own fit, the board, the settings, the capture dates, the fit time
and the OpenCV version, plus `status` when `--status TEXT` is given (a
verdict such as "done, marginal (...)", kept in the generated file so the
file is refitted, never hand-edited). It is plain `%YAML:1.0` that `cv2.FileStorage`
reads, written atomically. An existing file is refused before any fitting
unless `--force`. Nothing reads it yet: the synthetic studies still take
`camera_model` from `config/layout.yaml`.

**Is a difference real?** (`bootstrap_intrinsics.py`). This refits random
subsets of one session. If resampling reproduces the across-session spread,
pose sampling explains it and nothing physical changed. The principal point
(cx, cy) is the weakest-determined parameter: shifting it can be largely
compensated by shifting every board pose, so the optimiser sits in a shallow
valley. It was the suspected cause of camera 33661's 9.4 px hand-held cx
spread; two arm-driven sessions of that camera agree to 0.6 px (FINDINGS
§16), so the spread came from the hand-held method, not the camera.

---

## 7. The rig model and triangulation

**Files:** `config/layout.yaml`, `python/rig.py`, `python/triangulate.py`.

**Layout.** Ten cameras are given by tape-measured position in **inches**,
the unit the room was measured in, in a volume 61 × 134 × 119 in.
`load_layout()` is the only place that converts to millimetres; everything
downstream is mm. Every camera is aimed at one point (z = 30 in), and world
up fixes roll, so each rotation comes from a look-at construction. All ten
share one intrinsic model, taken from camera 33661. Its distortion
coefficients are placeholders chosen to match the 82° × 70° spec, not the
fitted values (FINDINGS §16).

**Projection** (`Rig.project`) is standard OpenCV Brown-Conrady, vectorised
over all cameras and points, with an **analytic Jacobian**
(pixel ← distorted ← normalised ← camera ← world). The self-test checks it
against `cv2.projectPoints` and against finite differences.

**Undistortion** uses 8 Newton steps on the distortion function. Newton
converges quadratically to machine precision. `cv2.undistortPoints`' default
fixed-point iteration stops around 1e-4 px at the frame edge, which is too
loose for a zero-noise test.

**Visibility.** A camera sees a point only if three things hold:

1. the point is in front of the camera
2. it projects onto the sensor
3. its undistorted radius is within what the image corners map to

The third test matters because Brown-Conrady, far outside its fitted range,
can fold points from behind the lens edge back onto the sensor, creating
phantom observations. `rig.py` also refuses coefficients whose distortion is
not monotonic out to that radius.

**Triangulation** has two stages.

1. **DLT seed.** A linear solve on undistorted, normalised rays, with Hartley
   conditioning: world coordinates are shifted and scaled to unit size and
   each row is normalised. Coordinates are ~3000 mm, and unconditioned rows
   would mix unit rotation terms with thousand-mm translations, swamping the
   SVD's null vector.
2. **Levenberg-Marquardt** on total reprojection error in **distorted pixel
   space**:
   - Marquardt damping scaled by diag(H), adapted per point.
   - Infinite cost for any step that puts the point behind an observing
     camera.
   - A point drops out of the batch once its step falls below 1e-10 mm.

**Why two stages.** DLT minimises an algebraic error with no physical
meaning. LM minimises pixel error, which is the maximum-likelihood estimate
under Gaussian centroid noise, but it needs a starting point. DLT provides it.

Fewer than two observers returns NaN plus the observer count, instead of a
guess. One code path handles every observer count: a boolean mask zeroes the
rows of cameras that didn't see the point.

`predicted_cov()` gives the first-order covariance σ²(JᵀJ)⁻¹. It is
independent of the Monte Carlo, so agreement between the two checks both.

**The self-test gates everything.** `python triangulate.py` runs 12 checks:

- projection matches OpenCV
- the Jacobian matches finite differences
- undistortion round-trips
- the rotations are proper
- the aim point lands on the principal point
- zero-noise recovery to sub-micron with all observers and with exactly two
- one observer returns NaN
- refinement never ends worse than its seed
- Monte Carlo RMS agrees with the prediction within 5%

The study refuses to report numbers if any check fails.

This triangulator is used today **only by the synthetic study**. Real
triangulation still needs frame sync, marker labeling and extrinsics.

---

## 8. Synthetic accuracy study

**Files:** `python/accuracy_study.py`, `config/study.yaml`,
`analysis/README.md`, `analysis/accuracy_study.json`.

**Question.** Before the rig is calibrated, how accurately can the mounted
layout triangulate a marker, and where?

**Method.**

- **Grid:** the volume at spacing of 6 in or less, plus a denser grid
  (3 in or less) on a box ±12 in around the perch.
- **Noise:** each point is projected into the cameras that can see it, and
  0.054 px Gaussian noise is added per axis. That is the measured static
  corner jitter (FINDINGS §6).
- **Trials:** each point is triangulated 200 times, and the RMS error is
  reported per axis.
- **Variations:**
  - heat-map slices
  - camera subsets: upper ring, lower ring, random sixes
  - 20 randomly perturbed layouts, to test sensitivity to tape-measure error
- **Execution:** points are processed in parallel batches across processes.
- **Provenance:** the run records the git commit and dirty flag, config
  hashes and library versions.

**Results** (FINDINGS §17).

- **Accuracy is a non-issue at this scale.** The perch box reaches 0.077 mm
  median. The whole volume reaches 0.098 mm.
- **Coverage is the real finding: 19% of the volume can't be triangulated
  at all.** Every camera aims at z = 30 in, so the top of the room, above
  about 90 in, falls outside every frustum.
  - The perch at z = 50 in is well inside the covered region: 96% of its box
    is seen by all ten cameras.
  - Anything that must be tracked above about 90 in needs the cameras
    re-aimed. *That consequence is inferred; the study didn't test a
    re-aimed layout.*
- **Both camera heights are needed.** The ceiling cameras look down, so
  depth along their rays is mostly vertical and z is their weak axis. The
  lower ring pins z but is weak in y. Only together are the three axes
  balanced.
- **The conclusions are robust to tape-measure error** (headline spread at
  most 9.3% over 20 perturbed layouts).

**What it isn't.** Every run assumes perfect calibration. Real extrinsics
error adds directly to these numbers, so they are a **floor, not a
forecast**. For scale, the previous rig measured 13.8 mm loop closure over a
9 m circuit.

---

## 9. Kinova Gen3 arm

`python/arm.py` talks to a Kinova Gen3 (6 DOF, Robotiq 2F-85 gripper) over Kinova's
Kortex Python API. It reads state, and makes small Cartesian moves and
joint-space moves whose end points are inside a box. Nothing in it reads PANOPTES output.

**Allow-lists.** The Kortex `BaseClient` stub carries reads and motion
commands side by side, so `Arm` never exposes it. Each use wraps it in
`_ReadOnly`, which raises on any method name not on the list it was built
with:

| List | Calls | Used by |
|---|---|---|
| `BASE_READS` | `GetFirmwareBundleVersions`, `GetProductConfiguration`, `GetMeasuredCartesianPose`, `GetMeasuredJointAngles`, `GetServoingMode`, `ComputeForwardKinematics`, `ComputeInverseKinematics` | every command |
| `DEVICE_CONFIG_READS` | `GetSerialNumber` | `status` |
| `DEVICE_MANAGER_READS` | `ReadAllDevices` | `status` |
| `MOTION_CALLS` | `OnNotificationActionTopic`, `ExecuteAction`, `Unsubscribe`, `Stop` | `Arm._execute()`: `reach_pose()` and `reach_joints()` with `--yes`, after every check |
| `STOP_CALLS` | `Stop` | `stop` |

`ExecuteAction` accepts any Kortex `Action` (twist, wrench, software
e-stop, sequences), so being on the list is not enough. `reach_pose()` and
`reach_joints()` build the two actions sent (`reach_pose` always with a
Cartesian speed constraint, `reach_joints` with none unless asked),
and `_execute()`, the only caller of `ExecuteAction`, checks
`WhichOneof("action_parameters")` is one of those two before sending.

**One `BaseClient` per router.** Every `BaseClient` registers itself with
the router as *the* receiver of Base notifications:
`RouterClient.notificationService` is a class-level dict keyed by service
id, so the last instance created wins. A second `BaseClient` for motion
would take the `ACTION_END` the first is waiting for. `Arm` makes one, in
`connect()`, and every wrapper shares it. (*Inferred* from the wheel's
source; not exercised on the arm.)

**A move,** following Kinova's `102-Movement_high_level` example:

1. `load_limits()` reads `config/arm.yaml` with `cv2.FileStorage`. Every
   field is required; speeds and timeout are capped in code (0.20 m/s,
   30 deg/s, 300 s), so a typo in the file can't ask for a fast move.
2. Measured pose, then the target. `move` adds to the measured pose; Euler
   angles are wrapped to (−180, 180].
3. Checks, before anything is sent: current pose **and** target inside the
   box; estimated time (larger of distance / translation speed and largest
   angle / rotation speed) within `move_timeout_s`; servoing mode
   `SINGLE_LEVEL_SERVOING`. `arm.py` refuses a wrong mode rather than
   setting it, so it never sends `SetServoingMode`.
4. Without `--yes`, stop here.
5. `reach_pose()` checks the box again, subscribes with
   `OnNotificationActionTopic`, sends `ExecuteAction`, and waits on a
   `threading.Event` that the callback sets on `ACTION_END` or
   `ACTION_ABORT`. Subscribing first means a fast `ACTION_END` can't be
   missed. On timeout or Ctrl-C it sends `Stop`. `Unsubscribe` runs in
   `finally`.
6. Measured pose again, and the error to target. The angle error is a
   per-axis Euler difference, wrapped. Near theta_y = ±90° one orientation
   has several Euler triples, so it can look large while the orientation
   matches.

**Joint limits** come from `config/arm.yaml`, `joint_1_limit_deg` …
`joint_6_limit_deg`, because Kortex has no read for them: in the 2.6 API every
limit read (`Base.GetAllJoints{Speed,Torque}{Hard,Soft}Limitation`,
`Get{Twist,Wrench}{Hard,Soft}Limitation`, `ControlConfig.GetKinematic*Limits`)
covers speed, acceleration, torque, force or twist (FINDINGS §20). So there
is nothing to add to `BASE_READS`; the values are copied by hand from the Web
App. They are asymmetric, so each is a `[low, high]` pair. `load_limits()`
puts them in `lim["joint_limits_deg"]` as `{actuator: (low, high) |
"continuous" | None}`; all six keys are required, so a missing one can't
silently mean "no limit". One function, `joint_limit_checks(lim, joints,
margin)`, returns `(refusals, warnings)`. It wraps each angle to ±180° and
refuses one less than `margin` inside either end (`joint_margin_deg()`, the
distance to the nearer end). All three callers use it:

| Caller | Margin | Null limit |
|---|---|---|
| `check_joint_move()` | 0 | one `WARNING` line with the target's angles, to `warn` (default `print`) |
| `arm_capture.joint_margin()`, teach | `TEACH_MARGIN_DEG` (10°) | `WARNING` with the pose's angles; the pose saves |
| `arm_capture.ik_check()` | `IK_JOINT_MARGIN_DEG` (5°) | not checked per sample; the session plan logs one `WARNING    at P0` with P0's angles |
| `arm_capture.build_taught()`, a viewpoint's IK solution | `VIEWPOINT_MARGIN_DEG` (10°) | not checked |
| `arm_capture._plan_hubs()`, the hub | `TEACH_MARGIN_DEG` (10°) | not checked |

A null limit (none now) warns rather than refuses, so a joint whose limit is
unknown stays usable; a set limit always refuses. `status` prints
each angle wrapped to ±180° with `fmt_joint_limit()`.

**A joint move** (`joints`, `goto-pose`, `session --from-pose`) has the same
shape, through `check_joint_move()` and `reach_joints()`. It exists because
`reach_joint_angles` skips the Cartesian planner, which rejects a path from or
across a singularity (FINDINGS §20).

- **The box** applies to `ComputeForwardKinematics` of the target, on the
  arm, the same pose the arm would report there, and the current pose must be
  inside too. `reach_joints()` repeats the check. Between the end points the
  tool follows an arc in joint space, which nothing checks.
- **Other checks:** six angles, the joint limits (below), servoing mode.
  Limited joints are sent in [0, 360), as the arm reports them. A continuous
  joint (`continuous_joints()`, from `config/arm.yaml`) is sent as the
  measured angle plus the change `CableTracker.plan()` picks (cable
  winding, below); without a tracker, the wrapped difference, at most 180°.
  IK returns continuous joints unwrapped (−444° seen, FINDINGS §20).
  `reach_joint_angles` accepts a continuous joint above 360° and turns the
  short way (FINDINGS §20, 2026-11-02).
- **Speed.** `reach_joints(joints, lim, dps=None)` leaves the action's
  `constraint` field unset unless `dps` is given, as in Kinova's
  `102-Movement_high_level/01-move_angular_and_cartesian.py`; the arm then
  plans its own profile. With `dps` it sends `JOINT_CONSTRAINT_SPEED`, which
  the arm checks against its acceleration limit and rejects for a short move
  (FINDINGS §20, rule 21). `--speed-constraint DEG_S` on `joints`,
  `goto-pose` and `session` is the only way to set `dps`; `config/arm.yaml`
  has no joint speed.
- **Time.** `check_joint_move(dps=, rate_dps=)` estimates the largest wrapped
  joint change over `dps`, else over `rate_dps`, a measured rate; with
  neither there is no estimate and no `move_timeout_s` check, only the
  timeout on the wait itself. `_execute()` stores `Arm.last_move_s`
  (`ExecuteAction` to the end event); `arm.py` prints it as `duration`.
  `_Session` keeps `joint_rates`, the largest joint change over the move's
  time for each joint move that ended and changed some joint by at least
  `JOINT_RATE_MIN_DEG` (1°), logs it as `measured_joint_dps`, and passes
  the median as `rate_dps`. Short moves spend more of their time
  accelerating, so the median tends to overestimate a long move's time. The
  wait, timeout and Ctrl-C are `_execute()`, shared with `reach_pose()`.

**Cable winding** (`CableTracker`, `joint_waypoints()`, `run_joint_move()`
in `arm.py`). The camera cable runs through the wrist, and the continuous
joints (1, 4, 6) wind it: every move takes the short way, yet the turns add
up over a session. The arm reports each joint in [0, 360), so the turns
can't be read back; they are counted in software.

- **Reference and count.** Each continuous joint's reference is the saved
  `hub` pose's angle (`cable_reference()`; the measured joints if no hub is
  saved). A tracker starts with each joint on its turn within 180° of the
  reference: the cable is assumed unwound then. `observe()` adds the
  measured change after every move. After a joint move it takes the change
  within 180° of the commanded one, so a 170° move isn't read as −190°;
  after a Cartesian move, the wrapped change (a pan turns a joint far less
  than 180°).
- **Choosing the turn.** `choose()` takes the short way if that ends
  within reference ± `cable_window_deg` (`config/arm.yaml`, 170). If not,
  it takes the turn inside the window nearest the current angle: the long
  way round, unwinding. With a window under 180° a target can have no turn
  inside it; it then takes the turn nearest the reference, flagged
  `outside the window`. With a `goal`, it takes the turn nearest the goal.
  `check_joint_move(..., cable=, goal=)` applies it and prints
  `cable      J1 … (short way | long way round: unwinding, turns A -> B)`.
- **Long moves as waypoints.** The arm has only been seen turning the short
  way (FINDINGS §20). A long-way move sent in one piece might go the short
  way, winding the cable further while the count said otherwise. So
  `joint_waypoints()` splits any move that turns a continuous joint more
  than 180° into `ceil(turn / CABLE_SEGMENT_DEG)` equal steps
  (`CABLE_SEGMENT_DEG` is 150). A limited joint is interpolated on its
  wrapped angle, the way it physically moves. `check_joint_move()` checks
  every waypoint's FK against the box and estimates the time per waypoint.
  `run_joint_move()` sends the steps, re-checking each from the measured
  joints and sending it the short way, and calls `observe()` after each.
  Waypoints make the box check stricter than a single joint move's, whose
  arc is unchecked. A joint 1 unwind sweeps the whole arm round the base,
  so it is usually refused.
- **In a session** (`_Session`):
  - The tracker is made at the start from P0's joints. The dry run's
    `cable` line gives the reference and the starting turns.
  - Every joint move goes through it.
  - Before a Cartesian move (`move()`), a joint outside the window is
    unwound first. `unwind()` makes an `unwind` entry: a joint move to the
    measured joints with each continuous joint on its in-window turn, the
    same pose. A refused or aborted unwind is a `SessionStop`.
  - The return to P0 lands on the starting turns (`goal=cable.start`). A
    Cartesian return (orbit mode) that ends a turn or more off is followed
    by an unwind to them.
  - Each log line ends `turns J1 +a J4 +b J6 +c`, turns from the reference.
- `arm.py joints` / `goto-pose` and `session --from-pose` make a tracker
  for the one move, assuming the cable unwound. They print its plan and,
  after `--yes`, the turns.

**Abort report.** `_execute()` copies every `ActionNotification` the
callback receives for the action, not only the last. On `ACTION_ABORT`,
`_abort_report()` reads `Base.GetTrajectoryErrorReport` (in `BASE_READS`: a
read of the arm's validation errors for the last rejected trajectory; it may
be stale if this action produced none) and builds two forms: `Arm.last_abort`,
the action sent and each notification and the report as protobuf text, which
`arm.py` prints under `what the arm reported:`; and the returned reason, one
line (`SUBCODE; notifications: …; trajectory errors: …`), which
`arm_capture.py` logs as before. `last_abort` is cleared when the next action
starts.

**Saved poses.** `config/arm_poses.yaml` holds, per name, the measured joints,
the Cartesian pose and the time, plus `square_px` from `teach`. An existing
name is overwritten only with `--force`. `write_poses()` writes the text itself,
through a temporary file and `os.replace`, as `FileStorage` would write
matrices. `goto-pose` moves to the joints; the Cartesian pose is shown and
checked by the session, not sent.

**Wrist check.** `wrist_singular()` wraps actuator 5 to ±180 and flags it
within 10° of 0 or 180: there actuators 4 and 6 line up. 180 is outside
actuator 5's limit, [−120, 128] (`config/arm.yaml`), so only 0 can occur on
this arm; both are checked anyway. `run_session()` checks
the measured joints. With `from_pose`, `go_to_saved_pose()` checks the saved
joints and its distance from the probe's start pose first, makes the joint
move, waits for a settled board, and the normal start checks then run from
there. Without `--yes` it stops at that move unless every joint is within 0.5°
of the pose, because the plan is built from the arm's pose and the status
file's board.

**Taught viewpoints** (`build_taught()`, `run_teach()`). The orbit is computed
from an estimated board position, and most of it fell outside the box
(FINDINGS §20). Taught poses are reachable by construction, and the operator
sees the board at each. Orbit stays behind `--orbit`.

- **Teach.** One Arm session per pose, opened only after Enter, so nothing is
  connected while the Web App jogs. There is no direct read of who holds the
  arm: "free" means `CreateSession` and `Base.Stop` succeed and the servoing
  mode is single-level. `settled_reading()` then gives the square size,
  stored as `square_px` with the pose. `joint_margin()` wants 10° from each
  confirmed limit in `config/arm.yaml` (see *Joint limits*).
- **Grid, in pose space.** `trilinear_pose()` blends the eight taught
  Cartesian (tool) poses: along left-right, then high-low, then near-far,
  each step `interpolate()`. That gives linear position and rotation along the
  shortest arc, `R_a·exp(t·log(R_aᵀR_b))`, which is the curve quaternion slerp
  traces. The camera is rigid on the tool, so the camera's orientation
  blends identically; its position differs only by the tool-frame offset,
  which is unknown (FINDINGS Open items). `square_px` blends trilinearly, only
  to predict the capture band. The earlier joint-space blend failed because
  the taught poses span several arm configurations (FINDINGS §20).
- **Corners take their taught joints** (`build_taught()`). A grid point with
  every index at 0 or n−1 is a taught pose: its `pose` and `joints_deg` are
  the saved ones, `ik_seed` is `"taught"` and `taught` names it. No IK is
  asked, because the arm was there, and its IK has refused such poses even
  seeded with their own joints (FINDINGS §20). The box, joint-limit and wrist
  checks still apply.
- **IK per other viewpoint** (`build_taught()`, at planning, on the arm). A
  viewpoint outside the box is skipped before IK. Otherwise
  `Arm.inverse_kinematics()` is seeded with the joints of the nearest taught
  pose by `pose_distance()`: metres plus `POSE_DIST_M_PER_RAD` (0.5) per
  radian, an arbitrary weighting. If that fails, it is seeded with the last
  viewpoint's joints. Each failed attempt is kept in `ik_failed` as
  `{seed, error}`, also when the second seed then solves. A solution within
  `VIEWPOINT_MARGIN_DEG` (10°) of a joint limit (`joint_limit_checks()`) is
  "unreachable", as is no solution; the wrist check runs on the solution.
  `ik_summary()` counts corners used, non-corners outside the box, solved by
  the nearest seed, solved by the previous seed, and failed with every seed,
  and lists each failure; the plan prints it as the `ik` line and one
  `IK failed` line per failed attempt, and the record keeps it as `ik`.
- **Families and order.** `families()`: in serpentine order, a solution
  joins the first family with a member within `FAMILY_DEG` (90°) of it on
  every limited joint (2, 3, 5: `joint_gap(..., lim)`, wrapped), or starts
  one. Continuous joints are left out: they define no configuration, and
  the cable tracker picks their turn (cable winding, above). `visit_order()` orders the
  viewpoints by capture band (`BAND_ORDER`: mid, then near, then far, by the
  predicted `capture_band`), family by family within a band, and serpentine
  within each family (`serpentine()`: the left-right index reverses each
  row, high-low each layer). Consecutive viewpoints of one family are near
  each other, though not always one grid step apart. Mid comes first because
  its views reach the most cells; the near corners sit at the reach edge
  (FINDINGS §20). The price is hub moves: a family that spans two bands is
  entered once per band (6 → 9 between viewpoints on the 2026-11-02 plan).
- **The hub.** `hub_legs()` counts the planned transitions: a change of
  family, or a limited-joint gap over `FAMILY_DEG` between consecutive
  viewpoints, plus the start and return legs by gap. None for start and
  return when the arm starts at the hub (every joint within
  `POSE_REACHED_DEG`), and at run time no hub move is made from or to the
  hub itself. `_plan_hubs()` prints the
  `families` report and refuses to start if any is planned without a usable
  `hub` pose: present, in the box, `TEACH_MARGIN_DEG` inside every limit.
  Viewpoints can be skipped at run time (their cells already full), so the
  plan isn't the visiting order. `_Session.hub_if_needed()` therefore
  decides again before each viewpoint move and the return. It compares the
  measured joints and the family of the last viewpoint reached with the
  target, and makes a joint move to the hub first if needed. A failed hub
  move skips the viewpoint (an abort counts towards the three-in-a-row
  stop). Cell pans don't change family.
- **Viewpoint moves.** `_Session.move_joints()` and `_reach_joints()`, to the
  planned IK solution. A failed
  viewpoint isn't recovered from, because the next one is a joint move and
  needs nothing from this one; three aborts in a row still end it. Cell pans are
  unchanged, Cartesian, from the pose reached.
- **Speed.** Viewpoint, hub and return moves go through
  `_Session.move_joints()` with the session's `dps` (`--speed-constraint`,
  default none) and its measured rate (above). Nothing bounds the tool's
  speed along the arc; the arm's own joint profile does.
- **Gains.** The probe's rotation gains and camera axes come from one pose and
  are used at all of them. The camera axes are fixed in the tool, so those
  carry; the px/deg and its f are not exact away from P0, and the pan's
  correction step takes up the difference.

**Probe: tool motion to image motion** (`python/arm_capture.py`). The
camera is on the arm and the board is on an easel. The session needs to know
which tool motion moves the board which way in the image. The probe measures
it rather than deriving it from a hand-eye calibration that doesn't exist
yet, so it doesn't depend on how the camera is mounted.

- **Tool-frame targets.** Kortex poses are x-y-z extrinsic Euler angles,
  R = Rz(θz)·Ry(θy)·Rx(θx) (Gen3 user guide, "Cartesian reference
  frames"). A rotation by δ about tool axis k is R·R_k(δ), post-multiplied;
  a translation along tool axis k adds s·R[:, k] to the position. The result
  is converted back to Euler angles. Adding δ to θx directly happens to equal
  the tool-x rotation; adding to θz rotates about base z, and adding to θy
  is neither. So `arm.py move --dt*` is not a tool-frame motion. The probe
  refuses |θy| > 80°, near the Euler singularity.
- **Moves.** All twelve targets (six probes, six returns to P0) pass
  `arm.check_move()` before any is sent. Each move is then checked again
  from the freshly measured pose and sent with `Arm.reach_pose()`. Returns go
  to the measured start pose P0, not "−2°", so errors don't accumulate.
- **Readings.** After `ACTION_END`, the probe skips 3 new frames, then
  averages 3 distinct frames that are full (88/88) and still (status
  `motion.still`). A frame that isn't starts the count over; 15 s without a
  reading stops the probe. Status older than 2 s is refused.
- **Return check.** The centroid after each return must be within 3 px of
  the reading before the probe. More means the camera moved in its mount,
  the easel moved, or the arm didn't return, and the result would mix two
  configurations.
- **Mapping.** Effects are scaled to per degree or per metre. For image x,
  image y and roll, the axis with the largest |effect| among the three
  rotations is chosen, and for x, y and size among the three translations,
  with its sign. `second_best_ratio` records how close the runner-up is.
- **Output.** `config/arm_camera.yaml`, FileStorage-readable, written via
  a temp file and rename. It holds the full sensitivity table, raw effects,
  return errors, start pose and board, and the mapping.

**Session: closed loop on the status file** (`arm_capture.py session`).
`capture --auto` decides which frames to keep. The session only moves the
camera and reads the status file, so a session driven by the arm is
recorded exactly like one held by hand. Mechanism, in `run_session()`:

- **Gains.** `load_gains()` reads `config/arm_camera.yaml`. It takes the two
  rotation axes the probe found move the board right and down, which need
  not be θy and θx, and forms J_rot, their 2×2 px/deg block. J_rot must be
  well conditioned and the two axes distinct. It also takes J_trans (px/m,
  all three translations) and the "closer" axis. The per-axis focal length
  is |J_rot row| × 180/π. It is an aiming gain, not the lens's focal length:
  the rotation is about the TCP, not the camera (FINDINGS §20).
- **Camera axes** (`camera_axes()`). The tool is not aligned with the
  camera, so the session needs the camera's axes in the tool frame. The size
  gains point at the centred board, so they are the optical axis z. A
  rotation about camera x moves the board +f in image y, and about camera y
  −f in image x, so the rotation gains' image-y column gives x, made
  orthogonal to z, and y = z × x. Needs nothing beyond the probe's file.
  The camera's *position* in the tool frame is not found.
- **Aiming** (`rotation_for_aim()`, `_Session._aim()`). The rays through
  `now` and `aim` are ((px − c)/f, 1). The camera turns by the angle between
  them, about their common normal, mapped to tool axes by the camera axes,
  about the TCP. For a centred board that angle is atan(dpx/f). Starting
  off centre, the two rays are what's correct; atan(dpx/f) from there
  overshoots by 1 + tan²θ, which is what the second leg of a long aim
  would do. Each aim starts from the live centroid and pose. A target over
  `two_move_px` from it takes two moves: `first_move_fraction` of the
  rotation, a re-read, then what's left, so the second leg corrects the first
  whatever its error. The total from the viewpoint's base pose is capped at
  `max_rotation_deg`. A last move that misses by over `centre_tol_px` gets one
  correction, composed from where the arm is. If it loses the board, the
  session goes back to the last good pose. Every move's error goes in the
  entry's `aim_error` and on an `aim` log line.
- **Refitting the aim** (`refit_aim()`). After the first of two moves, each
  bbox side's reach from the centroid is extrapolated along the move from how
  it grew so far, and the aim is moved in to keep that inside
  `edge_margin_px`. A tilted board is asymmetric about its centroid and its
  near edge grows fastest, which `aim_for_cell()` can't see from the base
  view. A refit is recorded as `aim_refit_px`.
- **Keeping the board in view** (`aim_for_cell()`). An off-axis board looks
  bigger. A pinhole camera magnifies a point at angle θ off axis by
  ~1 + tan²θ (`growth()`), and a tilted board's near edge grows more again.
  The board's half-size at the base, de-magnified to the centre, is
  re-magnified at the aim and padded by `FIT_SAFETY` (1.15). The aim starts
  at the cell centre and moves toward the image centre (bisection) until the
  board fits inside `edge_margin_px`. It is refused if the board that fits
  no longer overlaps the cell. Without this, corner targets lost the board
  on most tries in simulation.
- **Board estimate** (`estimate_board()`). Depth = f·square/`square_px`, with
  f the geometric mean of the two gains and square the board's `measured`
  size, along the ray through the centroid, into the tool frame by the camera
  axes, into the base frame by P0. The camera is taken to be at the TCP.
  That error is second order for the orbit, because the point is rigid
  with the tool at P0 and the orbit turns the tool about it: the true board
  stays near the optical axis at every viewpoint. `size_gain_distance_m`,
  `square_px`/|size gains|, needs no focal length and is printed beside it.
- **Orbit** (`orbit_viewpoints()`, `build_orbit()`). The viewpoints are
  `orbit_radii_m` × elevation × azimuth, as one nearest-first walk. Azimuth
  is about base z through the board, elevation about the horizontal axis
  across P0's line of sight. P0's orientation is first turned so its optical
  axis lands exactly on the board, then each viewpoint rotates that with the
  position, then adds any roll about the line of sight. So the optical axis,
  not tool z, points at the board (`pointing_error_deg()`), and image roll
  changes only by the commanded roll. Each viewpoint's size is
  `square_px`·r0/r. A viewpoint is visited while any bin of its capture band
  is short, even with no cell wanted, because the viewpoint is itself a new
  tilt. The box and `ik_check()` from P0 clip the cap; `cap` in the record
  and the `cap` line give requested, reachable, per radius, and the cause.
- **Lateral variants** (`build_variants()`, `lateral_variants: 1`). 3 depths along **tool z** (0,
  ±`depth_m`) × {0, ±lateral on each lateral axis}. The lateral axes are
  the probe's "right" and "down" translations; a lateral axis that is z is
  dropped. Each variant gets the compensating rotation
  −J_rot⁻¹·J_trans·t, so the board starts where it was. Its square size is
  predicted from P0's plus the probe's size gains (`square_per_m`·t).
- **Two bands per variant.** The *planner's* band names it: relative to
  P0's square size, mid within ±`mid_band_fraction`, near above, far below
  (`_band_of()`). The *capture* band is what `poseBin()` will call it, from
  `config/autocapture.yaml`'s fixed thresholds (`capture_band()`). Bin
  seeking and futility use the capture band, because capture's bins are
  what get filled. The two agree only if those thresholds are set to P0
  ±`mid_band_fraction`; the dry run prints the values. `depth_sanity()`
  warns if near and far differ by no more than `band_spread_min` of P0's
  size, or if every depth falls in mid, and gives the `depth_m` that would
  fix each. For the orbit the warning is that the radii don't reach all three
  capture bands.
- **What's wanted.** Fixed when a variant starts: the cells empty then,
  plus seekers for its capture band. Seekers are the four corner cells while
  its quadrant bins are short, and the two centre cells while its flat bin
  is short (about 85 px from a centred base, enough for capture's 20 px
  same-pose gate). Before each target the set is filtered against the
  status file, so cells filled meanwhile drop out; nothing is added. A
  corner view that capture bins into another band marks that band *futile*
  for corner seeking (see the band limit below). The next target is the
  nearest remaining cell to where the board is now.
- **IK validation** (`ik_check()`). After the box check and before
  anything is sent, the arm's own IK (`Arm.inverse_kinematics()`, Kortex
  `Base.ComputeInverseKinematics`) runs at `IK_SAMPLES` (4) points along
  the path: `interpolate()` gives linear position and the shortest-arc
  rotation. The first is seeded with the measured joints, each later one
  with the previous solution. Rejected:
  - no solution;
  - any joint with a confirmed limit in `config/arm.yaml` within
    `IK_JOINT_MARGIN_DEG` (5°) of it, after wrapping to ±180° (Kortex
    reports 0–360°; *inferred* from its docs). Null limits aren't checked
    here (see *Joint limits*);
  - any joint changing by more than `IK_JUMP_DEG` (45°) between samples,
    a branch change near a singularity.

  The reason is logged, the entry is `unreachable`, and nothing is sent. The
  IK only approximates the controller's trajectory; it does not prove the
  move will run. Joint order is taken as actuators 1–6 (*inferred*).
- **Each target** (`_Session.visit()`). Every move first passes
  `arm.check_move()` from the freshly measured pose (box, time estimate,
  servoing mode) and then `ik_check()`; a failure of either is logged and
  skipped. After `ACTION_END` the
  session skips 3 frames, then waits for full and still. Three consecutive
  frames under 88/88 mean the board left the view: one step to
  `halfway()` (mean position, half the relative rotation), then back to the
  last good pose. If the board doesn't settle there, or that way back fails
  its checks, the viewpoint is abandoned (below). It waits
  `min_interval_ms` and up to 4 frames for `accepted` to rise.
- **Abandoning a viewpoint** (`ViewpointAbandoned`). Raised wherever the
  next pan would start from a pose without the board in view: after the
  return from a skipped target, after the return from a lost correction,
  and after the return following an abort (`recover()`), whether the return
  ran and the board didn't settle or the return failed its checks.
  `run_session()` catches it per viewpoint, adds an `abandon` entry
  (`abandoned: board not in view`, the reason in `note`), logs `NAME:
  viewpoint abandoned (...)`, and goes on. In a taught session the next
  viewpoint is a joint move, which needs nothing from this pose. Before
  2026-11-02 the first of these ended the session (FINDINGS §20).
- **Reach edge** (`at_reach_edge()`, in `_viewpoint()`). If the first
  `REACH_EDGE_PANS` (2) cell pans tried from a viewpoint are both
  `unreachable` (the IK path check: no solution, a joint within the margin,
  or a branch jump), the rest of its cells aren't tried. An `abandon` entry
  (`abandoned: viewpoint at reach edge`, the untried cells in `note`) and a
  `NAME: viewpoint at reach edge` line record it. Only the first two pans
  count: a viewpoint whose first pan reached goes on to all its cells.
- **Aborts.** `ACTION_ABORT`, or a refused `ExecuteAction`
  (`arm.ActionRefused`), on a validated move raises `TargetAborted`.
  `visit()` logs the entry `aborted: <reason>` with the joints read after
  it. `recover()` then moves back to the last reached pose, validated the
  same way, and the session continues with the next target; if that way
  back fails its checks or the board isn't in view there, the viewpoint is
  abandoned. The count of
  consecutive aborts includes aborted returns, and only a reached *target*
  resets it. At `MAX_CONSECUTIVE_ABORTS` (3) the session raises a fatal
  `ArmError`. A move timeout stays fatal: `Base.Stop` has been sent and the
  arm's state is unknown.
- **Timing.** Each entry accumulates `ik_s` (validation), `move_s`
  (`ExecuteAction` to the end notification), `still_s` (waiting for a full,
  still board) and `decide_s` (`min_interval_ms` plus waiting for an
  accept). `timing_breakdown()` sums them, gives the remainder as other,
  and the time per move. Records from schema 1 have `settle_s` for
  `still_s`.
- **Endings.** `SessionStop`, from completion, an exhausted list, `q`
  (polled between moves; the terminal is in cbreak mode), Ctrl-C (raw, or
  `arm.ArmInterrupted` from a move, after `Base.Stop`) or a stale status
  file, returns to P0. A status with `stalled` set is not a dead capture:
  `_Session.live()` calls `ride_out_stall()`, which holds (it is only called
  between moves, so the arm is still) and polls until a fresh unstalled
  status, for up to `STALL_WAIT_S` (30 s) on its own clock. A stale or
  unreadable file during that wait counts as still stalled, since capture
  may be inside `Camera::restart()`. No recovery in time is a `SessionStop`
  (`capture stalled and did not recover within 30 s`), so it returns to P0.
  Time held is added to `settle()`'s deadline, so a recovered stall doesn't
  turn into a still timeout; it still counts in that entry's `still_s`. A stale file whose last word was `complete: true` is
  a normal end, since capture exits on completion. A fatal `arm.ArmError`
  (three consecutive aborts, or a move ending other than `ACTION_END` /
  `ACTION_ABORT`, such as a timeout) ends the session without another move.
  A board out of view never ends it; it abandons the viewpoint.
- **Record.** `session_arm.json`, schema `panoptes.session_arm/8`, in the
  capture's session folder (`session_path` from the status file), rewritten
  atomically after every target. It holds:
  - P0 and its joints;
  - for a taught session, `families` (names per family and `hub_legs`) and
    the `hub` pose used (null if none was needed);
  - the config used, including the IK parameters and the joint limits
    (`limits.joint_limits_deg`: `[low, high]`, `"continuous"` or null);
  - the gains' source and axes, `f_px` and `cam_axes_in_tool`;
  - `mode` (`taught`, `orbit` or `lateral`) and `taught_poses`, by name and
    value (null otherwise);
  - `board_estimate` and `cap` (null for a taught session);
  - the viewpoints or variants, with both bands (viewpoints also `orbit`, or
    `grid`, the joints `joints_deg` (taught at a corner, else IK), `family`,
    `ik_seed`, `taught` and `ik_failed`); `families.ik`, the `ik_summary()`;
  - every target: kind (`variant`, `cell`, `back`, `hub`, `abandon`, `unwind`,
    `return`), variant,
    cell, aim, outcome, the measured pose, capture's verdict, bin and
    accept, and the timers. Each commanded pose carries its absolute and
    `rel_p0` forms (translation and rotation vector in P0's tool frame), the
    joints before it, the IK solution, and whether it was `sent`; a joint
    move has `joints_deg` and `joint_speed_dps` instead of the IK solution. Aimed
    entries carry `aim_error` (leg, centroid, error, miss) and any
    `aim_refit_px`. Aborted entries also carry `joints_after_abort_deg`. A
    joint move past a null limit adds its warning to the entry's `warnings`.
    A joint move's commanded entry carries `cable` (per continuous joint:
    `sent_deg`, `turns_before`, `turns_after`, `how`), and `waypoints` when
    split. Every entry has `cable_turns` after it; `unwind` entries are the
    inserted unwinding moves;
  - `cable`: `reference_source`, `window_deg`, `reference_deg`,
    `start_deg`, `cumulative_deg`, `turns` and `unwinds`, rewritten with
    the record.

  The summary is stored too: `accepted` is capture's own count delta;
  `timing` is `timing_breakdown()`; `abandoned` and `reach_edge` count the
  two kinds of `abandon` entry; `capture_stalls` and `capture_stall_wait_s`
  count the stalls in the top-level `capture_stalls` list (each: `start_s`,
  `utc`, `frame`, `accepted`, `first_report`, `recovered`, `waited_s`,
  `last_report`, and `frame_after` if it recovered); `cable_turns_end`,
  `cable_net_turns` (end against start) and `cable_unwinds`. Schema 1 (2026-10-28) lacks joints, IK,
  `sent` and the timers; schema 2 lacks the board estimate, the cap and the
  aim errors; schema 3 lacks `mode` and `taught_poses`; schema 4 has the
  user guide's limits as `ik.joint_limits_deg` instead of
  `limits.joint_limits_deg`, no `warnings`, and a taught session's
  `joints_deg` is a joint-space blend, with no `families`, `hub` or `hub`
  entries; schema 5 has no `abandon` entries, no `abandoned` or
  `reach_edge`, and visits taught viewpoints family by family; schema 6 has
  no `capture_stalls`, and a stalled capture ended the session as dead;
  schema 7 has no `cable`, `cable_turns` or `unwind` entries, and turned
  continuous joints the short way always.

**Band limit (inferred, not measured).** `poseBin()` bands by
`apparentSquarePx()`, which is size-in-image. A camera aimed so the board
sits at a frame corner sees it ~1.3–1.5× bigger at the same distance, so
corner views from a mid or far P0 are binned a band closer. In simulation
(pinhole, no distortion), the mid and far quadrant bins never filled from
inside the box. The real lens's barrel distortion shrinks the effect by an
unknown amount. On the first hardware session a top-right view was binned
mid while P0 reads far (FINDINGS §20). Banding by the solvePnP distance `poseBin()` already
computes would remove it; that changes capture, so it is an open item.

**Why both ends in the box.** This holds for `reach_pose` only; a joint move
is an arc. If the tool moves in a straight line, a box
being convex means both ends inside keeps the whole path inside. Kinova's
user guide does not say a `reach_pose` path is straight (*inferred*, not
verified). The box bounds only the tool point; the links can leave it.

**Connection.** TCP to port 10000, then a Kortex session (`SessionManager`).
`Arm` is a context manager: `__exit__` closes the session and the transport,
also on an exception. Before Kortex connects, a plain socket connect with a
3 s timeout checks reachability, because `TCPTransport.connect()` has no
timeout and blocks for the OS TCP timeout on an unreachable host. Each RPC
gets a 3 s timeout (Kortex default 10 s).

**Session timeout: a crashed script releases the arm.** `CreateSessionInfo`
asks the arm to drop the session after 10 s with no traffic
(`session_inactivity_timeout`; `connection_inactivity_timeout` 2 s), the
values in Kinova's `examples/utilities.py`. While the process lives,
`SessionManager`'s background thread sends `KeepAlive` every 150 ms. If the
process dies, the keep-alives stop and the arm closes the session on its own.
**Whether closing the session also stops a move in progress is not known**:
the user guide doesn't say and it hasn't been tried. Don't count on it; the
E-stop is the stop (RUNBOOK §13).

**Device routing.** Serial number comes from `DeviceConfigClient` addressed
to the BASE's device id, found with `DeviceManagerClient.ReadAllDevices()`;
the same pattern as Kinova's `100-Overview/01-devices_routing.py`. Firmware
is `BaseClient.GetFirmwareBundleVersions()`, whose `main_bundle_version` is
a string; the `uint32` from `DeviceConfig.GetFirmwareVersion()` is not
decoded.

**Dependencies.** Separate from the camera side, in
`python/requirements-arm.txt`, installed with `--no-deps` because the wheel
pins a protobuf that cannot import on Python 3.12 (FINDINGS §20).

**Network.** The arm shares `enp6s0` with the cameras: a second static
address, 192.168.1.11/24, next to the cameras' link-local one
(RUNBOOK §13).

---

## Not built yet

Frame sync (`cModuleSync`), marker labeling, extrinsics (wand and L-frame),
reference markers, rigid-body pose, and streaming output.
The current state and open problems are in
[FINDINGS.md → Open items](FINDINGS.md#open-items).
