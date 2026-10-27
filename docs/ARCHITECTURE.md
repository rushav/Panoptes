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
from their names (FINDINGS §8, §9, §13). The habit that runs through the
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
| `lib/` | `camera` (SDK wrapper), `detect` (ChArUco), `coverage` (acceptance grid), `session` (capture output) | No UI code. Every SDK quirk is handled here, once. |
| `src/NN_name/` | One small program per task, each its own CMake project that compiles the `lib/` sources it needs | Thin: parse arguments, loop, draw, call `lib/`. |
| `python/` | Calibration solve, rig model, triangulation, synthetic study | Separate runtime (OpenCV 5.x in `.venv`); shares only file formats with the C++ side. |

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
ten cameras (FINDINGS §8). That is the cost the split exists to avoid.

There is no top-level CMake project. Each tool builds on its own (RUNBOOK
§3).

---

## 1. Discovery and the SDK frame model

**Files:** `lib/camera.cpp` (`discover()`, `Camera::Camera`,
`Camera::Impl::fetch()`), `src/01_discover/main.cpp`.

**Discovery is a network operation.** The cameras are powered over PoE,
self-assign link-local addresses, and broadcast their presence on UDP 13013.
`CameraManager::X().WaitForInitialization()` starts the SDK's listener and
returns as soon as the *first* device is ready, not all of them (FINDINGS §8).
So `discover()` polls `CameraList` every 250 ms until the count has held for
20 polls (5 s), capped at 120 polls (30 s). The window scales with camera
count. A 1.5 s window, tuned for four cameras, read a plateau mid-enumeration
as "done" and reported 5 of 10. The list can repeat a serial, so results are
collapsed by serial.

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
callbacks. `07_objdiag` sleeps 1 ms between empty polls, because a tight
`NextFrame()` loop can starve the SDK's receive thread (FINDINGS §12).

---

## 2. Two operating modes

**Files:** `lib/camera.h` (`Settings`, `Mode`), `lib/camera.cpp`
(`start()`, `apply()`, `actual()`, `diagnostics()`),
`src/05_identify/main.cpp`, `src/07_objdiag/main.cpp`.

The camera can send two very different things, and the system needs both.

**Grayscale: for calibration.** This is the full 1280×1024 image. A
checkerboard is not a set of bright blobs, so calibration needs the picture.
Frames are ~1.3 MB, delivery is bandwidth-bound at 6–8 fps whatever the
imager rate (FINDINGS §3), and the frames are not synchronised across
cameras. The defaults in `Settings` are the measured working point for
ChArUco capture:

| Setting | Value | Why |
|---|---|---|
| IR filter | in | Calibrate in the same optical configuration you track in. A glass plate in the light path shifts the focal plane, so intrinsics measured without it would not apply (FINDINGS §2). |
| IR intensity | 15 (max) | With the filter in, the ring is the light source. With the ring off, the frame averages 6/255 and detects nothing (§2). |
| Gain | 7 | Gain is free here: corner jitter 0.054 px at gain 7 against 0.056 px at gain 0 (§4). |
| Exposure | 8133 µs | Gave every marker at ~1.5 m on the previous 9×6 board (§2). The ceiling at 30 Hz is 33133, so there is 4× headroom (§1). |
| Frame rate | 30 Hz | Delivery is capped at ~8 fps anyway, and a lower rate raises the exposure ceiling (§1). |

**Object mode: for tracking.** The camera thresholds on board and sends only
blob centroids, each with its own area and roundness. Frames are tiny, so all
ten cameras stream at once, and this is the mode the SDK synchronises across
cameras (unverified here, FINDINGS §11).

- **Exposure:** short, because retroreflective markers are far brighter than
  the scene.
- **Gain:** 0.
- **Threshold:** the brightness cutoff, kept inside the camera's own reported
  range.
- **`SendEmptyFrames`:** on, so a frame with zero blobs still arrives. "Dark
  scene" and "dead stream" are then different observations.

The exact exposure and threshold are not settled. Two sets are in use
(FINDINGS §11).

**Changing mode is asynchronous.** `SetVideoType` takes effect on later
frames while already-queued frames keep the old type (FINDINGS §9).
`start()` sets the type, calls `Start()`, sets it again, then applies
settings, because settings applied while stopped are not reliably kept.
Every fetched frame's type is checked: `grab()` returns nothing for an
object frame, and `grabBlobs()` returns nothing for a grayscale one.

**Settings change one at a time.** Re-sending every setting mid-stream
stalls delivery (FINDINGS §12). `apply()` diffs against the last request and
sends only what changed. It sets the frame rate before the exposure, because
the rate sets the exposure ceiling (§1). `actual()` reads values back from the
camera; it never echoes the request.

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
contain three traps (FINDINGS §13):

- `SetBitMaskPixel` takes cell coordinates.
- `SetBlockingMask` with the wrong buffer size silently clears.
- `UpdateBlockingMask()` sends only chunks that differ from what the SDK
  *believes* the camera holds, and in a new process that belief is "empty".

So `Camera::setMask()` validates the size, commits an all-blocked mask, then
commits the target. Every chunk differs from something, so every chunk is
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
in FINDINGS §13.

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
88 interior corners (FINDINGS §17).

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
old board (FINDINGS §7). `applyLayout()` places the markers by the 4.7+ rule
for the configured `legacy_pattern` and recomputes 4.6's nearest-marker
tables. The result matches 5.x exactly.

**The fit residual, not the corner count, says the detection is right.**
`Detector::layoutResidualPx()` fits a homography from the detected corners'
board coordinates to their image positions and returns the worst residual.
Correct labels give about 1 px; a wrong layout or swapped rows/cols gives
hundreds, with a full corner count (FINDINGS §5). `03_detect` shows it as
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
decision, and the evidence is FINDINGS §5. RMS reprojection error is computed
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
(FINDINGS §6). It needs a different fix, so tilt is tracked separately: the
tool shows the share of captures tilted more than 20° (aim for 30% or more)
and a tilt histogram.

**Session output** (`Session`). Each accepted capture is written as
`frame_NNNN.png`. `session.json` is rewritten after every change and records:

- camera serial, OpenCV version, and the full board record from the YAML
  (measured and nominal sizes, dictionary, `legacy_pattern`, provenance)
- for each frame: marker and corner counts, and the exposure, gain,
  intensity, frame rate and filter state it was taken under

The rig will be moved and recalibrated repeatedly, and a folder of PNGs with
no record of how they were taken is nearly worthless. Sessions go to
`$HOME/panoptes-data/sessions`, outside the repo.

---

## 5. Solve and the distortion model

**Files:** `python/compare_models.py`, `python/fit_intrinsics.py`,
`python/bootstrap_intrinsics.py`.

The solve runs in Python because it needs OpenCV 5.x, which can't share a
process with the C++ tools' OpenCV 4.6 (FINDINGS §16).

**Detection in 5.x.** `CharucoDetector.detectBoard()` finds the corners and
`board.matchImagePoints()` turns their IDs into 3D board coordinates;
`cv2.calibrateCamera` then solves. OpenCV 5 removed the old
`interpolateCornersCharuco` / `calibrateCameraCharuco` path. The board comes
from `python/board.py`. It uses the record in the session's `session.json`,
or `config/charuco_board_old.yaml` for sessions recorded before the board
config existed, after checking that it matches what the session recorded.
`setLegacyPattern()` is set from that record. A wrong value on an even-row
board makes 5.x silently find zero corners (FINDINGS §7). Views with fewer
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
were not saved (FINDINGS §14).

**The final fit** (`fit_intrinsics.py`). This fits Brown-Conrady on every
frame and writes `intrinsics.json` into the session, with per-view error so
a bad capture can be spotted. Given several sessions, it prints the spread
across them. **That spread is the accuracy figure.** Held-out error says how
well the model fits its own data; it says nothing about whether a fresh
capture would land on the same numbers.

**Is a difference real?** (`bootstrap_intrinsics.py`). This refits random
subsets of one session. If resampling reproduces the across-session spread,
pose sampling explains it and nothing physical changed. The principal point
(cx, cy) is the weakest-determined parameter: shifting it can be largely
compensated by shifting every board pose, so the optimiser sits in a shallow
valley. That is the open question for camera 33661, whose cx spans 9.4 px
across three sessions (FINDINGS §14, Open items #1).

---

## 6. The rig model and triangulation

**Files:** `config/layout.yaml`, `python/rig.py`, `python/triangulate.py`.

**Layout.** Ten cameras are given by tape-measured position in **inches**,
the unit the room was measured in, in a volume 61 × 134 × 119 in.
`load_layout()` is the only place that converts to millimetres; everything
downstream is mm. Every camera is aimed at one point (z = 30 in), and world
up fixes roll, so each rotation comes from a look-at construction. All ten
share one intrinsic model, taken from camera 33661. Its distortion
coefficients are placeholders chosen to match the 82° × 70° spec, not the
fitted values (FINDINGS §14).

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

## 7. Synthetic accuracy study

**Files:** `python/accuracy_study.py`, `config/study.yaml`,
`analysis/README.md`, `analysis/accuracy_study.json`.

**Question.** Before the rig is calibrated, how accurately can the mounted
layout triangulate a marker, and where?

**Method.**

- **Grid:** the volume at spacing of 6 in or less, plus a denser grid
  (3 in or less) on a box ±12 in around the perch.
- **Noise:** each point is projected into the cameras that can see it, and
  0.054 px Gaussian noise is added per axis. That is the measured static
  corner jitter (FINDINGS §4).
- **Trials:** each point is triangulated 200 times, and the RMS error is
  reported per axis.
- **Variations:**
  - heat-map slices
  - camera subsets: upper ring, lower ring, random sixes
  - 20 randomly perturbed layouts, to test sensitivity to tape-measure error
- **Execution:** points are processed in parallel batches across processes.
- **Provenance:** the run records the git commit and dirty flag, config
  hashes and library versions.

**Results** (FINDINGS §15).

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

## Not built yet

Frame sync (`cModuleSync`), marker labeling, extrinsics (wand and L-frame),
reference markers, rigid-body pose, and streaming output.
The current state and open problems are in
[FINDINGS.md → Open items](FINDINGS.md#open-items).
