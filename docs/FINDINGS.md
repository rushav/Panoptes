# Mocap findings

What was learned running OptiTrack Prime 13W cameras and calibrating them on
this rig, and why it matters. This file is authoritative for **operational**
findings: what settings work, what the SDK actually does, what was measured.

Camera inventory, serials, model specifications and network layout live in
[hardware/cameras.md](hardware/cameras.md) and are not
repeated here.

Commands are in [RUNBOOK.md](RUNBOOK.md).

Every claim below marked **Verified** includes the measurement it came from.
Claims taken from code comments or from reading the SDK binary, with no
measurement recorded in this repo, are marked as such. Open items and open
questions are collected at the end.

Sections 1–10 were written against the GUI tool `src/intrinsics_capture`,
removed in commit a91b3c6. Their measurements stand; where they describe that
tool's behaviour, a note says what the current code does instead.

---

## 1. Exposure ceiling is set by the frame period, not fixed

**Verified — corrects an earlier assumption.**

The SDK re-reports `MinimumExposureValue()` / `MaximumExposureValue()` after
every `SetFrameRate()`, and the maximum tracks the frame period exactly.
Measured on camera 33661, setting each rate and then reading the range back:

| Frame rate | SDK exposure range | `floor(1e6/fps) − 200` |
|-----------:|-------------------:|-----------------------:|
| 240 Hz | 10 – 3966 | 3966 |
| 120 Hz | 10 – 8133 | 8133 |
| 60 Hz | 10 – 16466 | 16466 |
| 30 Hz | 10 – 33133 | 33133 |

Setting exposure to the reported maximum reads back at that maximum in every
case. Exposure units are microseconds; the constant 200 µs is readout overhead.

Two earlier beliefs were both wrong, in opposite directions:

- *"Exposure above ~6000 is clamped at 120 Hz."* Wrong number. The ceiling at
  120 Hz is 8133.
- *"The ceiling is a hard 8133 regardless of frame rate, tested at 120 and
  30 Hz."* This is what the **tool** showed, not what the camera does. Stage 1
  read the exposure range once at startup and never refreshed it, so the slider
  stayed pinned to the 120 Hz bound of 8133 even after switching to 30 Hz.
  Brightness rising all the way to 8133 was consistent with that stale bound
  and looked like a hard limit. The camera at 30 Hz will in fact go to 33133.

The tool now re-reads the range on every frame rate change and after applying
the startup defaults. Confirmed live: opening at 30 Hz logs
`exposure range at 30 Hz: 10..33133`.

*Current code:* `Camera::maxExposure()` (`lib/camera.cpp`) asks the SDK on
every call instead of caching, and `Camera::apply()` sets the frame rate
before the exposure so the exposure is not clamped against the old ceiling.
`src/05_identify` still caps its `]` key at a hardcoded 33133, which is
correct only at 30 Hz.

**Why it matters:** at 30 Hz there is 4× more exposure headroom available than
the working default of 8133 uses. If a board is ever too dark at working
distance, exposure is not the constraint people assumed it was.

---

## 2. The IR bandpass filter can stay IN for ChArUco capture

**Verified.**

The Prime 13W has a switchable 850 nm bandpass. With it IN, and the IR ring
doing the illuminating rather than room light:

| Setting | Value |
|---------|-------|
| IR filter | IN |
| IR intensity | 15 |
| Imager gain | 7 |
| Exposure | 8133 µs |
| Frame rate | 30 Hz |

Two 20-frame sessions captured this way at ~1.5 m gave **27/27 markers and
40/40 ChArUco corners in every single frame** (sessions
`intrinsics_33661_20260824_234853` and `..._235746`).

**Why it matters:** calibration then happens in the same optical configuration
the cameras track in. There is no filter-removal focal shift to characterise or
correct for — the lens never moves between calibrating and operating. This is a
better position than calibrating with the filter out and hoping the shift is
negligible.

For contrast: with the filter IN but the IR ring off and exposure at 400 µs,
the frame averages 6/255 and detects zero markers. The filter is only workable
together with the ring.

---

## 3. Grayscale delivery is bandwidth-bound at 6–8 fps

**Verified.**

Full-resolution 1280×1024 grayscale arrives at roughly 8 frames per second, at
both 120 Hz and 30 Hz imager rates. Measured by counting frames drained from
the SDK queue over 3-second windows:

```
decim=1  received 14.7 fps  1280x1024 | FrameRate()=120 ActualFrameRate()=120  DataRate=10.7 MB/s
decim=2  received  9.7 fps  1280x1024 | FrameRate()=120 ActualFrameRate()=120  DataRate=10.8 MB/s
decim=4  received 10.0 fps  1280x1024 | FrameRate()=120 ActualFrameRate()=120  DataRate=10.7 MB/s
```

`DataRate()` sits at ~10.7 MB/s across the board — about 8 frames of 1.31 MB.
`SetGrayscaleDecimation()` has no effect on this camera:
`GrayscaleDecimation()` stays 0 and the frame size never changes.

The imager genuinely is running at the configured rate — `ActualFrameRate()`
confirms it — the *delivery* of full-resolution grayscale is what is limited.
The removed GUI showed the two numbers separately; no current tool does.

**Why it matters:** harmless for tripod calibration, where the board is held
still. It would matter for anything wanting fast grayscale.

---

## 4. Imager gain is free at this exposure

**Verified.**

Corner jitter on a static board over 20 frames, same scene, same exposure
(8133 µs), IR filter IN, IR intensity 15, 30 Hz. Jitter is the per-corner
standard deviation of position across frames, over the 40 corners present in
every frame:

| Gain | Median jitter | Mean | Worst | Corners detected |
|-----:|--------------:|-----:|------:|------------------|
| 0 | 0.0561 px | 0.0568 px | 0.0820 px | 40/40 in all 20 frames |
| 7 | 0.0540 px | 0.0559 px | 0.0912 px | 40/40 in all 20 frames |

Gain 7 is marginally *better* at the median and marginally worse at the worst
corner. The difference is noise, not signal.

This also matches the previous rig's ~0.05 px static centroid noise, measured
independently on different hardware — two unrelated measurements landing on the
same figure is reassuring about both.

**Why it matters:** gain can be used freely to buy brightness. The usual worry
that gain amplifies sensor noise into corner uncertainty does not show up here.

The script that produced this, `corner_jitter.py`, is no longer in the repo,
so the measurement cannot currently be reproduced from here.

**Status after §7.** These sessions were taken with the old 9×6 board. Their
corners were very likely produced by the 4.6 C++ path, which mislabelled
every corner (§7); *which detector `corner_jitter.py` used is not recorded*.
Jitter is a repeatability metric: the spread of each corner's position across
frames of a static scene. A consistent mislabelling gives the same wrong ID to
the same physical point in every frame, so it does not invalidate the figure.
Re-measure it on the new board (§17) when its capture sessions start.

---

## 5. Coverage, not RMS, is what tells you the calibration is sound

**Verified by controlled experiment.**

This is the most important finding here.

On the previous rig a calibration reported a healthy 0.356 px RMS while 6 of 30
frame regions contained **zero** board observations. The consequence surfaced
much later: a blob near the left frame edge moved 3.9 px under undistortion
while centre-frame points moved 0.02 px. The eight rational-model coefficients
were extrapolating at the edges rather than being measured there.

To confirm the mechanism, two synthetic capture sets were rendered against
known ground truth (fx = fy = 780, cx = 632, cy = 518, a real radial distortion
field applied per pixel), then solved with `solve_intrinsics.py` (since
removed; `python/fit_intrinsics.py` is its replacement):

| Set | Empty cells (5×6) | RMS | fx recovered | Undistortion error vs truth, outer 20% of field |
|-----|------------------:|----:|-------------:|------------------------------------------------:|
| centre-clustered | 18 of 30 | **0.262 px** | 781.3 | **55.96 px** median |
| edge-covered | 0 of 30 | **0.223 px** | 780.0 | **26.82 px** median |

Read that table twice. **RMS does not distinguish the two sets** — the badly
sampled one even looks comparable, and its focal length is recovered to 0.2%
with fx and fy agreeing to 0.013%. Every headline number looks healthy. Yet
undistortion accuracy across the whole frame differs by 10× (1.563 px vs
0.147 px median), and at the frame edge by 2×.

The reason is structural: reprojection error is only computed where
observations exist. A region with no data contributes nothing to RMS, so RMS is
constitutionally incapable of reporting the gap.

**Why it matters:** coverage has to be watched *during* capture. Discovered
afterwards, it means recapturing. This is why `src/04_capture` draws the
coverage grid live and turns its status line green only at zero empty cells
(`lib/coverage.h`, `lib/coverage.cpp`). The Python solve does not re-check
coverage.

**The same lesson applies to detection: a full corner count is not an
acceptance criterion either.** On the old board the 4.6 C++ path reported
40/40 corners on every frame while every corner carried the wrong ID (§7). A
board with rows and cols swapped behaves the same way. Rendered synthetically,
it gave 54/54 markers, 88/88 corners and every ID in range, but a 249 px
residual. A count says how many corners were found, not whether they are the
right ones.

What does tell them apart is the **fit residual**: the worst distance between
the detected corners and a homography fitted from their board coordinates
(`Detector::layoutResidualPx()` in `lib/detect.cpp`, shown as `fit` by
`03_detect`). Correct labels give about 1 px (0.7–1.5 px measured, §7 and
§17); wrong labels give hundreds. Lens distortion raises it somewhat for a
board near the frame edge.

---

## 6. Pose variety separates focal length from distance

**Verified, in the negative.**

Two 20-frame sessions were captured for the gain comparison with the board on a
tripod, deliberately static. Tilt across all 20 frames of each: min 14.5°, max
14.6°, standard deviation **0.02°**. The board centre projected to the same
pixel in every frame.

Solving those sessions as if they were calibration data returns nonsense:
fx = 154, cx = 736 in one, fx = 554 with fx and fy 4.4% apart in the other —
against an expected fx of ~780. RMS was 0.08–0.09 px, *better* than a good
calibration, because 20 near-identical views are trivially easy to fit.

**Why it matters:** all-fronto-parallel views leave focal length and distance
mutually ambiguous — a small near board and a large far one produce the same
image. The tell is fx drifting away from fy. `src/04_capture` estimates board
tilt per capture via `solvePnP` (assuming f = 780 px, principal point at the
image centre) and displays the share of captures beyond 20° plus a tilt
histogram. It shows the number; it does not warn below 30%.

---

## 7. Board layout: OpenCV 4.6 does not build the legacy layout

**Verified, 2026-10-27.** This corrects an earlier version of this section,
which said 4.6's `CharucoBoard` already uses the legacy layout.

OpenCV ≥ 4.7 has `setLegacyPattern()`. The flag moves the markers onto the
other colour of square, and it only does so for boards with an **even** row
count. With an odd row count, legacy and non-legacy are the same layout.

**OpenCV 4.6's `CharucoBoard::create()` builds the NON-legacy layout.** This
was checked by dumping marker object points and IDs from 4.6 `create()` and
from 5.0 with each flag value. For the old 9×6 board, 4.6 equals 5.0
non-legacy and differs from 5.0 legacy: marker 0 sits on square (1, 0)
instead of (0, 0).

**The old 9×6 board has the legacy layout.** It was generated with calib.io's
"ChArUco Legacy" checkbox **unchecked**, yet it needs the legacy layout. On
frames from `intrinsics_33661_20260925_103705`:

- 5.0 with `setLegacyPattern(True)` finds 39–40 corners per frame, with a
  worst homography residual of 1.97 px over 15 frames.
- 5.0 without it finds no usable view. `detectMarkers` still finds the
  markers, so the failure looks like a detection problem rather than a layout
  mismatch.

**So the 4.6 C++ path mislabelled every old-board corner.**
`interpolateCornersCharuco` returned a full 40/40 corners, with 27/27 markers,
on every frame tested. But the IDs were attached to the wrong corners:

- A homography fitted from board coordinates to the detected corners left a
  worst residual of 200–234 px per frame.
- The same corner ID landed a median 70 px (max 248 px) from where 5.0
  legacy put it.
- Positionally, most corners still sat near a real checker corner (median
  0.7 px, worst 45 px). Their labels were wrong.

The earlier "confirmation" of this section was that 4.6 returned 40.0 of 40
corners on average. That was a count, and the count was right while the
labels were not (see the lesson at the end of §5).

**Consequences for the old-board sessions:**

- **Intrinsics are unaffected.** The Python solve re-detects every frame with
  OpenCV 5.x and `setLegacyPattern(True)`, so it never used the C++ labels
  (§14 stands).
- **Live tilt and distance feedback during those captures was unreliable.**
  `04_capture` estimated pose with `solvePnP` on the mislabelled corners, so
  the tilt, distance, tilted share and histogram it showed (§6) did not
  describe the board. The coverage tally counts corner positions, not IDs, so
  it was largely unaffected (*inferred*).

**Current code.** `lib/detect.cpp` (`applyLayout`) no longer trusts
`create()`'s layout. It places the markers by the 4.7+ rule for the
configured `legacy_pattern` and recomputes 4.6's nearest-marker tables:

- Its layout equals 5.0's for 9×6 and 12×9 with both flag values.
- Where the layout is unchanged, its tables equal `create()`'s.
- On the old frames above it gives 40/40 corners with a residual of
  0.98–1.52 px.

The board, including `legacy_pattern`, is defined in
`config/charuco_board.yaml`. The old board is kept in
`config/charuco_board_old.yaml` with `legacy_pattern: true`. `session.json`
records the full board (`lib/session.cpp`). `python/board.py` uses that
record, or the old-board file for sessions recorded before the board config
existed, after checking that it matches what the session recorded.

## 8. Device discovery is staggered and can duplicate

**Verified.**

`WaitForNewDevice()`-style waits return as soon as the *first* device
initialises, well under a second, while a cold camera network takes 4–8 seconds
to enumerate fully. Anything that trusts the first result sees a fraction of
the cameras.

`CameraList` can also return **the same serial more than once** — observed
consistently on 33661, which appears twice on every enumeration.

The removed tool polled the device list until the count had been stable for
1.5 s. **That window is too short for ten cameras**: tuned for four (6 stable
polls of 250 ms), it reported only 5 of 10 ready, because the count plateaus
while later cameras are still initialising (`lib/camera.cpp`, comment in
`discover()`). The current `discover()` polls every 250 ms and needs 20 stable
polls (5 s of no change), capped at 120 polls (30 s). `src/01_discover` uses
the same window.

*Current code:* `src/02_stream` and `src/03_detect` still carry the old
window (6 stable polls, 40-poll cap). Duplicates are collapsed by serial, but
`discover()` keeps the **first** entry for a serial and `01_discover` keeps
the **last**; neither prefers the `Initialized` one.

---

## 9. Other SDK behaviour worth knowing

**Verified** against the removed tool. The first two are handled in
`lib/camera.cpp` (and inline in `src/02_stream`, `src/03_detect`).

- `SetVideoType` is asynchronous. `VideoType()` reports the new mode
  immediately, but frames already queued still carry the old one, and
  `GrayscaleData()` on an object-mode frame returns null. In practice exactly
  one object-mode frame arrives after `Start()` and must be dropped.
  `Camera::start()` sets the type, calls `Start()`, then sets it again, and
  every fetched frame's `FrameType()` is checked.
- `NextFrame()` returns the **oldest** queued frame, as a
  `std::shared_ptr<const Frame>`. One call per loop iteration falls further
  behind on every pass; the queue has to be drained to the end each time.
  Dropping the `shared_ptr` frees the buffer; there is no manual `Release()`.
- SDK listener callbacks fire on its own discovery thread. Calling back into
  the SDK from inside one can deadlock, so callbacks only touch local state.
  (No current code registers a listener.)
- Starting a second instance immediately after killing the first occasionally
  hangs in `CameraLibraryStartup()` while the previous process's sockets are
  still bound. A couple of seconds between runs avoids it.

---

## 10. Working distance for intrinsics is 1–1.5 m

**Verified by observation, not yet by a systematic sweep.**

At 2–3 m the board fills too little of the frame, marker count begins
fluctuating rather than sitting at 27/27, and detected corners cluster
mid-frame — which is exactly the narrow-data failure described in section 5.
At 1–1.5 m the board reaches the frame edges and detection is stable.

This was observed with the old 9×6 board (67 mm markers). The 12×9 board
(§17) has 45 mm markers on a smaller pattern. Its distance limit is not yet
measured, and is *inferred* to be shorter.

---

## 11. Object mode is what ten cameras run on

**Partly verified.** Grayscale is ~1.3 MB per frame and one camera alone
delivers 6–8 fps (section 3), so ten grayscale streams are not an operating
mode. In object mode the camera thresholds on board and sends only blob
centroids, each with the camera's own area and roundness (`lib/camera.h`).
That is what `src/05_identify` and `src/07_mask` run all ten cameras on.

Object mode is described in the code comments as hardware-synchronised across
cameras. **Unverified here:** `Camera::diagnostics()` prints the SDK's
`IsVideoTypeSynchronous(ObjectMode)` answer, but no output is recorded, and no
synchronised capture has been built (see Open items).

**`SendEmptyFrames(true)` is required for diagnosis.** Without it the camera
sends nothing when it sees no markers, so "no frames" and "no blobs" are the
same observation and a dark scene looks like a dead stream. Every object-mode
tool turns it on. Source: code comment in `lib/camera.h`; no measurement
recorded.

**The object-mode operating point is not settled.** Two sets of values are
in use, and nothing in the repo measures one against the other:

| Source | Exposure (µs) | Threshold | Gain | Intensity | Rate |
|--------|--------------:|----------:|-----:|----------:|-----:|
| `src/05_identify`, `src/07_objdiag` defaults | 500 | 200 | 0 | 15 | 30 Hz |
| `config/mask.yaml` (masks learned here) | 10 | 40 | 0 | 15 | 30 Hz |

10 µs is the SDK's exposure floor (section 1). The `05_identify` source notes
that the floor "is not automatically a working value".

The camera reports its own threshold limits (`MinimumThreshold()` /
`MaximumThreshold()`). The code comments say a value outside them is silently
clamped or ignored and reads downstream as "no markers"; this is unverified.
`src/07_objdiag --sweep` measures the peak blob count across that range.

---

## 12. Change one setting at a time while streaming

**Observed, not measured** (code comments in `lib/camera.cpp`).

- Re-sending every setting mid-stream costs a settling period per command,
  which looks like a stall in frame delivery. `Camera::apply()` sends only
  the fields that changed since the last call; `setThreshold()` and
  `setExposure()` send one value. The vendor's own sample app also sets one
  value per user action.
- Settings applied while the camera is stopped are not reliably retained, so
  `Camera::start()` applies them after `Start()`.
- Reasserting the video type when it has not changed costs a mode-change
  settling period, so `apply()` only sends it on a real change.
- Polling `NextFrame()` in a tight loop can starve the SDK's own receive
  thread and itself look like 0 fps; `src/07_objdiag` sleeps 1 ms between
  empty polls.

---

## 13. Blocking masks

**SDK semantics: read from the SDK binary** (`cBlockControlBase` /
`cBlockControlMask` in `libCameraLibrary.so`), not from documentation.
`mask --probe` exists to confirm the behavioural ones on hardware; **no probe
output is recorded**, so those remain unverified except where noted.

- The mask is a grid of cells, not pixels. `BlockingGrid()` is the cell size
  in pixels; `BlockingMaskWidth/Height()` are the cell counts,
  `ceil(1280 / grid)` × `ceil(1024 / grid)`. Pixel (x, y) is in cell
  (x / grid, y / grid). **Verified:** all ten cameras report **640 × 512
  cells of 2 px** (`grid` in every `config/masks/<serial>.json`).
- `SetBitMaskPixel(X, Y)` takes **cell** coordinates despite its name. A
  pixel coordinate is bounds-checked only against the cell grid, so it masks
  the wrong cell or is silently ignored. No current code calls it.
- `AddBlockingRectangle` takes **pixel** coordinates and divides by the grid.
  No current code calls it.
- `SetBlockingMask` / `GetBlockingMask` take one byte per cell, row-major,
  and the size must be exactly cols × rows. `SetBlockingMask` with any other
  size **clears** the staged mask; `GetBlockingMask` does nothing.
  `Camera::stageMask()` checks the size before calling.
- Every edit goes into an SDK-side staging copy. Nothing reaches the camera
  until `UpdateBlockingMask()`, which sends only the chunks that differ from
  what the SDK believes the camera holds. That belief starts **empty** in
  every new process, so a mask the camera kept from an earlier run is
  invisible, and an incremental update on top of it leaves stale cells.
  `Camera::setMask()` therefore commits all-blocked and then the target,
  which forces every chunk to be sent whatever the camera held.
- Whether `ClearBlockingMask()` by itself reaches the camera is unverified.
- `IsBlockingMaskEnabled()` reports an SDK-side flag, not a camera readback.

**Learn and verify results, 2026-10-17 15:37** (from
`config/masks/*.json`; all ten cameras at the `mask.yaml` settings in
section 11):

| | Range over the ten cameras |
|---|---|
| Persistent sources found in a 30 s empty-volume window | 2 – 7 |
| Cells masked | 424 – 1177 (0.13% – 0.36% of the frame) |
| Verify window | 300 frames per camera |
| Residual persistent sources after masking | **0 on all ten** |
| Blobs per frame after masking | 0 on nine; 0.12 on 33276 |

So learn → apply → commit removes the static interference at these settings.

**Masks are settings-specific.** Exposure and threshold decide which glare
becomes a blob, so each saved mask records the settings it was learned
under and `mask --load` warns if any differ. A mask is also valid only while
its camera has not moved.

**The acceptance test has no recorded pass.** `mask --coverage` passes only
if the walked marker is never seen by fewer than `min_observers` (3)
cameras. Four walk logs are in `logs/`; in each the minimum is 0 observers,
with 17% to 100% of samples below 3. The logs don't record whether the
zero-observer samples are the marker outside the volume or a real gap.

---

## 14. Intrinsics of camera 33661

**Verified** by `python/fit_intrinsics.py` (OpenCV 5.0.0, Brown-Conrady,
all frames) on three sessions. The sessions are in `$HOME/panoptes-data/sessions`,
which is outside the repo:

| Session | Views | RMS (px) | fx | fy | cx | cy |
|---|---:|---:|---:|---:|---:|---:|
| `intrinsics_33661_20260924_155048` | 67 | 0.383 | 779.69 | 780.88 | **636.48** | 514.57 |
| `intrinsics_33661_20260925_094611` | 50 | 0.359 | 781.94 | 781.92 | **645.88** | 511.82 |
| `intrinsics_33661_20260925_103705` | 50 | 0.372 | 782.17 | 781.29 | **643.37** | 510.31 |
| Range | | | 2.48 | 1.05 | **9.40** | 4.26 |

cx is the outlier: 9.4 px across sessions against 2.5 px for fx. The
`bootstrap_intrinsics.py` docstring explains why cx and cy are the
weakest-determined parameters. Whether resampling one session reproduces this
spread has not been recorded (see Open items).

`config/layout.yaml` uses fx 782.0, fy 781.6, cx 644.6, cy 511.1. These match
the mean of the two 2026-09-25 sessions to within 0.1 px. That derivation is
inferred; `layout.yaml` says only "measured on 33661". Its distortion
coefficients are **assumed placeholders**, not these fits. For example,
`layout.yaml` k2 is 0.02, while the three fits give 0.082–0.099.

**Model choice.** `fit_intrinsics.py` and `layout.yaml` record Brown-Conrady
(5 coefficients), chosen over the rational model (8) because held-out testing
"showed the rational model's three extra coefficients bought nothing at
82 deg". `compare_models.py` prints held-out error and edge straightness but
saves only training RMS. On the 2026-09-24 session that file shows
Brown-Conrady 0.3756 px, rational 0.3746 px, and fisheye 181 px (a failed
fit). **The held-out numbers behind the choice are not recorded**, so the
reason is unverified from repo data.

---

## 15. Synthetic triangulation accuracy, nominal layout

**Verified as a computation; it is a floor, not a forecast.** Run 2026-10-17
by `python/accuracy_study.py`. It assumes perfect calibration, static centroid
noise of 0.054 px, shared 33661 intrinsics with placeholder distortion, and
geometric visibility only. Full tables, method and assumptions:
[`analysis/README.md`](../analysis/README.md); numbers and provenance:
`analysis/accuracy_study.json`.

| | Median | p90 | Max |
|---|---:|---:|---:|
| Whole volume, RMS 3D error | 0.098 mm | 0.193 mm | 0.82 mm |
| Perch box (perch ± 12 in), RMS 3D error | 0.077 mm | 0.089 mm | 0.102 mm |

- **19% of the volume cannot be triangulated at all** (80.7% is seen by 2+
  cameras), and 24.3% is seen by fewer than 3. The gap is the top of the room,
  above about 90 in. Every camera aims at z = 30 in, so that region falls
  outside every frustum. Coverage, not accuracy, is the main volume finding.
- The perch box is fully covered: 96% of its points are seen by all ten, none
  by fewer than eight.
- The two camera heights balance the axes. Upper six alone: z is weak (0.105 mm
  median on the perch box). Lower four alone: y is weak (0.075 mm). All ten:
  0.043 / 0.047 / 0.043 mm.
- Any six cameras are worse, up to about 1.8× on the perch box. The worst
  sixes keep only one lower camera.
- 20 perturbed layouts (±2 in position, 2° aim, ±2° roll) move every headline
  statistic by at most 9.3%. Tape-measured positions are good enough for these
  conclusions.
- For scale, the previous rig measured 13.8 mm loop closure over a 9 m
  circuit. The geometry and centroid noise use very little of that budget.

---

## 16. Build and host environment

**Verified on this host** except where noted.

- Every binary that talks to the cameras needs
  `cap_net_raw,cap_net_admin+eip`. Linking writes a new file, which **drops
  file capabilities**, so `setcap` must be rerun after every relink. (A fresh
  build on 2026-10-19 showed no capabilities on any binary under `getcap`.)
- Running a tool under `sudo` instead breaks desktop audio for the session
  and leaves root-owned files behind. (From the previous RUNBOOK; not
  re-tested.)
- The C++ tools link the system OpenCV **4.6** (`libopencv-dev`). The Python
  scripts use OpenCV **5.x** from `opencv-contrib-python` in `.venv`. The
  split is deliberate. OpenCV 5 removed the `aruco` calibration path that 4.6
  uses and added `CharucoDetector` / `matchImagePoints`, and 5.x cannot be
  loaded into a process already linked to 4.6. The venv also keeps the numpy
  2.x that the wheel needs away from apt's `python3-matplotlib` and
  `python3-scipy`, which are built against numpy 1.x.
- Ubuntu 24.04 marks the system Python as externally managed (PEP 668), so
  `pip install` outside a venv is refused. `python3.12-venv` is not installed
  by default.

---

## 17. The calib.io 12×9 board

**Verified on hardware, 2026-10-27, camera 33664.**

This board replaced the old 9×6 board for intrinsics. Its definition and
provenance are in `config/charuco_board.yaml`.

| | |
|---|---|
| Vendor | calib.io, on an 800 × 600 mm aluminium plate |
| Pattern | 9 rows × 12 cols of 60 mm squares (nominal), 720 × 540 mm |
| Markers | 45 mm (nominal), `DICT_5X5_100`; 54 markers, IDs 0–53 |
| Corners | 11 × 8 = 88 interior corners |
| `legacy_pattern` | `false`. With 9 rows it makes no difference (§7). |

The measured sizes are the caliper values in the YAML. Nominal sizes are
never used in their place.

**Hardware detection test.** `03_detect` on camera 33664 gave 54/54
markers, 88/88 corners, max ID 53 and a fit residual (§5) of 0.7–0.8 px. The
result was identical with `--legacy` and `--no-legacy`. That matches the
offline check: with an odd row count, both flag values give the same marker
layout in `lib/detect` and in 5.0. calib.io's statement that odd-row boards
are unaffected by the OpenCV layout change therefore holds on this rig.

**The 5×5 dictionaries are nested.** IDs 0–53 have identical bit patterns in
`DICT_5X5_100`, `_250` and `_1000`, checked in both OpenCV 4.6 and 5.0. A
board generated in any of them detects in the others. `_100` is used because
it has the largest error-correction margin: `maxCorrectionBits` 3, against 2
for `_250` and `_1000`. The old board's sessions record `DICT_5X5_1000`;
their IDs 0–26 are the same markers.

---

## Open items

1. **cx cross-session discrepancy.** 9.4 px across three sessions of the
   same camera (section 14). Running `bootstrap_intrinsics.py` on one session
   would show whether pose sampling explains it; that has not been recorded.
2. **WRONG MODE false alarm.** `05_identify` and `07_mask` print `WRONG MODE`
   when `modeIsObject()` is false. That flag describes the most recent
   *fetch*, and `fetch()` resets it when no frame was waiting. A camera
   streaming object frames normally, with nothing queued at the moment of the
   report, therefore prints `WRONG MODE`. This is inferred from
   `lib/camera.cpp` and has not been reproduced deliberately.
3. **Tools don't report mask state.** Only `07_mask` knows about masks, and
   it leaves them applied on exit. `05_identify`, `07_objdiag` and
   `04_capture` neither read nor clear them, and the "blocking mask" line in
   `diagnostics()` is the SDK flag, not the camera's state. A camera still
   holding a mask is blind in those cells, which looks like a detection bug.
   Whether masks survive a process restart or a power cycle is what
   `mask --probe` step 1 tests; no result is recorded.
4. **Mask tool's compile-time config path.** `MOCAP_CONFIG_DIR` is the
   absolute path of `config/` at configure time. Moving or renaming the repo
   breaks the default. Workaround: rebuild, or pass `--config DIR`.
5. **Frame sync** across cameras, via `cModuleSync` (the SDK header
   describes it as "utilized to synchronize frame data"), not `Synchronizer`.
   Not built; the choice is unverified on hardware.
6. **Marker labeling.** Not built.
7. **Extrinsics: wand and L-frame.** Not built. A wand works in object mode,
   which is synchronised; a board needs grayscale, which is not
   (`src/notes.md`).
8. **Reference markers.** Not built.
9. **Object-mode operating point** (section 11). Two settings sets are in
   use, and masks are valid only for the one they were learned under.
10. **Mask acceptance test has no recorded pass** (section 13).
11. **Inconsistencies found while documenting, 2026-10-19:**
    - `src/02_stream` and `src/03_detect` use the short discovery window
      that undercounts ten cameras (section 8).
    - `05_identify` hardcodes the 30 Hz exposure ceiling (section 1).
    - `session.json`'s `created` field is rewritten on every flush, so it
      records the last write. The session folder name is local time;
      `created` is UTC.
    - `config/cameras.yaml` and `docs/hardware/cameras.md` list six cameras,
      with 33275 "excluded, defocused". `config/layout.yaml` mounts ten,
      including 33275 as camera E, and all ten have masks. No code reads
      `cameras.yaml`.
    - `PyYAML` is in `python/requirements.txt`, but no script imports it;
      `rig.py` reads YAML through `cv2.FileStorage`.
    - `layout.yaml` distortion is assumed although fitted coefficients
      exist (section 14).

## Open questions

- **Does the 2–3 m detection limit matter for extrinsics?** It does not for
  intrinsics, where the board can simply be brought closer. Extrinsics needs a
  target visible to several cameras at once across the capture volume, which
  will put it further away than 1.5 m from at least some of them. Whether the
  ChArUco board is detectable at usable rates at those distances — or whether
  extrinsics needs a different target entirely — is untested.
- **Distortion coefficient interpretability.** (Written when the rational
  model was in use; the current fit is Brown-Conrady, section 14.)
  Rational-model coefficients come
  out large and correlated (k1≈k4, k2≈k5) because the model is a ratio of two
  polynomials. Recovered geometry is accurate regardless, but there is no
  established threshold on the coefficients themselves for spotting a bad fit.
  Comparing undistortion displacement fields between calibrations would be a
  more direct check, and is not yet built.
- **Whether 5×6 is the right coverage grid.** 30 cells over 88 corners per view
  on the current 12×9 board (40 on the old one)
  means a single well-placed capture can touch many cells at once. A finer grid
  would be stricter; nothing has been done to establish the right granularity.
