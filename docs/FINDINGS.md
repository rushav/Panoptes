# Findings

Everything measured on this rig, and the rules that follow from it. This file
is the single home for measured numbers and for open items; other docs cite it
by section (§N).

- How the code works: [ARCHITECTURE.md](ARCHITECTURE.md)
- Commands: [RUNBOOK.md](RUNBOOK.md)
- Camera inventory, serials, specs, network: [hardware/cameras.md](hardware/cameras.md)

**Marking.** **Verified** means a measurement is recorded here. *Inferred*
means read from code, a code comment or the SDK binary, with no measurement in
this repo.

**Record notes.**

- §3–§12 were first written against the GUI tool `src/intrinsics_capture`,
  removed in commit a91b3c6. The measurements stand; tool behaviour described
  in them has since changed (see ARCHITECTURE).
- 2026-10-19: the repo root is what used to be `mocap/` in GIX-TALON, renamed
  PANOPTES.
- 2026-10-28: sections renumbered. Topic section **old §N is now §N+2**
  (Key numbers and Rules were inserted as §1–§2). Comments in code and config
  that cite "FINDINGS N" use the old numbers.

---

## 1. Key numbers

- **Corner jitter (static):** 0.0543 px radial median at gain 7, 0.0600 px at gain 0; 2026-10-27, camera 33664, `intrinsics_33664_20261027_151529` / `_152034`, `python/corner_jitter.py`. §6
- **Gain:** free at the grayscale operating point (gain 7 better than gain 0 at median, p90 and max); same sessions. §6
- **Intrinsics repeatability, 33661:** range fx 2.48, fy 1.05, cx 9.40, cy 4.26 px over 3 sessions; same-day pair fx 0.23, fy 0.63, cx 2.51, cy 1.51 px; 2026-09-24/25, `python/fit_intrinsics.py`. §16
- **Distortion model:** Brown-Conrady (5); training RMS BC 0.3756 / rational 0.3746 / fisheye 181 px on `intrinsics_33661_20260924_155048`, `python/compare_models.py`; held-out numbers not recorded. §16
- **Coverage beats RMS:** centre-clustered set (18/30 empty) RMS 0.262 px vs edge-covered 0.223 px, yet edge undistortion error 55.96 vs 26.82 px; synthetic, `solve_intrinsics.py` (removed), date not recorded. §7
- **4.6 mislabelling:** old 9×6 board, 40/40 corners with homography residual 200–234 px; 2026-10-27, frames of `intrinsics_33661_20260925_103705`. §9
- **calib.io 12×9 board test:** 54/54 markers, 88/88 corners, fit 0.7–0.8 px, same with `--legacy` and `--no-legacy`; 2026-10-27, camera 33664, `03_detect`. §19
- **Mask learn/verify:** 0 residual persistent sources on all ten cameras, 424–1177 cells masked (0.13–0.36%); 2026-10-17 15:37, `config/masks/*.json`. §15
- **Mask coverage walk:** no recorded PASS, minimum 0 observers in all 4 logs; 2026-10-16/17, `logs/coverage_*.csv`. §15
- **Triangulation floor:** 0.077 mm median (perch box), 0.098 mm (whole volume), 19% of the volume untriangulable; 2026-10-17, `python/accuracy_study.py`, `analysis/accuracy_study.json`. §17
- **Exposure ceiling:** `floor(1e6/fps) − 200` µs (33133 at 30 Hz); camera 33661, date not recorded. §3
- **IR filter in:** 27/27 markers, 40/40 corners in every frame at ~1.5 m; 2026-08-24, `intrinsics_33661_20260824_234853` / `_235746`. §4
- **Grayscale delivery:** ~10.7 MB/s, 6–8 fps whatever the imager rate; camera and date not recorded. §5
- **Pose variety:** 20 static views at tilt std 0.02° solve to fx = 154 and fx = 554 against ~780, RMS 0.08–0.09 px; 2026-08-24 gain sessions. §8
- **Working distance:** 1–1.5 m for intrinsics (old 9×6 board), observation only. §12
- **Kortex API on Python 3.12:** wheel 2.6.0.post3 imports with protobuf 3.20.3 (`--no-deps`), not with its pinned 3.5.1 or with 4.25.8; 2026-10-28, `.venv`. Never yet connected to the arm. §20
- **IMU parameters:** not in this repo. No IMU document, config or measurement exists to link.

---

## 2. Rules we learned

These must not be re-derived. **This list is identical to the one in
[CLAUDE.md](../CLAUDE.md#verified-sdk-and-hardware-behaviour-do-not-fix-these-away);
change both together.**

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
20. **The Kortex wheel pins `protobuf==3.5.1`, which cannot import on
    Python 3.12.** Install `python/requirements-arm.txt` with `--no-deps`
    (protobuf 3.20.3). protobuf 4.x refuses the wheel's generated code.
    `pip check` reporting the pin mismatch is expected. (§20)

---

## 3. Exposure ceiling is set by the frame period

**Verified**, camera 33661, setting each rate and reading the range back. It
corrects an earlier assumption.

| Frame rate | SDK exposure range | `floor(1e6/fps) − 200` |
|-----------:|-------------------:|-----------------------:|
| 240 Hz | 10 – 3966 | 3966 |
| 120 Hz | 10 – 8133 | 8133 |
| 60 Hz | 10 – 16466 | 16466 |
| 30 Hz | 10 – 33133 | 33133 |

- The SDK re-reports `MinimumExposureValue()` / `MaximumExposureValue()` after
  every `SetFrameRate()`. Exposure set to the reported maximum reads back at
  that maximum at every rate. Units are µs; the 200 µs is readout overhead.
- Two earlier beliefs were wrong, in opposite directions:
  - "Exposure above ~6000 is clamped at 120 Hz." The 120 Hz ceiling is 8133.
  - "The ceiling is a hard 8133 at any rate, tested at 120 and 30 Hz." That
    was the removed tool reading the range once at startup, so its slider
    stayed at the 120 Hz bound after switching to 30 Hz. The fixed tool then
    logged `exposure range at 30 Hz: 10..33133`.
- At 30 Hz there is 4× more headroom than the 8133 µs default uses, so
  exposure is not the constraint on a dark board.
- How the code handles it: [ARCHITECTURE §2](ARCHITECTURE.md#2-two-operating-modes).
  `src/05_identify` still caps its `]` key at 33133 (Open items).

## 4. The IR bandpass filter can stay IN for ChArUco capture

**Verified**, camera 33661, 2026-08-24.

| Setting | Value |
|---------|-------|
| IR filter | IN (850 nm bandpass) |
| IR intensity | 15 |
| Imager gain | 7 |
| Exposure | 8133 µs |
| Frame rate | 30 Hz |

- Two 20-frame sessions at ~1.5 m (`intrinsics_33661_20260824_234853` and
  `..._235746`) gave **27/27 markers and 40/40 corners in every frame**.
- Calibration then uses the same optical configuration as tracking, so there
  is no filter-removal focal shift to characterise.
- With the filter in but the ring off at 400 µs, the frame averages 6/255 and
  detects zero markers. The filter is only workable with the ring.

## 5. Grayscale delivery is bandwidth-bound

**Verified**, by counting frames drained from the SDK queue over 3 s windows
at 120 Hz:

```
decim=1  received 14.7 fps  1280x1024 | FrameRate()=120 ActualFrameRate()=120  DataRate=10.7 MB/s
decim=2  received  9.7 fps  1280x1024 | FrameRate()=120 ActualFrameRate()=120  DataRate=10.8 MB/s
decim=4  received 10.0 fps  1280x1024 | FrameRate()=120 ActualFrameRate()=120  DataRate=10.7 MB/s
```

- `DataRate()` sits at ~10.7 MB/s, about 8 frames of 1.31 MB. The text of
  this finding has always given ~8 fps at both 120 and 30 Hz, and 6–8 fps in
  the live view.
- **Inconsistent:** the three logged lines show 9.7–14.7 fps received, above
  that. The record doesn't say where the 6–8 fps figure was measured
  (Open items).
- `ActualFrameRate()` confirms the imager runs at the configured rate; only
  delivery is limited. The removed GUI showed the two numbers separately; no
  current tool does.
- `SetGrayscaleDecimation()` has no effect: `GrayscaleDecimation()` stays 0
  and the frame size never changes.
- Harmless for calibration with the board held still; it would matter for
  anything wanting fast grayscale.

## 6. Corner jitter, and why gain is free

**Verified, 2026-10-27**, camera 33664, calib.io 12×9 board (§19).

**Method.** Board and camera fixed; per-corner std of detected position across
frames, by `python/corner_jitter.py` (RUNBOOK §7). Radial std is
`sqrt(std_x² + std_y²)`; per-axis is radial / √2. Each session: 200 frames at
1–1.5 m, board static, centred and tilted; 8133 µs, filter in, intensity 15,
30 Hz.

| Gain | Session | Frames kept | Fit, median | Radial median | p90 | Max | Per-axis median |
|-----:|---------|------------:|------------:|--------------:|----:|----:|----------------:|
| 7 | `intrinsics_33664_20261027_151529` | 200/200 | 0.70 px | **0.0543 px** | 0.0705 | 0.0879 | 0.0384 |
| 0 | `intrinsics_33664_20261027_152034` | 199/200 (1 rejected for corner count) | 0.70 px | **0.0600 px** | 0.0820 | 0.1052 | 0.0424 |

- All 88 corners were present in every kept frame. Per-corner results:
  [analysis/jitter/intrinsics_33664_20261027_151529_corner_jitter.json](../analysis/jitter/intrinsics_33664_20261027_151529_corner_jitter.json),
  [analysis/jitter/intrinsics_33664_20261027_152034_corner_jitter.json](../analysis/jitter/intrinsics_33664_20261027_152034_corner_jitter.json).
  Both record commit `da80eec` with `git_dirty: true` (`corner_jitter.py` was
  not yet committed).
- **Gain is free.** Gain 7 beats gain 0 at the median, p90 and worst corner.
  The worry that gain amplifies noise into corner uncertainty doesn't show up.

**The original figure is reproduced, and was a radial median.** Original: two
20-frame sessions, old 9×6 board, camera 33661, 40 corners
(`intrinsics_33661_20260824_*`; sessions and script are gone):

| Gain | Median | Mean | Worst |
|-----:|-------:|-----:|------:|
| 0 | 0.0561 px | 0.0568 px | 0.0820 px |
| 7 | 0.0540 px | 0.0559 px | 0.0912 px |

- The new gain 7 radial median agrees to 0.6%; the per-axis median (0.0384)
  does not. So the old figure was radial (*inferred*; the old definition is
  not recorded). Gain 0 agrees less closely: 0.0600 vs 0.0561, 7% higher.
- Camera, board, frame count (200 vs 20) and pose all differ; agreement
  within 10% across that is the reproduction.
- The old corners were likely mislabelled by the 4.6 path (§9). That doesn't
  affect repeatability, since a consistent mislabelling gives the same wrong
  ID to the same point in every frame. The new sessions used OpenCV 5.x with
  correct labels (fit 0.70 px).
- Matches the previous rig's ~0.05 px static centroid noise, measured
  independently on different hardware.

**Edge figure unmeasured.** All 88 corners fell in the centre region of the
5×6 grid in both sessions. These are centre-of-frame numbers only.

**The accuracy study is conservative by about √2.** `config/study.yaml`
`noise_px: 0.054` is labelled "1-sigma per image axis", and
`accuracy_study.py` draws it independently on x and y
(`rng.normal(0, sigma, (k, trials, C, 2))`, and `predicted_cov()` uses the
same sigma). But 0.054 is the radial median; per-axis is 0.0384. So §17's
error figures are pessimistic by ~1.41× for centre-of-frame corners, if error
scales linearly with noise (*inferred*; not re-run at 0.0384).

**Consequence for auto capture's still gate** (operator report, 2026-10-28;
no session recorded). With the board static, `04_capture --auto` showed HOLD
STILL intermittently. Gate 2 then tested the largest of 88 frame-to-frame
corner moves against 0.3 px. A static corner moves ~√2 × 0.054 ≈ 0.08 px
between frames (*inferred* from the jitter above, not measured as a
frame difference), so a max over 88 corners trips on the tail. The gate
now tests the median against `still_px` (0.3) and the max against
`still_max_px` (1.5); mechanism in
[ARCHITECTURE §5](ARCHITECTURE.md#5-automatic-frame-selection). The new
gate is checked on synthetic corners only (`tests/test_autocapture`), not
yet on the camera.

## 7. Coverage, not RMS, says a calibration is sound

**Verified by controlled experiment.** The most important finding here.

**Previous rig.** A calibration reported 0.356 px RMS while 6 of 30 frame
regions had **zero** observations. Later, a blob near the left edge moved
3.9 px under undistortion while centre points moved 0.02 px. The eight
rational coefficients were extrapolating at the edges.

**Synthetic test.** Two capture sets rendered against known truth (fx = fy =
780, cx = 632, cy = 518, a real radial distortion field per pixel), solved
with `solve_intrinsics.py` (removed; `python/fit_intrinsics.py` replaces it):

| Set | Empty cells (5×6) | RMS | fx recovered | Undistortion error vs truth, outer 20% of field |
|-----|------------------:|----:|-------------:|------------------------------------------------:|
| centre-clustered | 18 of 30 | **0.262 px** | 781.3 | **55.96 px** median |
| edge-covered | 0 of 30 | **0.223 px** | 780.0 | **26.82 px** median |

- **RMS does not separate them.** The bad set recovers fx to 0.2%, with fx and
  fy agreeing to 0.013%. Yet undistortion over the whole frame is 10× worse
  (1.563 vs 0.147 px median) and 2× worse at the edge.
- Reprojection error exists only where observations exist, so RMS cannot
  report a gap. Coverage has to be watched during capture. The Python solve
  doesn't re-check it.

**A full corner count is not an acceptance criterion either.** The 4.6 path
gave 40/40 with every ID wrong (§9). A board with rows and cols swapped,
rendered synthetically, gave 54/54 markers, 88/88 corners, every ID in range,
and a **249 px** fit residual. Correct labels give about 1 px (0.7–1.5 px
measured, §9, §19); wrong labels give hundreds; distortion raises it somewhat
near the frame edge. How the residual is computed:
[ARCHITECTURE §4](ARCHITECTURE.md#4-intrinsics-capture).

## 8. Pose variety separates focal length from distance

**Verified, in the negative**, on the two 2026-08-24 gain sessions (§6).

- Board on a tripod, deliberately static: tilt min 14.5°, max 14.6°, std
  **0.02°** over 20 frames; board centre on the same pixel every frame.
- Solved as calibration data: fx = 154, cx = 736 in one; fx = 554 with fx and
  fy 4.4% apart in the other; expected ~780. RMS 0.08–0.09 px, *better* than a
  good calibration.
- All-fronto-parallel views make focal length and distance ambiguous (a small
  near board looks like a large far one). The tell is fx drifting from fy.

## 9. Board layout: OpenCV 4.6 does not build the legacy layout

**Verified, 2026-10-27.** Corrects an earlier version that said 4.6 already
uses the legacy layout.

- OpenCV ≥ 4.7 `setLegacyPattern()` moves markers to the other colour of
  square, only for an **even** row count.
- **4.6 `CharucoBoard::create()` builds the NON-legacy layout.** Checked by
  dumping marker object points and IDs from 4.6 and from 5.0 with each flag.
  For the old 9×6 board, 4.6 equals 5.0 non-legacy: marker 0 on square (1, 0)
  instead of (0, 0).
- **The old 9×6 board is legacy**, although generated with calib.io's
  "ChArUco Legacy" box **unchecked**. On frames of
  `intrinsics_33661_20260925_103705`:
  - 5.0 with `setLegacyPattern(True)`: 39–40 corners per frame, worst
    homography residual 1.97 px over 15 frames.
  - 5.0 without it: no usable view, though `detectMarkers` still finds the
    markers, so it looks like a detection problem.
- **So the 4.6 C++ path mislabelled every old-board corner.** It returned
  40/40 corners and 27/27 markers on every frame tested, but:
  - homography residual 200–234 px per frame
  - the same ID landed a median 70 px (max 248 px) from 5.0 legacy's position
  - positionally most corners sat near a real checker corner (median 0.7 px,
    worst 45 px); the labels were wrong
  - the earlier "confirmation" was a 40.0/40 average count
- **Consequences:** intrinsics are unaffected (the Python solve re-detects
  with 5.x and the legacy flag; §16 stands). Live tilt and distance in
  `04_capture` during those captures were unreliable (`solvePnP` on wrong
  labels); the coverage tally counts positions, so it was largely unaffected
  (*inferred*).
- **The fix** (`applyLayout`, [ARCHITECTURE §4](ARCHITECTURE.md#4-intrinsics-capture)):
  its layout equals 5.0's for 9×6 and 12×9 with both flag values; where the
  layout is unchanged its tables equal `create()`'s; on the old frames above
  it gives 40/40 with residual 0.98–1.52 px.

## 10. Device discovery is staggered and can duplicate

**Verified.**

- `WaitForNewDevice()`-style waits return when the *first* device initialises,
  well under a second. A cold ten-camera network takes 4–8 s to enumerate.
- `CameraList` can return **the same serial twice**: 33661 appears twice on
  every enumeration.
- A 1.5 s stable window (6 polls of 250 ms, tuned for four cameras) reported
  **5 of 10**: the count plateaus while later cameras initialise. The current
  window is 20 stable polls (5 s), 30 s cap
  ([ARCHITECTURE §1](ARCHITECTURE.md#1-discovery-and-the-sdk-frame-model)).
- `src/02_stream` and `src/03_detect` still use the short window (Open items).

## 11. Other SDK behaviour

**Verified** against the removed tool.

- `SetVideoType` is asynchronous. `VideoType()` reports the new mode at once,
  but queued frames keep the old one, and `GrayscaleData()` on an object
  frame is null. In practice exactly one object-mode frame arrives after
  `Start()`.
- `NextFrame()` returns the **oldest** queued frame as
  `std::shared_ptr<const Frame>`. One call per loop falls further behind each
  pass. Dropping the pointer frees the buffer; there is no `Release()`.
- SDK listener callbacks fire on its discovery thread; calling back into the
  SDK from one can deadlock. (No current code registers a listener.)
- Starting an instance right after killing one occasionally hangs in
  `CameraLibraryStartup()` while the old sockets are bound. A couple of
  seconds between runs avoids it.

## 12. Working distance for intrinsics is 1–1.5 m

**Verified by observation**, not a systematic sweep, on the old 9×6 board
(67 mm markers).

- At 2–3 m the board fills too little of the frame, the marker count
  fluctuates below 27/27, and corners cluster mid-frame (the §7 failure).
- At 1–1.5 m the board reaches the frame edges and detection is stable.
- The 12×9 board (§19) has 45 mm markers on a smaller pattern; its limit is
  unmeasured and *inferred* to be shorter.

## 13. Object mode is what ten cameras run on

**Partly verified.**

- Grayscale is ~1.3 MB/frame and one camera delivers 6–8 fps (§5), so ten
  grayscale streams are not an operating mode. `05_identify` and `07_mask` run
  all ten in object mode.
- **Hardware sync is unverified here.** `Camera::diagnostics()` prints
  `IsVideoTypeSynchronous(ObjectMode)`, but no output is recorded and no
  synchronised capture exists.
- **`SendEmptyFrames(true)` is needed for diagnosis.** Without it, no markers
  means no frames, so a dark scene looks like a dead stream. Source: code
  comment in `lib/camera.h`; no measurement.
- **The operating point is not settled.** Nothing measures one set against
  the other:

| Source | Exposure (µs) | Threshold | Gain | Intensity | Rate |
|--------|--------------:|----------:|-----:|----------:|-----:|
| `src/05_identify`, `src/07_objdiag` defaults | 500 | 200 | 0 | 15 | 30 Hz |
| `config/mask.yaml` (masks learned here) | 10 | 40 | 0 | 15 | 30 Hz |

- 10 µs is the exposure floor (§3); the `05_identify` source notes the floor
  "is not automatically a working value".
- The camera reports its own threshold limits. Code comments say a value
  outside them is silently clamped or ignored (*unverified*).
  `07_objdiag --sweep` measures blob count across that range.

## 14. Change one setting at a time while streaming

**Observed, not measured** (code comments in `lib/camera.cpp`).

- Re-sending every setting mid-stream costs a settling period per command,
  which looks like a stall. The vendor sample app also sets one value per
  action.
- Settings applied while the camera is stopped are not reliably retained.
- Reasserting an unchanged video type costs a mode-change settling period.
- A tight `NextFrame()` poll can starve the SDK's receive thread and look
  like 0 fps.
- How the code respects each: [ARCHITECTURE §1–§2](ARCHITECTURE.md#2-two-operating-modes).

## 15. Blocking masks

**SDK semantics read from the binary** (`cBlockControlBase` /
`cBlockControlMask` in `libCameraLibrary.so`), not documentation. `mask
--probe` exists to confirm them on hardware; **no probe output is recorded**,
so they are unverified except where noted.

- The mask is a grid of cells. `BlockingGrid()` is the cell size in px;
  `BlockingMaskWidth/Height()` are `ceil(1280 / grid)` × `ceil(1024 / grid)`;
  pixel (x, y) is in cell (x / grid, y / grid). **Verified:** all ten cameras
  report **640 × 512 cells of 2 px** (`grid` in every
  `config/masks/<serial>.json`).
- `SetBitMaskPixel(X, Y)` takes **cell** coordinates; a pixel coordinate is
  bounds-checked against the cell grid only, so it masks the wrong cell or is
  ignored. `AddBlockingRectangle` takes **pixels** and divides by the grid.
  No current code calls either.
- `SetBlockingMask` / `GetBlockingMask` take one byte per cell, row-major,
  exactly cols × rows. Any other size: `SetBlockingMask` **clears** the staged
  mask; `GetBlockingMask` does nothing.
- Edits go to an SDK-side staging copy. `UpdateBlockingMask()` sends only
  chunks differing from what the SDK believes the camera holds, and that
  belief starts **empty** in every process: a retained mask is invisible, and
  an incremental update leaves stale cells.
- Whether `ClearBlockingMask()` alone reaches the camera is unverified.
- `IsBlockingMaskEnabled()` is an SDK flag, not a camera readback.
- How `07_mask` handles all this: [ARCHITECTURE §3](ARCHITECTURE.md#3-blocking-masks).

**Learn and verify, 2026-10-17 15:37** (`config/masks/*.json`; all ten
cameras at the `mask.yaml` settings, §13):

| | Range over the ten cameras |
|---|---|
| Persistent sources found in a 30 s empty-volume window | 2 – 7 |
| Cells masked | 424 – 1177 (0.13% – 0.36% of the frame) |
| Verify window | 300 frames per camera |
| Residual persistent sources after masking | **0 on all ten** |
| Blobs per frame after masking | 0 on nine; 0.12 on 33276 |

- So learn → apply → commit removes static interference at these settings.
- **Masks are settings-specific** (exposure and threshold decide which glare
  becomes a blob) and valid only while the camera hasn't moved.
- **The acceptance walk has no recorded pass.** `mask --coverage` passes only
  if the marker is never seen by fewer than `min_observers` (3). Four logs in
  `logs/` (2026-10-16 ×3, 2026-10-17 ×1): each has a minimum of 0 observers,
  with 17%–100% of samples below 3. The logs don't say whether zero-observer
  samples are the marker outside the volume or a real gap.

## 16. Intrinsics of camera 33661

**Verified** by `python/fit_intrinsics.py` (OpenCV 5.0.0, Brown-Conrady, all
frames). Sessions are in `$HOME/panoptes-data/sessions`, outside the repo.

| Session | Views | RMS (px) | fx | fy | cx | cy |
|---|---:|---:|---:|---:|---:|---:|
| `intrinsics_33661_20260924_155048` | 67 | 0.383 | 779.69 | 780.88 | **636.48** | 514.57 |
| `intrinsics_33661_20260925_094611` | 50 | 0.359 | 781.94 | 781.92 | **645.88** | 511.82 |
| `intrinsics_33661_20260925_103705` | 50 | 0.372 | 782.17 | 781.29 | **643.37** | 510.31 |
| Range | | | 2.48 | 1.05 | **9.40** | 4.26 |

- **Repeatability.** The same-day 2026-09-25 pair differs by fx 0.23, fy 0.63,
  cx 2.51, cy 1.51 px. This is the basis of the auto-mode validation rule
  (RUNBOOK §9).
- **cx is the outlier**: 9.4 px across all three, against 2.5 px for fx. Why
  cx/cy are weakest-determined: [ARCHITECTURE §6](ARCHITECTURE.md#6-solve-and-the-distortion-model).
  Whether resampling one session reproduces the spread is not recorded.
- `config/layout.yaml` uses fx 782.0, fy 781.6, cx 644.6, cy 511.1, within
  0.1 px of the mean of the two 2026-09-25 sessions (*inferred* derivation;
  the file says only "measured on 33661"). Its distortion is an **assumed
  placeholder**: k2 0.02 against 0.082–0.099 in the three fits.
- **Model choice.** Brown-Conrady (5) over rational (8), because held-out
  testing "showed the rational model's three extra coefficients bought nothing
  at 82 deg" (`fit_intrinsics.py`, `layout.yaml`). `compare_models.py` saves
  only training RMS: on the 2026-09-24 session BC 0.3756 px, rational
  0.3746 px, fisheye 181 px (failed fit). **The held-out numbers behind the
  choice are not recorded**, so the reason is unverified from repo data.

## 17. Synthetic triangulation accuracy, nominal layout

**Verified as a computation; a floor, not a forecast.** Run 2026-10-17 by
`python/accuracy_study.py`. Assumes perfect calibration, 0.054 px centroid
noise, shared 33661 intrinsics with placeholder distortion, geometric
visibility only. Method and full tables:
[`analysis/README.md`](../analysis/README.md); numbers and provenance:
`analysis/accuracy_study.json`.

| | Median | p90 | Max |
|---|---:|---:|---:|
| Whole volume, RMS 3D error | 0.098 mm | 0.193 mm | 0.82 mm |
| Perch box (perch ± 12 in), RMS 3D error | 0.077 mm | 0.089 mm | 0.102 mm |

- **19% of the volume can't be triangulated** (80.7% seen by 2+ cameras);
  24.3% is seen by fewer than 3. The gap is above about 90 in: every camera
  aims at z = 30 in. Coverage, not accuracy, is the main volume finding.
- The perch box is fully covered: 96% of points seen by all ten, none by
  fewer than eight.
- Camera heights balance the axes. Upper six alone: z weak (0.105 mm median,
  perch box). Lower four alone: y weak (0.075 mm). All ten: 0.043 / 0.047 /
  0.043 mm.
- Any six cameras are worse, up to ~1.8× on the perch box; the worst sixes
  keep one lower camera.
- 20 perturbed layouts (±2 in position, 2° aim, ±2° roll) move every headline
  by at most 9.3%. Tape-measured positions are good enough.
- For scale, the previous rig measured 13.8 mm loop closure over a 9 m
  circuit.
- The study overstates noise by ~√2 (§6).

## 18. Build and host environment

**Verified on this host** except where noted.

- Every camera binary needs `cap_net_raw,cap_net_admin+eip`. Linking writes a
  new file, which **drops file capabilities**; a fresh build on 2026-10-19
  showed none on any binary under `getcap`.
- Running a tool under `sudo` breaks desktop audio for the session and leaves
  root-owned files (from the previous RUNBOOK; not re-tested).
- C++ links system OpenCV **4.6** (`libopencv-dev`); Python uses **5.x** from
  `opencv-contrib-python` in `.venv`. OpenCV 5 removed the `aruco` calibration
  path 4.6 uses and added `CharucoDetector` / `matchImagePoints`; 5.x can't
  load into a process already linked to 4.6. The venv also keeps the wheel's
  numpy 2.x away from apt's `python3-matplotlib` and `python3-scipy` (numpy
  1.x).
- Ubuntu 24.04 marks system Python externally managed (PEP 668), so `pip
  install` outside a venv is refused. `python3.12-venv` is not installed by
  default.
- On 2026-10-19 no NetworkManager profile was bound to `enp6s0` (link down).
  How it was given link-local addressing is not recorded.

## 19. The calib.io 12×9 board

**Verified on hardware, 2026-10-27, camera 33664.** Replaced the old 9×6
board for intrinsics. Definition and provenance: `config/charuco_board.yaml`.

| | |
|---|---|
| Vendor | calib.io, on an 800 × 600 mm aluminium plate |
| Pattern | 9 rows × 12 cols of 60 mm squares (nominal), 720 × 540 mm |
| Markers | 45 mm (nominal), `DICT_5X5_100`; 54 markers, IDs 0–53 |
| Corners | 11 × 8 = 88 interior corners |
| `legacy_pattern` | `false`; with 9 rows it makes no difference (§9) |

- Measured sizes are the caliper values in the YAML; nominal is never used in
  their place.
- **Detection test.** `03_detect` on 33664: 54/54 markers, 88/88 corners, max
  ID 53, fit 0.7–0.8 px, identical with `--legacy` and `--no-legacy`. Matches
  the offline check (odd row count: both flags give the same layout in
  `lib/detect` and 5.0), so calib.io's statement that odd-row boards are
  unaffected holds here.
- **5×5 dictionaries are nested.** IDs 0–53 have identical bits in
  `DICT_5X5_100`, `_250` and `_1000` (checked in 4.6 and 5.0). `_100` is used
  for its larger error-correction margin: `maxCorrectionBits` 3 vs 2. The old
  board's sessions record `DICT_5X5_1000`; their IDs 0–26 are the same
  markers.

---

## 20. Kinova Gen3 arm and the Kortex Python API

**Arm, as stated by the operator** (not yet read back by this repo): Kinova
Gen3 at 192.168.1.10, Kortex TCP port 10000, firmware 2.8.0-5, 6 DOF, no
gripper, reached from `enp6s0` at 192.168.1.11/24 alongside the cameras'
link-local address. `python/arm.py status` prints the arm's own values for
each; until it has been run, none is verified (Open items).

**Kortex API install, verified 2026-10-28** on this host, Python 3.12.3.

- Wheel `kortex_api-2.6.0.post3-py3-none-any.whl` from Kinova's Artifactory
  (`.../generic-public/kortex/API/2.6.0/`), sha256
  `22863493b89ae0ef5270afd58881842cc1814df75736e8ceb0f0d79b869dbbe9`, built
  2023-04-28. Its metadata: `Requires-Dist: protobuf (==3.5.1)`,
  `deprecated (==1.2.7)`. An exact pin, so no newer protobuf satisfies it.

| protobuf | Installs | Imports `kortex_api` |
|---|---|---|
| 3.5.1 (the pin) | yes | **no**: `AttributeError: module 'collections' has no attribute 'MutableMapping'` (removed in Python 3.10) |
| 3.20.3 (last 3.x) | yes, with `--no-deps` | **yes**; pure-Python implementation; messages round-trip |
| 4.25.8 | yes, with `--no-deps` | **no**: `Descriptors cannot be created directly` (unless `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python`) |

- **Warnings with 3.20.3.** None under the default filters. With
  `-W always`: 101 `DeprecationWarning`s on import, 98 `EnumValueDescriptor()`
  and one each `EnumDescriptor()`, `FileDescriptor()` ("Create unlinked
  descriptors is going to go away") and `datetime.utcfromtimestamp()`.
  Harmless today; they mark why 4.x fails.
- `pip check` reports
  `kortex-api 2.6.0.post3 has requirement protobuf==3.5.1, but you have protobuf 3.20.3`.
- Installed into `.venv` alongside numpy 2.5.2 and opencv-contrib-python
  5.0.0.93; both still import.
- API 2.6 against firmware 2.8 has not been exercised. Kinova versions them
  separately; whether every call `arm.py` uses answers on 2.8.0-5 is
  unverified.

## Open items

The single open-items list. CLAUDE.md and the other docs link here.

1. **cx cross-session discrepancy.** 9.4 px across three sessions of 33661
   (§16). Run `bootstrap_intrinsics.py` on one session to see whether pose
   sampling explains it; not yet recorded.
2. **WRONG MODE false alarm.** `05_identify` and `07_mask` print it when
   `modeIsObject()` is false. That flag describes the last *fetch*, which
   resets it when no frame was waiting, so a healthy camera with nothing
   queued prints `WRONG MODE`. *Inferred* from `lib/camera.cpp`; not
   reproduced deliberately.
3. **Tools don't report mask state.** Only `07_mask` knows about masks, and it
   leaves them applied. `05_identify`, `07_objdiag` and `04_capture` neither
   read nor clear them, and `diagnostics()` shows the SDK flag, not the
   camera's state. A masked camera looks like a detection bug. Whether masks
   survive a restart or power cycle is `mask --probe` step 1; no result
   recorded.
4. **Mask tool's compile-time config path.** `MOCAP_CONFIG_DIR` is the
   absolute path of `config/` at configure time; moving the repo breaks the
   default. Workaround: rebuild, or `--config DIR`.
5. **Frame sync** via `cModuleSync` (SDK header: "utilized to synchronize
   frame data"), not `Synchronizer`. Not built; unverified on hardware.
6. **Marker labeling.** Not built.
7. **Extrinsics: wand and L-frame.** Not built. A wand works in object mode,
   which is synchronised; a board needs grayscale, which is not
   (`src/notes.md`).
8. **Reference markers.** Not built.
9. **Object-mode operating point** (§13). Two settings sets in use; masks are
   valid only for the one they were learned under.
10. **Mask acceptance walk has no recorded PASS** (§15).
11. **Auto capture (`04_capture --auto`) is unvalidated** on real data. The
    rule is in RUNBOOK §9: an auto and a manual session, same camera, same
    day, agreeing within fx/fy ~1 px and cx/cy ~3 px.
12. **Grayscale fps record is inconsistent** (§5): logged 9.7–14.7 fps against
    the stated ~8 / 6–8 fps.
13. **Inconsistencies found 2026-10-19:**
    - `src/02_stream` and `src/03_detect` use the short discovery window
      (6 stable polls, 40-poll cap) that undercounts ten cameras (§10).
    - `05_identify` hardcodes the 30 Hz exposure ceiling, 33133 (§3).
    - `session.json`'s `created` is rewritten on every flush, so it records
      the last write. The folder name is local time; `created` is UTC.
    - `config/cameras.yaml` and `docs/hardware/cameras.md` list six cameras
      with 33275 "excluded, defocused"; `config/layout.yaml` mounts ten,
      including 33275 as camera E, and all ten have masks. No code reads
      `cameras.yaml`.
    - `PyYAML` is in `python/requirements.txt` but unused; `rig.py` reads YAML
      through `cv2.FileStorage`.
    - `layout.yaml` distortion is assumed although fitted coefficients exist
      (§16).
14. **Does the 2–3 m detection limit matter for extrinsics?** Not for
    intrinsics (bring the board closer). Extrinsics needs a target several
    cameras see at once across the volume, so beyond 1.5 m for some. Whether
    the board is detectable there at usable rates, or extrinsics needs a
    different target, is untested.
15. **Distortion-coefficient interpretability.** (Written for the rational
    model; the fit is now Brown-Conrady, §16.) Rational coefficients come out
    large and correlated (k1≈k4, k2≈k5). Geometry is accurate regardless, but
    there is no threshold on the coefficients for spotting a bad fit.
    Comparing undistortion displacement fields between calibrations would be
    more direct; not built.
16. **Whether 5×6 is the right coverage grid.** 30 cells against 88 corners
    per view (40 on the old board) means one capture can touch many cells. A
    finer grid would be stricter; the right granularity is not established.
17. **Arm first contact not run.** `python/arm.py status` has never reached
    the arm (§20). Record its output here: firmware, serial, DOF and end
    effector against the stated values, and whether each Kortex call
    answers on firmware 2.8.0-5.
