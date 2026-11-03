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
- **Arm-driven intrinsics, 33664 (validated):** 3 sessions, RMS 0.17–0.18 px; mean fx 771.83, fy 771.53, cx 629.49, cy 518.84 px; range fx 0.50, fy 0.39, cx 1.03, cy 0.47 px, against the hand-held 33661 same-day pair's fx 0.23, fy 0.63, cx 2.51, cy 1.51 px (cx spread 2.4× smaller, cy 3.2×); gain 0, intensity 15, exposure 8133 µs (gain 0 against IR ring glare at near poses); joint fit over all 80 views: fx 771.89, fy 771.59, cx 629.63, cy 518.82 px, RMS 0.172 px, in `config/intrinsics/33664.yaml`; 2026-11-02, `intrinsics_33664_20261102_150512` / `_184959` / `_190116`, `python/fit_intrinsics.py`. §16
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
- **First arm session:** 5 accepted in 35.7 s, then `ACTION_ABORT INVALID_PARAM` at the 6th target (the Cartesian planner rejected the path, below); reach error ≤ 0.09 mm / 0.009°; rotation ran at 2.5–2.7°/s against 10 configured; 87% of the time moving; 2026-10-28, camera 33664, `tests/fixtures/session_arm_33664_20261028_185017.json`. §20
- **Arm-camera probe:** θy +11.39 px/deg in image x, θx −13.13 px/deg in y, θz −1.03° roll per deg; returns within 0.40 px; board at 31.5 px/sq (far); 2026-10-28, camera 33664, `config/arm_camera.yaml`. §20
- **Arm session 2:** 15 accepted, 29/30 cells, 10 bins short, 389 s, stopped by Ctrl-C; cell 29 tried 7 times, never filled; 2026-10-28 19:10, camera 33664, `tests/fixtures/session_arm_33664_20261028_191025.json`. §20
- **Camera on the arm:** optical axis 12.06° off tool z (rotations of 13 frames, PnP against the arm), 13.4° from the size gains; lens f 768 px against 653/752 px from the rotation gains; same session, 23 views, RMS 0.16 px. §20
- **Kortex API on Python 3.12:** wheel 2.6.0.post3 imports with protobuf 3.20.3 (`--no-deps`), not with its pinned 3.5.1 or with 4.25.8; 2026-10-28, `.venv`. Never yet connected to the arm. §20
- **Joint moves:** `reach_joint_angles` with `JOINT_CONSTRAINT_SPEED` 10 aborts `METHOD_FAILED` on short moves; `GetTrajectoryErrorReport`: `INVALID_JOINT_ACCELERATION`, actuator index 2, value 10, max 4.41. The same `goto-pose hub` with no constraint succeeded; a 5° joint-6 move succeeded with the constraint. 2026-11-02, serial WO520851-2. §20
- **Arm joint limits:** from the Web App (Kortex 2.6 can't read them): actuator 2 [−129, +138], 3 [−148, +152], 5 [−120, +128], 1/4/6 continuous. At power-up the arm rests at actuator 2 −129.4°, 3 −160.2° (12.2° past its limit, no fault), 5 −120.0°; jog to mid-range before every session. 2026-11-02, serial WO520851-2. §20
- **Taught viewpoints, planned on the arm:** joint-space blend impossible (taught poses span 248.7° on joint 2); pose-space plan: 16/27 viewpoints solved by the arm's IK; families on joints 2/3/5 only: 5 families, 4 hub moves starting at the hub (6 and 7 counting every joint and not starting at the hub); 4 outside the box, 6 without an IK solution, 1 near a limit. 2026-11-02, read-only IK, `tests/fixtures/arm_poses_20261102.yaml`. §20
- **Taught sessions, 2026-11-02:** 15:05, 28 accepted, 30/30 cells, 4 bins short, 649 s, 55 of 100 targets IK-unreachable (23/23 pans at near corner `lr0_hl0_nf0`); 17:52, 10 accepted, ended on "board not back in view after returning from a skipped target", 31 unreachable (28 at the two near-high corners); 18:49 and 19:01, after the fixes: 23 and 25 accepted, 30/30 cells, 0 viewpoints abandoned, 3 at the reach edge each. Camera 33664, `tests/fixtures/session_arm_33664_20261102_150512.json` / `_175213.json`. §20
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
21. **Joint moves carry no speed constraint by default.** With
    `JOINT_CONSTRAINT_SPEED` the arm validates the implied acceleration and
    aborts a short move `METHOD_FAILED` (`INVALID_JOINT_ACCELERATION`); with
    no constraint it picks a profile itself. `--speed-constraint DEG_S` is
    opt-in. (§20)

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
  For jitter only: arm-driven sessions needed gain 0 against IR ring glare
  at near poses (§16).
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

## 16. Intrinsics: 33661 hand-held, 33664 arm-driven

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

**Arm-driven intrinsics of 33664, validated 2026-11-02.** Camera on the
Kinova arm, calib.io 12×9 board (§19) on the easel, taught session
(`arm_capture.py session`, §20, RUNBOOK §13) into `04_capture --auto`.
Settings read back from the camera in each `session.json`: gain 0,
intensity 15, exposure 8133 µs, 30 Hz, IR filter in. Fit by
`python/fit_intrinsics.py` (OpenCV 5.0.0, Brown-Conrady, all frames,
1280 × 1024); every session ended with 0 of 30 coverage cells empty.

| Session | Views | RMS (px) | fx | fy | cx | cy |
|---|---:|---:|---:|---:|---:|---:|
| `intrinsics_33664_20261102_150512` | 29 | 0.180 | 771.64 | 771.40 | 630.09 | 518.59 |
| `intrinsics_33664_20261102_184959` | 25 | 0.168 | 772.14 | 771.79 | 629.06 | 518.85 |
| `intrinsics_33664_20261102_190116` | 26 | 0.166 | 771.71 | 771.41 | 629.32 | 519.06 |
| Mean | | | 771.83 | 771.53 | 629.49 | 518.84 |
| Range | | | 0.50 | 0.39 | **1.03** | 0.47 |
| **Joint fit, all frames** | 80 | 0.172 | 771.89 | 771.59 | 629.63 | 518.82 |

The joint fit (`fit_intrinsics.py --write-camera`, run by Claude
2026-11-03 03:31 UTC) is the camera's result, in
`config/intrinsics/33664.yaml`, with the ranges above as `uncertainty_px`.
Its dist is k1 −0.0939, k2 0.0877, p1 −0.0004, p2 −0.00004, k3 −0.0266.
Each joint parameter lies inside the per-session range.

- **Against hand-held repeatability** (33661 above, old 9×6 board): the
  same-day hand-held pair spread fx 0.23, fy 0.63, cx 2.51, cy 1.51 px.
  The arm's three sessions spread cx 1.03 and cy 0.47 px: **cx 2.4× and cy
  3.2× tighter**, fy tighter too (0.39 against 0.63); fx is wider (0.50
  against 0.23), still well inside the ~1 px agreement rule of RUNBOOK §9. Against all three hand-held sessions (cx 9.40) the cx gain is
  9×. cx was the hand-held weak point (Open items #1). Not like for like:
  another camera, another board, and three arm sessions against a pair.
- **Gain 0, not the gain 7 of §6.** At the near poses the camera's own IR
  ring glared off the board; gain 0 was needed against it (operator's
  observation; not measured). §6's jitter result is unaffected, but gain is
  not free at near poses with the ring at 15.
- Distortion agrees across the three: k1 −0.093 to −0.095, k2 0.084 to
  0.090, k3 −0.024 to −0.029.
- The board's `square_mm.measured` is still the 60.0 placeholder
  (`config/charuco_board.yaml`). K doesn't depend on the board's scale, so
  these numbers stand; extrinsics would not.
- This is repeatability of auto sessions. It is not the auto-against-manual
  check of Open items #11.

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

**`arm.py status` readback, 2026-11-02** (read-only, run by Claude):
firmware bundle `2.8.0-r.8` (Armbase 2.8.0-r.5, actuators and interconnect
2.7.0-r.1, vision 2.3.1-r.5), serial `WO520851-2`, 6 DOF, end effector
`END_EFFECTOR_TYPE_ROBOTIQ_2F_85` (the operator's statement above says no
gripper), devices BASE 0, BIG_ACTUATOR 1–3, SMALL_ACTUATOR 4–6,
INTERCONNECT 7, VISION 8. Every read `status` makes answered on 2.8.0 with
API 2.6. Pose x +0.4408 y +0.0544 z +0.2941 m, θ +89.875 / −4.474 / +95.128°;
servoing `SINGLE_LEVEL_SERVOING`. Joints, deg, as reported:

| Actuator | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| Reported (0–360) | 169.979 | 230.559 | 199.779 | 185.307 | 239.964 | 87.952 |
| Wrapped to ±180 | +169.979 | −129.441 | −160.221 | −174.693 | −120.036 | +87.952 |
| User guide limit (Table 45) | continuous | ±128.9 | ±147.8 | continuous | ±120.3 | continuous |
| Web App limit [low, high] | continuous | [−129, +138] | [−148, +152] | continuous | [−120, +128] | continuous |
| Margin to the Web App limit | — | −0.4 | **−12.2** | — | −0.0 | — |

**Joint position limits: the Web App shows them; Kortex 2.6 can't read
them.** The Web App's Safeties → Actuators page, "Joint Limit Low/High",
read by the operator 2026-11-02 (serial WO520851-2, firmware 2.8.0):
actuators 1, 4 and 6 at 0/0 (disabled, so continuous); actuator 2
[−129, +138]; 3 [−148, +152]; 5 [−120, +128]. They are asymmetric and
are not the user guide's ±128.9 / ±147.8 / ±120.3. `config/arm.yaml` holds
them as `[low, high]` (ARCHITECTURE §9).

The Kortex 2.6 API (wheel 2.6.0.post3; protobuf descriptors of every
service listed 2026-11-02) has no position-limit read. Its limit reads are
`Base.GetAllJoints{Speed,Torque}{Hard,Soft}Limitation` (`LimitationType`:
force, acceleration, velocity, torque only),
`Base.Get{Twist,Wrench}{Hard,Soft}Limitation` and
`ControlConfig.GetKinematicHardLimits` / `GetKinematicSoftLimits` /
`GetAllKinematicSoftLimits` (`KinematicLimits`: twist, joint speed, joint
acceleration). ActuatorConfig has none. Kinova's `master` ControlConfig
reference lists the same calls. So the values are copied by hand, and go
stale if someone changes them in the Web App.

**The arm powers up resting on its stops** (operator's account; it has no
brakes). The pose in the table above is that resting pose: actuators 2 and 5
sit at their low limits (−0.4° and −0.04° past), and actuator 3 at −160.2°,
**12.2° past its −148 limit, without a fault**. Why actuator 3 rests that far
outside its configured limit is not known. Every session must start by
jogging the arm to mid-range: until then `arm.py` refuses a joint target out
there and the session IK check refuses paths near it. After the operator
jogged it, the same day, `status` read actuator 2 −89.2° (margin +39.8),
3 −109.5° (+38.5), 5 −80.1° (+39.9), and a dry-run `joints` move back to the
resting angles was refused on joints 2, 3 and 5.

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

**Motion (`arm.py move` / `goto` / `stop`), 2026-10-28: nothing measured.**
The code has run only against a fake `BaseClient` (in a scratch directory,
not the repo). That checked:
- dry runs send nothing;
- box, timeout-estimate and servoing-mode refusals send nothing;
- the one action sent is a `reach_pose` with the configured speeds;
- subscribe comes before `ExecuteAction`, and unsubscribe after;
- an abort or a timeout exits non-zero, and a timeout sends `Stop`.

Not known, and not in the Gen3 user guide:
- whether a `reach_pose` moves the tool in a straight line, which the
  box-convexity argument relies on (ARCHITECTURE §9);
- whether the arm stops a move in progress when the session drops.

**Euler convention, from the Gen3 user guide** ("Cartesian reference
frames": "x-y-z extrinsic convention"), not checked on the arm:
R = Rz(θz)·Ry(θy)·Rx(θx). `arm_capture.py` builds its tool-frame targets
on it. If it were wrong, the probe's rotations would be about other axes
than labelled. The labels in `config/arm_camera.yaml` would then be wrong,
but the motions would still be 2° and 30 mm.

**Probe result, 2026-10-28 18:26 local** (`config/arm_camera.yaml`,
`created_utc` 2026-10-29T01:26:48Z, camera 33664). It was run by the
operator; the terminal output and conditions were not recorded, so
everything here is read from the file. Camera on the arm, board on an easel.

- Start pose x 0.5495, y −0.0301, z 0.4806 m, θ 84.20 / −1.56 / 95.85°.
  Board at the start: centroid (627.4, 503.3) px, square 31.5 px (far band;
  ~1.5 m at 780 px focal), roll 93.0° (board rows vertical in the image).
- Per +1° of tool rotation: θx moves the board −13.13 px in y (−0.54 in
  x); θy moves it +11.39 px in x (+0.45 in y); θz rolls it −1.03° and
  moves it under 0.4 px. Each runner-up axis is 3–5% as strong as the
  chosen one (`second_best_ratio`).
- Per +1 m of tool translation: x moves it +724 px in x; y moves it
  +531 px in y (and −4.8 px of square); z moves it +259 px in x and grows
  the square +20.2 px.
- Return errors 0.02–0.40 px against the 3 px limit.
- This shows, from the file alone, that `capture --auto-dry --status-file`
  ran on the camera, that the arm made the probe's twelve box-checked moves,
  and that every return came back within 0.4 px.
- *Inferred:*
  - The clean single-axis θx/θy/θz responses fit the x-y-z extrinsic
    convention the tool-frame targets are built on. They don't prove it.
  - Tool z is not the optical axis: moving along z shifts the image
    259 px/m sideways. The x and y translation gains also differ (724 vs
    531 px/m) at one depth. The camera is probably mounted at an angle to
    the tool frame; the session uses the measured gains, so it needs no
    hand-eye model.
  - At this P0 (31.5 px, far) the session's ±0.10 m depth variants give
    ~33.5 and ~29.5 px: all far. Its dry run against this file says to
    move the board about 0.30 m closer and rerun the probe.

**First arm-driven session, 2026-10-28 18:50 local**, camera 33664, P0 at
the probe's start pose. Session `intrinsics_33664_20261028_185017`; record
copied verbatim to `tests/fixtures/session_arm_33664_20261028_185017.json`.
Read from the record; nothing else was logged.

- **Result.** 8 moves, 6 targets. Capture accepted 5 frames in 35.7 s
  (`capture_accepted_total` 6, one before the session). It ended on the
  6th target with `ACTION_ABORT`, reason `INVALID_PARAM`. 16 cells and 13
  bins were left. The arm was not moved after the abort.
- **Reach accuracy.** Each reached pose was within 0.05–0.09 mm and
  0.006–0.009° of its command (measured pose against commanded).
- **Speed.** Rotations of 6–27° ran at a steady 2.5–2.7°/s, against
  `max_rotation_dps` 10. Arm motion was 87% of the session's time;
  waiting for a still board was 5%, 0.30–0.35 s per target. 4.5 s per move.
  Why the arm is capped near 2.7°/s (a soft limit on the arm, or how the
  firmware reads `CartesianSpeed.orientation`) is not known.
- **The P0 view itself was binned `far/-x-y`:** tilted 20° or more from
  square-on, as capture computes tilt. The flat bin can't be reached near
  this P0. Entry 4, aimed at the top right of the frame, was binned
  `mid/+x-y` while P0 reads far (31.5 px/sq). That is the off-axis bias of
  open item 18, now on the real camera (*inferred* from the bins).
- **The aborted target** (entry 5, cell 2): x 0.54954, y −0.03009,
  z 0.4805 m (P0's position), θ 102.517 / −1.680 / 97.789°. That is a
  rotation of (18.37, 1.95, 0)° about the tool axes from P0, reached from
  entry 4's pose by a 16.6° rotation, mostly about tool y (−16.3°). The
  abort came 4.5 s into a move that takes ~6 s at 2.7°/s, so about 3/4
  along the path, not at submission.
- **Why the arm refused it: not determined.** Ruled out from the record:
  - the box, because the position is P0's;
  - the Euler range, because θy is −1.7° and every angle is ordinary;
  - the speed field, because the same constraint ran the 7 moves before
    it, all pure rotations like this one.

  The endpoint is the largest tool-x rotation commanded (18.4°, against
  17.2° reached). The record has no joint angles, so a joint limit or
  singularity handling along the path can't be checked. Its timing points
  that way, but `INVALID_PARAM` is a generic code, not
  `CONTROL_JOINT_POSITION_LIMIT` or `CONTROL_OUTSIDE_WORKSPACE`. The
  Kinova Web App's event log for 18:51 may name it. Since this session,
  every move is IK-checked along its path, and an abort records the joints
  before and after (ARCHITECTURE §9), so a repeat will show more.
- **With relative bands** (P0 ±8%), depth 0.10 m at the probe's
  20.2 px/m along tool z puts all three depths in mid (±6.4%). Near and far
  differ by 12.9% (the sanity threshold is 15%). `depth_m` ≥ 0.124 m would
  separate them. Capture binned every view by its fixed 35/40 px
  thresholds, so P0 was capture-far.

**`INVALID_PARAM` on `reach_pose`, 2026-10-30.** It means the arm's Cartesian
planner rejected the path, from a singular start or across a singularity. It
does not mean a parameter was bad. Rushav's reading from the arm, 2026-10-30;
not reproduced from a record here.

- **Today's example:** not recorded. Today's session folders hold no
  `session_arm.json`, so the command, joints and target aren't here. **To fill
  in from the terminal output.**
- **The 18:50 abort above fits a crossing:** valid numbers, 7 like moves
  before it, and it ended about 3/4 along the path. Which singularity is
  still unknown, as that record has no joints.
- **The 19:10 session's P0 was clear of the wrist singularity:** actuator 5 at
  −105.0° (read from the record).

**`METHOD_FAILED` on `reach_joint_angles`, 2026-11-02 (resolved).** Every
joint move tried had aborted `METHOD_FAILED`, e.g. `goto-pose hub` from a pose
6° away, with the LED green, the Web App closed, every joint mid-range and
`Base.Stop` sent first. With the abort report added to `arm.py` (ARCHITECTURE
§9), the arm's `GetTrajectoryErrorReport` for that move read:

- `TRAJECTORY_ERROR_TYPE_INVALID_JOINT_ACCELERATION`, index 2 (actuator 3,
  `joint_identifier` counting from 0), value 10, max 4.41: "Desired speed for
  actuator (2) results in invalid acceleration".

So the cause is our `JOINT_CONSTRAINT_SPEED` 10 deg/s, not the angles or the
identifiers: on a short move the arm can't reach that speed within its
acceleration limit and rejects the trajectory. Kinova's joint examples send no
constraint. Checks, run by Rushav on the arm, 2026-11-02: the same `goto-pose
hub` with no constraint succeeded at a calm speed (duration not recorded); a
5° joint-6 move succeeded with the constraint. Joint moves now carry no
constraint unless `--speed-constraint DEG_S` (rule 21). The `goto-pose`
failure of 2026-10-30 (Rushav's report; no output recorded) was likely the
same: it sent the same constraint. Unmeasured: the arm's own joint speed and
acceleration when unconstrained; the sessions' records now log each joint
move's `measured_joint_dps`.

**Where the 19:10 session's arm time went, read from its record 2026-10-30.**
389 s total, 339 s moving (87%), 6.2 s moving per move. Cell pans took 297 s
of that over 47 moves, viewpoint moves 34 s over 7, the return 8 s. So faster
viewpoint moves alone recover at most about 30 s of that run; the time is in
the pans, which stay at `max_rotation_dps` and ran at ~2.7°/s (open item 19).

**Second arm session, 2026-10-28 19:10 local**, camera 33664, session
`intrinsics_33664_20261028_191025`; record copied verbatim to
`tests/fixtures/session_arm_33664_20261028_191025.json`. Run with the
lateral variants, before the orbit existed.

- **Result.** 55 moves, 15 accepted (16 frames with the one before the
  session), 29/30 cells, 10 of 15 bins short, 389 s, 11 targets unreachable
  (joints 2 and 5, near P0), no aborts, ended by Ctrl-C.
- **Tilted bins stayed short** because the variants barely change the view
  angle. The frames span 9–47° of tilt (PnP), but mostly from pointing, not
  from where the camera stands.
- **Cell 29 (bottom right).** 7 tries of 4 moves each, none filled it. Each
  try was: first move, board lost, halfway back, correction, lost again,
  back. The board left the frame.
- **The run was already using the atan aim.** `rotation_for_aim()` as it
  then stood reproduces every recorded first command within 0.004°. The
  linear gains would differ by over 1°. So the misses below are not the
  small-angle error.
- **First-try misses, read from accepted frames at first-try poses.** Cell
  18: 938.6 against 896.0 px in x (+43, overshoot). Cell 8: 921.7 against
  877.7 (+44, overshoot). Cell 5: 287.4 against 243.1 (+44, undershoot, since
  that move goes left). Right overshoots and left undershoots, so no single
  focal length fixes both.
- **Why cell 29 was lost** (*inferred*, from a model). Near that aim the
  tilted board is asymmetric about its centroid: predicted 249 px below it
  and 182 px above, reaching 1015 of 1024 px with the aim's margin already
  spent. `aim_for_cell()` assumed symmetry. On the rig-like simulation the
  board is lost at this aim even with the aim hit exactly.

**Camera geometry, measured from that session**, 2026-10-30, offline, on
its PNGs and `session_arm.json`. Not a calibration to reuse: 23 views, one
afternoon, scratch scripts.

- **Lens.** `calibrateCamera` on the 23 frames of `_190515` and `_191025`
  (k1, k2 only, one focal length): RMS 0.16 px, f 767.65 px, principal point
  (627.7, 521.4), k1 −0.085, k2 0.055. Fitting the 33664 jitter session
  `_151829` instead gave fx 847, fy 834, k3 −44: static views can't fix
  the intrinsics (§8).
- **The rotation gains are not the focal length.** 11.39 and 13.13 px/deg
  give 653 and 752 px against 768. They are the response to a rotation about
  the TCP, which also moves the camera.
- **Optical axis.** Rotations between 13 frames, PnP against the arm
  (Kabsch): 12.06° off tool z, toward −y. Residual 0.94° median over 78
  pairs of median rotation 24.5°. The probe's size gains alone give 13.4°,
  1.32° from that. So the 2026-10-28 note above, that the camera is mounted
  at an angle, is now measured, and the probe's file is enough to find the
  axis.
- **Distance at P0.** PnP 1.456 m. From the size gains, 1.513 m. From
  square size and f, 1.336 m with the gains' geometric mean, 701 px (−8%).
- **The camera turns more than the arm reports.** Moving 0.15 m along tool z,
  the arm reports −3.39° about y and PnP sees −1.91°; translations themselves
  match within 10 mm. That is ~10°/m, and it would explain a probe gain no
  rigid pinhole can give: x +724 px/m, against 768/1.456 = 527 px/m at most
  (*inferred*: arm or mount compliance, or the arm's kinematic model).
- **Camera position in the tool frame: not determined.** A rigid fit of the
  camera and board to 9 frames leaves 9.2 px per point and doesn't reproduce
  the θz or translation gains. With the axes fixed, the best single lever on
  11 recorded centroids is (−0.31, −0.40, −0.13) m at 12.9 px RMS. That is
  too long to be the mount: treat it as effective, absorbing the unreported
  rotation. It reproduces the right-overshoot, left-undershoot pattern, so
  the tests use it as a rig-like geometry.

**Aiming and orbit, offline, 2026-10-30.** `python/test_arm_capture.py`, 59
checks. On the rig-like geometry, corners 347–374 px away: first move
leaves 139–151 px, final miss 7–12 px, and cell 29 ends 12 px from its
refitted aim after one lost board and a correction. The other three corners
need no correction. The literal atan(dpx/f) from the current centroid,
in place of the two rays, ended 25 px past the aim on two corners, against
7 px. The orbit's optical axis is on the estimated centre within 0.0001°
by construction. The true board lands 22 px (median), 109 px (worst) from the
image centre, 88/88 at all 75 viewpoints. **None of this has run on the arm.**

**Why ~1.0 m and 10–12° of lean** (*computed*, not run): the default box,
the recorded P0 and gains, the 75-viewpoint cap, box only (the IK needs
the arm). Viewpoints inside the box:

| Easel distance | P0 x 0.55 | P0 x 0.43 |
|---|---|---|
| 1.0 m | 15/75 (r0 9, r+0.20 6, r−0.20 0) | 23/75 (r0 9, r+0.20 2, r−0.20 12) |
| 1.45 m | 10/75 | 12/75 |

At 1.0 m only ±12° of elevation stays in the box. Of the 15 viewpoints
reachable at P0 x 0.55, centred views only: a board square to P0 gives 15
flat; leaned 11°, 12 flat and 3 tilted past 20°; leaned 22°, 6 flat and 9
tilted, all with the same sign of tilt about x. Lean adds tilted views on one
side only; the opposite side still comes from pointing at frame corners.

**Taught session: why the joint-space blend failed, 2026-11-02.** The eight
poses taught that day (`config/arm_poses.yaml`, copied to
`tests/fixtures/arm_poses_20261102.yaml`) span, after unwrapping to the
first corner, 207.4 / **248.7** / 152.5 / 200.3 / 213.7 / 228.5° on joints
1–6 (computed from the file). Blending joint vectors that far apart passes
through arms nobody taught. The old 170° spread rule refused it.

On joint 2, the near corners sit at −110 to −70° and the far-low corners at
+123°: two arm configurations (operator's reading). By the planner's family
rule (more than 90° apart on any joint, continuous joints included) the
eight form 7 groups; only near_left_high and near_left_low share one. Most
of that comes from joints 1, 4 and 6. So the session now blends the taught
Cartesian poses and lets the arm's IK pick the joints (ARCHITECTURE §9).

**Pose-space plan on the arm, 2026-11-02** (read-only: `build_taught()` and
`_plan_hubs()` called from a scratch script on those poses and the
`hub` saved 21:51:41Z, with `ComputeInverseKinematics` only; run by Claude):

| Result | Viewpoints |
|---|---|
| Outside the box (before IK) | 4 (`lr0_hl2_nf0`, `lr2_hl0_nf2`, `lr0_hl1_nf2`, `lr0_hl2_nf2`) |
| No IK solution: "first/second convergence of the IK has failed" | 4 (`lr0_hl0_nf0`, `lr1_hl0_nf0`, `lr2_hl0_nf0`, `lr2_hl2_nf0`) |
| No IK solution: "IK solution found but it is outside of joint position limits" | 2 (`lr1_hl2_nf0`, `lr1_hl2_nf2`) |
| Solved within 10° of a limit (joint 3 +144.5°) | 1 (`lr1_hl1_nf2`) |
| Solved and usable | **16**, in 6 families of 4, 4, 2, 4, 1, 1 |

The `families` line planned 7 hub transitions: 5 between viewpoints, 1 from
the start pose, 1 on the return. The hub passed its checks.

**The arm's IK refuses poses it was taught at, 2026-11-02.** Of the four
"convergence failed" viewpoints above, three are grid corners (`lr0_hl0_nf0`
near_left_high, `lr2_hl0_nf0` near_right_high, `lr2_hl2_nf0`
near_right_low). In a session dry run the same day (Rushav's report; output
not recorded here) IK refused 5 corners with "first convergence of the IK has
failed". At a corner the planner asked IK for the taught pose itself
(`trilinear_pose()` reproduces the eight 2026-11-02 poses to 5.7e-14,
`test_arm_capture.py`), seeded first with the joints of the nearest taught
pose, which is that same corner, so the seed was the exact answer (inferred
from the code; the seed wasn't logged). `ComputeInverseKinematics` is not
reliable at these poses even from the solution. Grid corners now take their
taught joints with no IK (ARCHITECTURE §9). How many non-corner viewpoints
fail, and with which seed, is printed by the dry run's `ik` line; not yet
run.

- The arm's IK applies joint position limits of its own: it says so when it
  refuses.
- It returns continuous joints unwrapped: joint 6 came back at −444.386°.

**Joint targets outside [0, 360) are accepted, 2026-11-02** (run by
Rushav; closes the open item). `arm.py joints` with joint 1's target at 3°
from a measured 347.7° sent joint 1 as 363° (the measured angle plus the
wrapped +15.3°; continuous joints are sent the short way, ARCHITECTURE §9).
`reach_joint_angles` accepted it and the base turned +15°, the short way,
not −345°. So the arm takes angles above 360° on a continuous joint as
given. Below 0° (e.g. −160°) was not tried.

**Families on joints 2, 3 and 5 only, 2026-11-02** (the same read-only plan,
rerun by Claude; arm at the hub, every joint within 0.002° of it). Joints 1,
4 and 6 no longer split families, and a joint move now turns a continuous
joint the short way from its measured angle (ARCHITECTURE §9). The arm
starting at the hub now counts no start or return hub move.

| | Families (viewpoints each) | Hub transitions | Usable viewpoints |
|---|---|---|---|
| Every joint counted (above) | 6 (4, 4, 2, 4, 1, 1) | 7 (5 between, 1 start, 1 return) | 16/27 |
| Joints 2, 3, 5 only, starting at the hub | 5 (4, 4, 2, 5, 1) | 4 (4 between, 0, 0) | 16/27 |

The IK solutions didn't change, only the grouping. The remaining families
differ on the limited joints. Family 1 has joint 2 near −110° and joints
3 and 5 negative; family 4 has joint 2 at +76 to +123°. That follows the
taught corners, which form 4 groups on joints 2, 3, 5:
(near_left_high, near_right_high, near_left_low), (near_right_low,
far_left_high), (far_right_high), (far_left_low, far_right_low). Fewer hub
moves needs corners taught in fewer configurations (Open items).

**Taught sessions on the arm, 2026-11-02** (camera 33664; records copied to
`tests/fixtures/session_arm_33664_20261102_150512.json` and `_175213.json`;
numbers read from the records). Both used the same 19 usable viewpoints in
6 families, visited family by family, so both began at the near corners.

| | 15:05 | 17:52 |
|---|---|---|
| Ended | target list exhausted | "board not back in view after returning from a skipped target" |
| Accepted | 28 | 10 |
| Targets / moves sent | 100 / 123 | 52 / 57 |
| IK-unreachable | 55 (23 `lr0_hl0_nf0`, 16 `lr2_hl0_nf0`, 8 `lr2_hl1_nf0`) | 31 (17 `lr0_hl0_nf0`, 11 `lr2_hl0_nf0`) |
| Cells left / bins short | 0 / 4 (near and far `+x+y`, `-x+y`) | 3 / 12 (every mid and far bin) |
| Total, moving | 649 s, 83% | 283 s, 82% |

- **17:52's end.** Entry 51 (`lr0_hl1_nf0`, cell 4): the target, the
  halfway step and the way back to the viewpoint were all sent and reached.
  The board then didn't settle there (`settle()` gave lost or timeout; the
  record doesn't say which), and the session ended
  with 19 viewpoints planned and 5 visited. The entry was never finished:
  no camera reading, no `total_s`.
- **A failed way back, unnoticed.** Entries 3 and 4 of 17:52: the first leg
  of a pan ran, the board left the view, and the way back failed the IK path
  check. The entry then read `unreachable`, the arm stayed where the board
  was cut off (68 and 48 corners), and the next pan started from there.
- **Near corners are at the reach edge.** `lr0_hl0_nf0` reached no cell in
  either session: 23 and 17 pans, every one IK-unreachable. At 15:05, 21 of
  the 23 failed at the first IK sample (25% of the path); at 17:52 the
  failures spread from 25% to 100%.

Since then (ARCHITECTURE §9, replayed by `test_arm_capture.py` on these
records, and run on the arm at 18:49 and 19:01, below):
- A board out of view after a return, or a way back that fails its checks,
  abandons the viewpoint instead of ending the session or panning on blind.
- A viewpoint whose first two pans are IK-unreachable is left at the reach
  edge. Replayed: 15:05 would skip 48 pans (21 at `lr0_hl0_nf0`, 17 at
  `lr2_hl0_nf0`, 10 at `lr2_hl1_nf0`), but the last two viewpoints went on
  to 3 accepts each after two failed pans, so 6 of its 28 accepts came from
  pans the rule now skips. Whether mid viewpoints, now visited first, fill
  those cells is untested. 17:52 would skip 16 pans, 0 accepted, including
  cell 4 at `lr0_hl1_nf0`, the pan that ended it.
- Visit order mid, near, far: 9 mid viewpoints first (from `lr0_hl1_nf1`),
  then 6 near, then 4 far. Hub transitions between viewpoints go from 6 to 9
  on this plan.

**After the fixes, 2026-11-02 18:49 and 19:01** (records in
`intrinsics_33664_20261102_184959` and `_190116`, schema 6; not copied
into the repo):

| | 18:49 | 19:01 |
|---|---|---|
| Ended | target list exhausted | target list exhausted |
| Accepted / targets | 23 / 57 | 25 / 53 |
| IK-unreachable | 15 | 15 |
| Viewpoints visited / abandoned / at the reach edge | 14 / 0 / 3 | 13 / 0 / 3 |
| Hub moves | 6 | 5 |
| Cells left / bins short | 0 / 7 | 0 / 6 |
| Total, moving | 568 s, 85% | 487 s, 85% |

Both began at the mid viewpoints (`lr0_hl1_nf1` first). The reach edge was
`lr0_hl0_nf0`, `lr2_hl0_nf0` and `lr2_hl2_nf0` both times. The resulting
intrinsics are in §16.

**Status file and session code, offline.** Run only offline:
- the status writer, in `tests/test_autocapture`;
- the reader, against `tests/fixtures/capture_status.json`;
- the probe, against a fake arm and a simulated camera writing status
  files (scratch directory, not the repo). The sensitivities it recovered
  matched the simulation's, and its refusals and return-check stop worked.
- the session (`arm_capture.py session`); first hardware run above. Since
  then tested by `python/test_arm_capture.py` (31 checks then, in the repo),
  including a replay of that abort, IK rejection, joint-limit and
  branch-change rejection, abort recovery and the three-abort stop.
  Before that, tested against a simulation: camera on the arm 8 cm ahead of the TCP, pinhole
  780 px, the board 1.25 m away on an easel, and a Python stand-in for
  `capture --auto` (gates 1–3, 5, 6, coverage, pose bins, status file).
  The probe was run on the simulation first, so the session used gains
  measured the same way. Results:
  - 21 moves; all 30 cells and every flat bin filled; near quadrant bins
    filled.
  - Mid and far quadrant bins stayed short: off-axis views look closer
    (`apparentSquarePx` grows ~1 + tan²θ), so corner views from mid/far are
    binned near. The dry run had predicted it for exactly those bands.
  - With rotation gains deliberately 0.6× wrong: 62 moves, 8 board-lost
    recoveries, 11 corrections, still ending cleanly at P0.
  - Endings tested: `q`, a dying capture, Ctrl-C during a move (all back at
    P0), and an `ACTION_ABORT` (no further move).
  - Every commanded pose was asserted inside the box.

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
17. **Arm: outputs not recorded.** The probe has run on the arm (§20), and
    `arm.py status` was recorded 2026-11-02 (§20): its reads answer on
    2.8.0, and the end effector is a Robotiq 2F-85, not "no gripper". Still
    unrecorded: whether the motion calls answer on 2.8.0. Then the first motion,
    `move --dz 0.02 --yes` (RUNBOOK §13): result, measured error, and
    whether the tool path looked straight. The probe's terminal output is
    unrecorded too. A first session has run and aborted (§20; item 19).
    Next: a full session, its summary and `session_arm.json`, and an
    intrinsics fit of it against a manual one (RUNBOOK §9 rule).
18. **Distance bands are apparent size, so off-axis views read closer.**
    `poseBin()` bands by `apparentSquarePx()`; a board at a frame corner
    looks ~1.3–1.5× bigger at the same distance (pinhole; less with the
    real lens's barrel distortion, by how much is unmeasured). Corner (tilted)
    views are then binned a band closer than their distance, so in
    simulation the mid and far quadrant bins never filled from an arm
    session. Seen on hardware: a top-right view in the first session was
    binned mid while P0 reads far (§20). Hand-held capture has the same
    bias. Candidate fix: band by
    the solvePnP distance `poseBin()` already computes. Not changed, as it
    changes what capture accepts (§20; ARCHITECTURE §9).
19. **Arm aborts with `INVALID_PARAM`; rotations run slow.** `INVALID_PARAM`
    is the Cartesian planner rejecting the path (§20). Sessions now refuse a
    singular wrist at the start, but the IK check along a path does not
    measure closeness to a singularity, so a crossing can still abort:
    check the next record's `joints_after_abort_deg`. Separately,
    rotations run at ~2.7°/s against `max_rotation_dps` 10 (§20): check the
    arm's Cartesian soft limits.
20. **Arm: the camera turns more than the arm reports, and its position in
    the tool frame is unknown** (§20). The planner works from the probe and
    closes the loop on the image, so it doesn't need either. Anything that
    uses the arm's pose as a measurement would, starting with hand-eye
    calibration.
21. **Orbit and two-move aiming have not run on the arm.** Next: a probe at
    the new easel and P0, a dry run, then a session: its `cap` line, its
    `aim_error` lines, and the bins. The box leaves 23/75 of the cap at best
    (§20), and IK can only remove more. Whether that fills the tilted bins, or
    the box or `orbit_angles_deg` has to change, is untested.
22. **Taught sessions have not run on the arm.** Teach has:
    eight taught poses with `square_px` were saved 2026-11-02 (inferred from
    `config/arm_poses.yaml`; its output wasn't recorded). `joints` and
    `goto-pose` have run on the arm, 2026-11-02 (§20, `METHOD_FAILED`);
    `--from-pose` and `session --poses` are tested against stubs
    only; the session's IK plan has run on the arm, read-only (§20). Hub
    moves have never run. Also unknown: how the Web App shows up, since
    "arm free" rests on `Base.Stop` failing while it holds the arm; whether
    the probe's px/deg stays good enough at far taught poses; and whether the
    fixed 35/40 px bands span a far/near ratio under 0.6 (the `WARNING` at
    the dry run). To check at first use: that
    `ComputeForwardKinematics` agrees with the measured pose (`target FK`
    against a saved pose); the arm's unconstrained joint speed (`arm.py`
    prints each move's `duration`; sessions log `measured_joint_dps`); and
    whether a long hub move fits `move_timeout_s` (30 s). The first joint
    move of a session has no time estimate, as nothing is measured yet.
23. **Actuator 3 rests 12.2° outside its limit.** At power-up it read
    −160.2° against the Web App's low limit of −148, with no fault (§20);
    actuators 2 and 5 rest within 0.4° of theirs. Unknown whether the
    limit is enforced only while servoing, the stop sits past it, or the
    Web App and Kortex use different joint zeros. Until it is known,
    `config/arm.yaml` holds the Web App's values and sessions start after
    jogging to mid-range.
24. **Taught corners of 2026-11-02 span four arm configurations** on
    joints 2, 3 and 5 (§20). A session from them plans 5 families and 4 hub
    moves, and 11 of 27 viewpoints didn't solve (before corners took their
    taught joints; the dry run's `ik` and `IK failed` lines now give the
    non-corner count and seeds, not yet recorded). Teaching the corners again
    in one configuration (same signs on joints 2, 3, 5) should cut both;
    untried.
