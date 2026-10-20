# Mocap findings

What was learned running OptiTrack Prime 13W cameras and calibrating them on
this rig, and why it matters. This file is authoritative for **operational**
findings: what settings work, what the SDK actually does, what was measured.

Camera inventory, serials, model specifications and network layout live in
[hardware/cameras.md](hardware/cameras.md) and are not
repeated here.

Commands are in [RUNBOOK.md](RUNBOOK.md).

Every claim below marked **Verified** includes the measurement it came from.
Open questions are collected at the end.

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
The tool shows the two numbers separately for this reason.

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

Reproduce with `python/corner_jitter.py <session_dir> ...`.

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
field applied per pixel), then solved with this repo's own
`solve_intrinsics.py`:

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
afterwards, it means recapturing. This is why the capture tool draws the
coverage grid live and why the solve panel repeats the empty-cell count next to
the intrinsics.

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
image. The tell is fx drifting away from fy. The tool now estimates board tilt
per capture via `solvePnP` and flags when fewer than 30% of captures exceed
20°.

---

## 7. `setLegacyPattern(True)` is required on OpenCV 5.x

**Verified.**

`board.setLegacyPattern(True)` is required in the Python solver even though the
board was generated with calib.io's "ChArUco Legacy" checkbox **unchecked**.
Without it, `CharucoDetector.detectBoard()` returns zero corners on frames
where `detectMarkers` happily finds 26 of 27 markers — a silent failure that
looks like a detection problem rather than a layout mismatch.

The C++ side needs no such flag: OpenCV 4.6's `CharucoBoard` already uses what
later versions call the legacy layout. Confirmed on real captures — 4.6's
`interpolateCornersCharuco` returns 40.0 of 40 corners on average across both
20-frame sessions with no flag set.

So the same board is described two different ways depending on the OpenCV
major version. `legacy_pattern: true` in
`config/charuco_board.yaml` carries the flag to the 5.x solver;
`session.json` records it so a session stays interpretable later.

---

## 8. Device discovery is staggered and can duplicate

**Verified.**

`WaitForNewDevice()`-style waits return as soon as the *first* device
initialises, well under a second, while a cold camera network takes 4–8 seconds
to enumerate fully. Anything that trusts the first result sees a fraction of
the cameras.

`CameraList` can also return **the same serial more than once** — observed
consistently on 33661, which appears twice on every enumeration.

The tool polls the device list until the count has been stable for 1.5 s, and
collapses duplicates by serial, preferring the entry that has reached
`Initialized`.

---

## 9. Other SDK behaviour worth knowing

**Verified.** All of these are handled in
`src/intrinsics_capture/camera_source.cpp`.

- `SetVideoType` is asynchronous. `VideoType()` reports the new mode
  immediately, but frames already queued still carry the old one, and
  `GrayscaleData()` on an object-mode frame returns null. In practice exactly
  one object-mode frame arrives after `Start()` and must be dropped.
- `NextFrame()` returns the **oldest** queued frame. One call per loop
  iteration falls further behind on every pass; the queue has to be drained to
  the end each time.
- SDK listener callbacks fire on its own discovery thread. Calling back into
  the SDK from inside one can deadlock, so callbacks only touch local state.
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

---

## Open questions

- **Does the 2–3 m detection limit matter for extrinsics?** It does not for
  intrinsics, where the board can simply be brought closer. Extrinsics needs a
  target visible to several cameras at once across the capture volume, which
  will put it further away than 1.5 m from at least some of them. Whether the
  ChArUco board is detectable at usable rates at those distances — or whether
  extrinsics needs a different target entirely — is untested.
- **Distortion coefficient interpretability.** Rational-model coefficients come
  out large and correlated (k1≈k4, k2≈k5) because the model is a ratio of two
  polynomials. Recovered geometry is accurate regardless, but there is no
  established threshold on the coefficients themselves for spotting a bad fit.
  Comparing undistortion displacement fields between calibrations would be a
  more direct check, and is not yet built.
- **Whether 5×6 is the right coverage grid.** 30 cells over 40 corners per view
  means a single well-placed capture can touch many cells at once. A finer grid
  would be stricter; nothing has been done to establish the right granularity.
