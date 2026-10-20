# Motion capture system — build notes

Learning goal: understand the logic well enough to explain the project,
justify the decisions, and defend them in an interview. Implementation
trivia (version quirks, API signatures) lives in code comments, not here.

---

## Why a mocap system at all

The drone has to estimate its own position using only onboard sensors.
Proving it does that accurately requires an independent measurement of
where it actually was. That's what mocap provides: ground truth.

Hard rule: mocap data never enters the drone's control loop. It measures,
it doesn't steer. That separation is what makes the project's central
claim meaningful — if mocap were flying the drone, the onboard estimate
wouldn't be proving anything.

## Why we're building it instead of buying it

OptiTrack's own software (Motive) is Windows-only. The vendor's Linux SDK
gives us camera control, hardware frame sync, and 2D marker centroids —
and nothing above that. No calibration, no triangulation, no 3D.

So the layers we own: calibration (intrinsic and extrinsic),
triangulation, marker labeling, rigid-body pose, streaming output.
That's the whole system above the 2D layer.

## 09/08 — Discovery

Cameras power over PoE, self-assign link-local addresses, and broadcast
their presence on the network. So "finding a camera" is a network
operation, not opening a file. Discovery has to listen and wait.

The vendor SDK is thinly documented and its behavior has to be
characterized empirically rather than read about. Discovery was the first
example: the "cameras are ready" signal fires when the *first* camera
initializes, not all of them. Trusting it gives an incomplete list. We
poll until the count stops growing instead.

That pattern — verify by measurement, don't trust the documentation or
the UI readback — recurs throughout this project.

## 09/11 — Two video modes, and why it matters

The camera can send two very different things:

**Object mode** — the camera thresholds onboard and sends a short list of
bright-blob centroids (x, y, area, roundness). Small, fast, and
hardware-synchronized across cameras. This is what mocap runs on in
operation.

**Grayscale mode** — the full raw image. Large, slower, and *not*
synchronized across cameras.

Calibration needs grayscale, because a checkerboard isn't a set of bright
blobs — you need the actual pattern. This split has a consequence that
shapes the extrinsics decision later: a board-based method requires
grayscale, which isn't synced, so if the board moves between two cameras'
exposures their views disagree and no single board pose fits both. A wand
of retroreflective markers works in object mode, which *is* synced. That's
why commercial mocap uses wands for extrinsics, and it isn't arbitrary.

## 09/23 — Streaming

Working acquisition: open a camera by serial, request grayscale, pull
frames, save one as PNG.

Two behaviors worth understanding rather than memorizing:

- Mode changes don't take effect instantly. Frames already queued carry
  the old mode, so every frame's type is verified before it's trusted.
- The frame queue hands back the *oldest* frame, not the newest. Read one
  per loop and you fall progressively behind real time. We drain the queue
  each iteration and keep the latest.

Both are the same shape of problem: the API's obvious reading is wrong in
a way that produces plausible-looking output rather than an error.

## Hardware findings that drive settings

Measured on camera 33661, all verified:

- **Exposure ceiling is frame-period-derived**: `floor(1e6/fps) − 200`.
  At 30 Hz that's 33133 units, 4× the headroom available at 120 Hz.
  Originally believed to be a fixed ~6000; the UI was caching a stale
  range. Trust measurements over readbacks.
- **The 850 nm IR bandpass filter stays installed during calibration.**
  With IR intensity 15 and adequate exposure, all 27 markers detect at
  ~1.5 m. This matters because a flat glass plate in the optical path
  shifts the focal plane — calibrating without the filter would produce
  intrinsics that don't apply to the operating configuration.
- **Gain is effectively free** at operating exposure. Static-board corner
  jitter: 0.0561 px at gain 0 vs 0.0540 px at gain 7, 40/40 corners both
  ways. Matches the ~0.05 px static noise seen on the previous rig,
  measured independently.
- **Working distance for intrinsics is ~1–1.5 m.** Further out the board
  fills too little of the frame and corners cluster mid-image, which is
  the coverage failure described below. The 2–3 m detection limit will
  matter for extrinsics, where the board must be seen across the volume.

## The most important calibration lesson

RMS reprojection error is not a calibration acceptance criterion.

RMS measures fit quality over the data you collected. If board poses
cluster mid-frame, you're only measuring error where lens distortion is
mild, and the distortion coefficients are then *extrapolating* at the
frame edges rather than being measured there. The metric looks clean
precisely because the data is narrow.

Tested against synthetic data with known ground truth:

| Set              | Empty cells | RMS      | fx (truth 780) | Undistort error, outer 20% |
|------------------|-------------|----------|----------------|----------------------------|
| centre-clustered | 18/30       | 0.262 px | 781.3          | 55.96 px                   |
| edge-covered     | 0/30        | 0.223 px | 780.0          | 26.82 px                   |

The badly-sampled set had *better* RMS and recovered focal length to 0.2%,
with fx/fy agreeing to 0.013% — and was 10× worse at the frame edge.

So neither RMS nor fx/fy agreement detects bad spatial coverage. Live
frame coverage is the acceptance criterion. Coverage you can see during
capture is fillable; coverage discovered afterward means recapturing.