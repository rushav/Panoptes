# Synthetic triangulation study

Predicts how accurately the PANOPTES rig can triangulate a marker anywhere in
the capture volume, using only the mounted layout, before any calibration
exists.

> **This is a floor, not a forecast.** Every number here assumes perfect
> calibration: the triangulator knows the exact intrinsics and extrinsics
> that produced the data. Real extrinsics carry their own error, and that
> error adds to everything below. For scale, the previous rig measured
> **13.8 mm loop closure over a 9 m circuit**. These sub-0.1 mm figures show
> how much of that budget the camera geometry and centroid noise use up
> (very little). They do not show what the new rig will measure.

## Running it

```
.venv/bin/python mocap/python/triangulate.py      # self-test only
.venv/bin/python mocap/python/accuracy_study.py   # full study, ~2 min on 32 cores
```

| File | Role |
|---|---|
| `config/layout.yaml` | Rig geometry in **inches** as tape-measured, plus the camera model |
| `config/study.yaml` | Grid spacing, noise, trial counts, perch box, subsets, perturbation sizes |
| `python/rig.py` | Loads both configs (the single inch-to-mm conversion), look-at extrinsics, Brown-Conrady projection plus analytic Jacobian, undistortion, visibility |
| `python/triangulate.py` | DLT (SVD) seed, then per-point Levenberg-Marquardt on pixel reprojection error, and the self-test |
| `python/accuracy_study.py` | The study. Writes the PNGs, `accuracy_study.json` (numbers plus provenance) and `accuracy_study.npz` (per-point arrays) here |

The PNGs and the `.npz` are gitignored because they are regenerated on every
run. The JSON is tracked.

## Method

- The grid covers the whole volume at a spacing of 6 in or less (6048 points). The
  perch box (perch ±12 in) gets its own grid at a spacing of 3 in or less (729 points).
- A camera sees a point only if three things hold: the point is in front of it, it
  projects onto the 1280×1024 sensor, and its ray is inside the lens's true field
  of view. The third test matters because Brown-Conrady can fold points far off-axis
  back onto the sensor. The model's field of view comes out at 81.9° × 68.2°,
  against the 82° × 70° spec.
- Every observing camera gets independent Gaussian noise of 0.054 px per image
  axis. Each point is triangulated in 200 trials and the study reports RMS error,
  per axis and in total.
- Each Monte Carlo result is cross-checked against the first-order covariance
  σ²(JᵀJ)⁻¹ (the `MC/pred` column). The median ratio is 0.998 to 1.000 in every
  run. Per point, the 1st to 99th percentile is 0.92 to 1.08, which is the
  sampling noise expected from 200 trials. So the Monte Carlo machinery and the
  linearisation each validate the other.

## Results (run 2026-10-17, commit a7f8ec5 plus these uncommitted files)

### Self-test: passed 12/12

The zero-noise checks recover truth to about 1e-12 mm, nine orders of magnitude
under the 1 µm bar. This holds with every camera and with exactly two cameras.
Fewer than two observers returns NaN and is reported. The projection matches
`cv2.projectPoints` to 2e-11 px.

### Nominal layout, all 10 cameras (RMS 3D error, mm)

| Region | Median | p90 | Max | x / y / z median | 2+ cams | Under 3 cams |
|---|---|---|---|---|---|---|
| Whole volume | 0.098 | 0.193 | 0.82 † | 0.049 / 0.060 / 0.062 | 80.7% | 24.3% |
| **Perch box** | **0.077** | **0.089** | **0.102** | 0.043 / 0.047 / 0.043 | 100% | **0%** |

† The maximum is at (16.6, 134, 89.2) in, on the far wall, where only two cameras
see the point. Among points with 3 or more cameras the maximum is 0.53 mm.

**Coverage, not accuracy, is the main finding about the volume.** Nineteen
percent of the volume cannot be triangulated at all. That gap is the region above
about 90 in. Every camera is aimed at z = 30 in, so the top of the room falls
outside every frustum:

| Height | 2+ cams | 3+ cams |
|---|---|---|
| Up to 71 in | 100% | 100% |
| 83 in | 100% | 90% |
| 89 in | 96% | 69% |
| 95 in | 59% | 34% |
| 101 in | 36% | 5% |
| 107 in | 4% | 0% |
| 113 in and above | 0% | 0% |

The perch at z = 50 in sits well inside the fully covered region. 96% of the
points in its box are seen by all ten cameras, and none by fewer than eight.

### Axis split: two camera heights are what balance z

On the perch box, the median RMS error per axis (mm) is:

| Cameras | x | y | z |
|---|---|---|---|
| All 10 | 0.043 | 0.047 | 0.043 |
| Upper 6 only | 0.062 | 0.064 | **0.105** |
| Lower 4 only | 0.059 | **0.075** | 0.048 |

The ceiling cameras look steeply down, so depth along their rays is mostly
vertical, and z is their weak axis. The lower ring sees the perch nearly
side-on and pins z, but it is weak along y. Together the two rings bring all
three axes to the same level. Neither ring does that alone.

### Camera subsets (perch box, mm)

| Subset | Median | p90 | Under 3 cams |
|---|---|---|---|
| All 10 | 0.077 | 0.089 | 0% |
| Upper 6 | 0.139 | 0.163 | 0% |
| Lower 4 | 0.106 | 0.137 | 0.5% |
| Random 6: ABEGHJ | 0.101 | 0.118 | 0% |
| Random 6: AFGHIJ | 0.139 | 0.158 | 0% |
| Random 6: ADGHIJ | 0.138 | 0.155 | 0% |
| Random 6: ABDEGH | 0.106 | 0.120 | 0% |

Losing four cameras to occlusion costs at most about 1.8× on the perch box, and
the box keeps 3 or more observers throughout. The worst sixes are the ones that
keep only one lower camera: AFGHIJ, ADGHIJ, and the all-upper six. That matches
the axis split above.

All ten earn their place. Every subset of six is clearly worse, and which
cameras are lost matters more than how many.

Subset volume statistics are computed over the points that each subset can
triangulate, so a subset with less coverage drops its hardest points. Read the
coverage columns in the JSON alongside the error numbers.

### Sensitivity to tape-measure error: ROBUST

The study ran 20 perturbed layouts. In each one, every camera moved by up to
±2 in per axis, its aim tilted by up to 2°, and its roll changed by up to ±2°.
Each layout is triangulated as if it were perfectly calibrated:

| Metric | Nominal | Min to max over layouts | Spread |
|---|---|---|---|
| Perch median | 0.076 | 0.075 to 0.077 | 2.8% |
| Perch p90 | 0.089 | 0.089 to 0.091 | 2.5% |
| Volume median | 0.098 | 0.098 to 0.099 | 1.4% |
| Volume p90 | 0.195 | 0.187 to 0.205 | 9.3% |
| Volume share with 2+ cams | 80.7% | 79.6% to 81.6% | 2.4% |
| Perch points under 3 cams | 0% | 0% to 0% | none |

Single-point maxima move more, up to 21%, as is expected for a worst case. The
headline numbers barely move, and no plausible layout leaves the perch short of
cameras. **Tape-measure estimates are good enough for this conclusion.**

## Verified vs assumed

**Verified, by the self-test and by internal consistency:**
- The triangulation recovers truth exactly with no noise.
- The projection matches OpenCV.
- The Jacobian matches finite differences.
- Undistortion inverts distortion.
- Refinement never ends worse than its DLT seed.
- The Monte Carlo results agree with first-order theory.
- The layout loads in inches and is used in mm, converted in one place.
- The aim point projects to the principal point on every camera.

**Assumed, not measured:**
- **Distortion coefficients** are plausible placeholders. They were chosen so
  the field of view matches the spec, but they were never fitted.
- **All ten cameras share the intrinsics of camera 33661.**
- **Noise** is static centroid jitter (0.054 px) and is the same at every range.
  Real markers shrink with distance, and moving markers blur, so far-field and
  in-flight noise will be higher.
- **Visibility is geometric only.** The study models no occlusion by the drone,
  the perch or people, no marker size limit, and no IR falloff with range.
- **Calibration is perfect.** This is the largest assumption, and it is why
  these results are a floor.

Full provenance is in `accuracy_study.json`: the layout in inches, the study
config, config SHA-256 hashes, the intrinsics, library versions, the git commit
and dirty flag, the timestamp, and the assumptions above.
