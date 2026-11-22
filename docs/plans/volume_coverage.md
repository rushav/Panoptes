# Plan: volume coverage

Owner: Dom. Reviewer: Rushav. Branch: `feat/volume-coverage`. Status: Phase 1 built 2026-11-21 (uncommitted); tested on the stale layout only.

## Goal

How much of the room can the cameras see?

1. Split the capture volume into small cubes (voxels).
2. For each voxel, count the cameras that can see its centre.
3. A voxel counts as **seen** if **3 or more** cameras see it.
4. For each voxel, also measure the **triangulation angle**: the angle between
   the lines from the voxel to two cameras that see it. Wide angles give good
   depth; angles near 0° (or near 180°, cameras facing each other) give poor
   depth.

Output: the percentage of the room that is seen, where the gaps are (heatmap
slices), and the triangulation angle across the room.

## Decisions (Dom, 2026-11-21)

- **Test on the old numbers first.** Run on the stale `layout.yaml` with
  `--allow-unmeasured` to prove the tool works. When tape measurements come
  in, edit `layout.yaml`, set `measured: 1`, and rerun the same command; no
  code changes.
- **Seen = 3 or more cameras.** Configurable (`min_cameras_seen`).
- **Voxel size is configurable**, `grid.spacing_in`, starting at 2 in.
- **Angle threshold is configurable**, `min_angle_deg`, to be tuned later.
- **Effective angle = `min(θ, 180° − θ)`**, range 0–90°. Depth precision
  from a camera pair goes as sin θ, and sin 120° = sin 60°, so a 120° pair is
  exactly as good as a 60° pair; 180° (cameras facing each other) is as bad
  as 0°. The raw angle is kept in the `.npz` as well.
- **Python environment:** create `.venv` in the repo per RUNBOOK §2.

## What already exists

| Piece | Where | Status |
|---|---|---|
| Camera positions, aim point, volume size | `config/layout.yaml` | Exists, but **stale**: pre-remount estimates, `measured: 0`. Dom will tape-measure and update this file; the tool just reruns. |
| Camera model (lens, field of view) | `config/layout.yaml` `camera_model`, `python/rig.py` | Exists. One shared model (33661) for all ten. |
| "Can camera *c* see point *X*?" | `rig.Rig.visible()` | Exists: in front of the camera, on the sensor, inside the true field of view. |
| Cameras per point, "< 3 cameras" flag | `python/accuracy_study.py` | Exists, but bundled with a slow Monte Carlo error study on a coarse 6 in grid, and it refuses to run on the stale layout. |
| Heatmap drawing | `accuracy_study.py` `panel()` etc. | Exists; reused. |
| Triangulation angle | — | **Does not exist.** New. |
| Seen/not-seen summary of the whole room at a fine grid | — | **Does not exist.** New. |

So the camera geometry and the visibility test are done. What's new is a
fast, geometry-only tool that runs them on a fine grid and adds the angle.

## Phase 1: build now

**New files**

- `python/volume_coverage.py`. Named `volume_coverage`, not `coverage`:
  `lib/coverage.*` is the intrinsics capture grid, something else.
- `config/coverage.yaml`: settings, in inches like `layout.yaml`.

**Steps**

1. **Load cameras** with `rig.load_layout()` (inches → mm happens there,
   once). Wrap it in one small function, `load_rig()`, so that after
   extrinsics exist only that function changes.
2. **Refuse on a stale layout** unless `--allow-unmeasured` is given, the same
   way `accuracy_study.py` does, and record `layout_measured` in the output.
3. **Make the grid**: voxel centres at `grid.spacing_in` (default 2 in) over
   the volume in `layout.yaml`.
4. **Count cameras** per voxel with `rig.visible()`. Process in chunks so a
   fine grid fits in memory.
5. **Triangulation angle** per voxel, over every pair of cameras that see it
   (at most 45 pairs):
   - angle between the two lines voxel → camera centre;
   - **effective angle** = `min(θ, 180° − θ)`, because 180° (cameras facing
     each other) is as bad for depth as 0°;
   - keep the **best** (max effective) and the **median** per voxel.
6. **Flags**, thresholds from `config/coverage.yaml`:
   - `seen` = count ≥ `min_cameras_seen` (3);
   - `good_angle` = best effective angle ≥ `min_angle_deg` (default to be
     agreed with Rushav).
7. **Outputs** to `analysis/coverage/`:
   - `volume_coverage.json`: % of the volume seen by 0, 1, 2, 3+ cameras;
     % seen; % seen with a good angle; angle percentiles; a by-height table;
     provenance (git commit, hash of `layout.yaml` and `coverage.yaml`,
     `layout_measured`).
   - `volume_coverage.npz`: the full per-voxel grids.
   - PNG heatmaps: horizontal slices at configured heights plus one vertical
     slice through the perch; each with two panels, camera count and best
     angle.
8. **Self-test** (`--self-test`):
   - two cameras whose lines meet at a known 60° → angle 60°;
   - cameras facing each other → effective angle 0°;
   - a point behind every camera → count 0;
   - the layout's aim point → seen by all 10;
   - the fast vectorised angles match a slow per-point loop;
   - results don't depend on chunk size.

   A full run won't produce numbers if any check fails.

**Config sketch** (`config/coverage.yaml`)

```yaml
%YAML:1.0
grid:
  spacing_in: 2.0
min_cameras_seen: 3        # a voxel is "seen" at this many cameras
min_angle_deg: 15          # placeholder; agree with Rushav
slices:
  z_in: [12, 30, 51.5, 80, 110]   # re-choose once the rig is measured
  perch_plane: y
chunk_points: 200000
output:
  dir: "../analysis/coverage"
```

**Docs touched in the same change:** RUNBOOK §1.9 and §10 (command and
expected output), ARCHITECTURE (new section on the tool), FINDINGS Open
items (#29 notes this tool also needs the measured layout; a new item for
Phase 3). `.gitignore` gains `/analysis/coverage/*.png`.

**Done when:** `--self-test` passes, and a full run with `--allow-unmeasured`
prints the summary and writes the files. Then, when Dom updates
`layout.yaml` with tape measurements and sets `measured: 1`, it runs with no
flag.

## Phase 2: optional realism (sketch, confirm with Rushav)

- Maximum detection range per camera.
- Box-shaped obstacles in the room, listed in config.
- Each camera's own lens model from `config/intrinsics/<serial>.yaml`
  instead of the shared one. Needs a change to `rig.py`, reviewed on its own.

## Phase 3: check against reality (sketch, after extrinsics)

Walk one marker through the room while `src/08_sync` records. Triangulate
each frame with the real camera poses, look up the predicted camera count for
that voxel, and compare with how many cameras actually saw the marker.
Report the % that match and where they disagree.

Limitation: a frame seen by fewer than 2 cameras can't be placed in 3D, so
blind spots can't be checked directly. Options: fill gaps from the frames
before and after; place the marker at a few tape-measured spots in the
predicted blind areas.

## Questions for Rushav

Settled by Dom (see Decisions): Python env, angle threshold (configurable,
starts at 15°), 3 cameras for "seen".

1. Python 3.14 is this machine's system Python; RUNBOOK §2 and the pins
   assume 3.12. Does `python/requirements.txt` install on 3.14, or should
   the venv be built with 3.12?
2. Should `analysis/coverage/volume_coverage.json` be tracked in git?
3. OK to add one optional argument to `accuracy_study.py`'s `panel()` so its
   colour-bar label can read "deg" instead of "RMS mm"?
4. Which slice heights matter on the remounted rig (perch height, top of the
   flight path)?
