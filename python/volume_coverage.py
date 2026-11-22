"""
Volume coverage for the PANOPTES rig: how much of the capture volume the
cameras can see, and how well placed they are to triangulate it.

Pure geometry. No pixel noise, no Monte Carlo, no triangulation: every number
is computed once per voxel from the camera poses and the camera model. That
is what makes a fine grid cheap. (accuracy_study.py answers a different
question, millimetres of error under noise, and needs the Monte Carlo.)

Per voxel centre X:
  count       cameras for which rig.visible(X) holds (in front, on the
              sensor, inside the true field of view; rig.py)
  pair angle  for every pair of cameras that both see X, the angle at X
              between the rays to the two camera centres, raw in [0, 180]
  effective   min(raw, 180 - raw), in [0, 90]. Depth precision from a pair
              goes as sin(raw), so 120 deg is as good as 60 deg and two
              cameras facing each other (180 deg) are as bad as 0 deg.
  best        the largest effective angle over the pairs (NaN under 2 cameras)
  median      the median effective angle over the pairs
  seen        count >= min_cameras_seen                       (coverage.yaml)
  good        seen and best >= min_angle_deg                  (coverage.yaml)

Camera poses come through load_rig(), the one place to change when poses
come from extrinsics instead of layout.yaml's positions and aim point.

Usage: python volume_coverage.py [--self-test] [--allow-unmeasured]
                                 [layout.yaml] [coverage.yaml]

A layout marked `measured: 0` is refused, as accuracy_study.py refuses it:
its positions, volume, aim and perch are stale. --allow-unmeasured runs
anyway, warns, and records `layout_measured: false` in the JSON provenance.
--self-test runs the checks only and exits 0 if all pass; a full run runs
them first and refuses to report numbers if any fails.

Outputs into analysis/coverage/ (coverage.yaml output.dir):
volume_coverage.json, volume_coverage.npz, slice PNGs, slices_overview.png
"""

import os, sys, json, time, hashlib, platform, subprocess, datetime, argparse
import numpy as np
import cv2

import rig as rigmod
from rig import MM_PER_IN
from accuracy_study import panel, titled, hcat

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "..", "config")
DEFAULT_LAYOUT = os.path.join(CONFIG, "layout.yaml")
DEFAULT_COVERAGE = os.path.join(CONFIG, "coverage.yaml")


# ---------------------------------------------------------------- loading

def load_rig(layout_path, quiet=True):
    """
    -> (rig.Rig, info). THE camera-pose boundary.

    Today the poses are layout.yaml's positions, each looking at its aim
    point (rig.load_layout). Once extrinsics exist, a reader for their output
    returns rig.Rig(names, serials, centres_mm, R_world_to_camera, model)
    here, plus an info dict with the same keys (volume_min/max, perch,
    measured, raw), and nothing below this function changes.
    """
    return rigmod.load_layout(layout_path, quiet=quiet)


def load_config(path):
    """coverage.yaml -> dict, lengths in mm. Raises ValueError on a bad value."""
    y = rigmod.read_yaml(path)
    cfg = {
        "spacing_mm": float(y["grid"]["spacing_in"]) * MM_PER_IN,
        "chunk": int(y["grid"]["chunk_points"]),
        "min_seen": int(y["min_cameras_seen"]),
        "min_angle_deg": float(y["min_angle_deg"]),
        "slices_z_mm": [float(z) * MM_PER_IN for z in y["slices"]["z_in"]],
        "perch_plane": y["slices"]["perch_plane"],
        "out_dir": os.path.normpath(os.path.join(os.path.dirname(path), y["output"]["dir"])),
        "raw": y,
    }
    if cfg["spacing_mm"] <= 0 or cfg["chunk"] < 1:
        raise ValueError(f"{path}: grid.spacing_in and grid.chunk_points must be positive")
    if cfg["min_seen"] < 2:
        raise ValueError(f"{path}: min_cameras_seen must be at least 2")
    if not 0 <= cfg["min_angle_deg"] <= 90:
        raise ValueError(f"{path}: min_angle_deg is an effective angle, 0..90")
    if cfg["perch_plane"] not in ("x", "y", "z"):
        raise ValueError(f"{path}: slices.perch_plane must be \"x\", \"y\" or \"z\"")
    return cfg


# ---------------------------------------------------------------- grid

def voxel_centres(lo, hi, spacing):
    """
    Centres of an even voxel grid filling [lo, hi], voxel edge <= spacing on
    every axis. Centres, not corners, so every voxel stands for the same
    volume and a fraction of voxels is a fraction of the volume.
    An axis with lo == hi (a slice plane) gets the single value lo.
    """
    axes = []
    for a, b in zip(lo, hi):
        if b > a:
            n = max(1, int(np.ceil((b - a) / spacing - 1e-9)))
            axes.append(a + (np.arange(n) + 0.5) * (b - a) / n)
        else:
            axes.append(np.array([a], float))
    pts = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
    return pts, axes


# ---------------------------------------------------------------- geometry

def _coverage_block(rig, pts):
    vis = rig.visible(pts)                                       # N x C
    count = vis.sum(1)
    u = rig.C[None] - pts[:, None]                               # N x C x 3, point -> camera
    u /= np.maximum(np.linalg.norm(u, axis=-1, keepdims=True), 1e-9)
    i, j = np.triu_indices(len(rig.names), 1)                    # every camera pair once
    cos = np.einsum("npk,npk->np", u[:, i], u[:, j])
    raw = np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))         # N x P, 0..180
    eff = np.minimum(raw, 180.0 - raw)                           # 0..90
    both = vis[:, i] & vis[:, j]
    n = np.arange(len(pts))
    k = np.where(both, eff, -1.0).argmax(1)                      # best pair per point
    ok = count >= 2
    best = np.where(ok, eff[n, k], np.nan)
    best_raw = np.where(ok, raw[n, k], np.nan)
    med = np.full(len(pts), np.nan)
    if ok.any():
        med[ok] = np.nanmedian(np.where(both[ok], eff[ok], np.nan), 1)
    return vis, count, best, best_raw, med


def coverage(rig, pts, chunk=100000):
    """
    Per point: visible (N,C), count, best/median effective angle and the raw
    angle of the best pair (deg; NaN under 2 cameras). Batched in `chunk`
    points so memory stays bounded at any grid spacing.
    """
    parts = [_coverage_block(rig, pts[s:s + chunk]) for s in range(0, len(pts), chunk)]
    keys = ("visible", "count", "best_eff_deg", "best_raw_deg", "median_eff_deg")
    return {k: np.concatenate([p[i] for p in parts]) for i, k in enumerate(keys)}


def flags(res, cfg):
    seen = res["count"] >= cfg["min_seen"]
    with np.errstate(invalid="ignore"):
        good = seen & (res["best_eff_deg"] >= cfg["min_angle_deg"])
    return seen, good


# ---------------------------------------------------------------- summary

def _pct(a):
    if a.size == 0:
        return None
    return {f"p{p}": float(np.percentile(a, p)) for p in (5, 25, 50, 75, 95)}


def summarise(res, cfg, names):
    n, best, med = res["count"], res["best_eff_deg"], res["median_eff_deg"]
    seen, good = flags(res, cfg)
    ge2 = n >= 2
    hist = np.bincount(n, minlength=len(names) + 1)
    return {
        "voxels": int(len(n)),
        "count_histogram": hist.tolist(),                       # index = cameras
        "frac_count_0": float((n == 0).mean()),
        "frac_count_1": float((n == 1).mean()),
        "frac_count_2": float((n == 2).mean()),
        "frac_count_3plus": float((n >= 3).mean()),
        "frac_seen": float(seen.mean()),                        # >= min_cameras_seen
        "frac_good": float(good.mean()),                        # seen and angle ok
        "frac_seen_poor_angle": float((seen & ~good).mean()),
        "best_angle_deg_over_ge2": _pct(best[ge2]),
        "best_angle_deg_over_seen": _pct(best[seen]),
        "median_angle_deg_over_ge2": _pct(med[ge2]),
        "per_camera_frac_visible": {nm: float(res["visible"][:, c].mean())
                                    for c, nm in enumerate(names)},
    }


# ---------------------------------------------------------------- provenance

def provenance(paths, cfg, info, rig):
    def git(*a):
        try:
            return subprocess.run(["git", *a], cwd=HERE, capture_output=True, text=True).stdout.strip()
        except OSError:
            return None
    return {
        "timestamp": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "git_commit": git("rev-parse", "HEAD"),
        "git_dirty": bool(git("status", "--porcelain", "--", ":/python",
                              ":/config/layout.yaml", ":/config/coverage.yaml")),
        "config_sha256": {os.path.basename(p): hashlib.sha256(open(p, "rb").read()).hexdigest()
                          for p in paths},
        "versions": {"python": platform.python_version(), "numpy": np.__version__,
                     "opencv": cv2.__version__, "platform": platform.platform()},
        "layout_measured": info["measured"],
        "layout_inches": info["raw"],
        "coverage_config": cfg["raw"],
        "units_internal": "mm",
        "intrinsics": {k: rig.model[k] for k in ("fx", "fy", "cx", "cy", "dist", "resolution")},
        "assumed": {
            "poses": "layout.yaml positions, each camera looking at the aim point",
            "intrinsics": "one shared model for all cameras (layout.yaml camera_model)",
            "visibility": "geometric only: on sensor, in front, inside FOV. No occlusion, "
                          "no detection range, no marker size or brightness",
            "angle": "effective angle min(raw, 180 - raw) at the voxel centre",
        },
    }


# ---------------------------------------------------------------- heat maps

def slice_maps(rig, info, cfg):
    """One PNG per plane (camera count | best angle) plus an overview."""
    lo, hi, sp = info["volume_min"], info["volume_max"], cfg["spacing_mm"]
    pa = "xyz".index(cfg["perch_plane"])
    planes = [("z", z) for z in cfg["slices_z_mm"]] + [(cfg["perch_plane"], info["perch"][pa])]
    stale = "" if info["measured"] else "  [STALE LAYOUT, measured: 0]"
    files, overview, rows = [], [], []
    for axis, val in planes:
        a = "xyz".index(axis)
        l2, h2 = lo.copy(), hi.copy()
        l2[a] = h2[a] = val
        pts, axes = voxel_centres(l2, h2, sp)
        r = coverage(rig, pts, cfg["chunk"])
        seen, good = flags(r, cfg)
        keep = [i for i in range(3) if i != a]
        h_ax, v_ax = axes[keep[0]], axes[keep[1]]
        shape = (len(h_ax), len(v_ax))
        n, best = r["count"].reshape(shape), r["best_eff_deg"].reshape(shape)
        marks = [(c[keep[0]], c[keep[1]], nm) for c, nm in zip(rig.C, rig.names)]
        p = info["perch"]
        box = (((p[keep[0]], p[keep[1]]),) * 3) if abs(p[a] - val) <= sp else None
        labels = (f"{'xyz'[keep[0]]} (in)", f"{'xyz'[keep[1]]} (in)")
        pc = panel(best, n, h_ax, v_ax, "count", 90.0, cfg["min_seen"], marks, box, labels)
        pg = panel(best, n, h_ax, v_ax, "error", 90.0, cfg["min_seen"], marks, box, labels,
                   unit="best angle deg")
        name = f"{axis} = {val / MM_PER_IN:g} in"
        sub = (f"{seen.mean():.0%} seen by >= {cfg['min_seen']} cameras; {good.mean():.0%} also "
               f"with a pair >= {cfg['min_angle_deg']:g} deg. Angle = min(t, 180 - t). + = perch.")
        img = titled(hcat([pc, pg]), f"Camera coverage and best triangulation angle, {name}{stale}", sub)
        fn = f"slice_{axis}{val / MM_PER_IN:g}".replace(".", "p") + "in.png"
        cv2.imwrite(os.path.join(cfg["out_dir"], fn), img)
        files.append(fn)
        rows.append({"plane": name, "frac_seen": float(seen.mean()), "frac_good": float(good.mean()),
                     "file": fn})
        if axis == "z":
            overview.append(titled(pc, name))
    if overview:
        ov = titled(hcat(overview), f"Cameras seeing each point, horizontal slices{stale}",
                    f"all {len(rig.names)} cameras; outlined = under {cfg['min_seen']} cameras")
        cv2.imwrite(os.path.join(cfg["out_dir"], "slices_overview.png"), ov)
        files.append("slices_overview.png")
    return files, rows


# ---------------------------------------------------------------- self-test

def _ring(model, angles_deg, dist=3000.0, z=0.0):
    """Synthetic rig: cameras on a horizontal circle round the origin, all looking at it."""
    up = np.array([0, 0, 1.0])
    a = np.radians(angles_deg)
    centres = np.stack([dist * np.cos(a), dist * np.sin(a), np.full_like(a, z)], 1)
    R = np.stack([rigmod.look_at(c, np.zeros(3), up) for c in centres])
    names = [f"T{k}" for k in range(len(a))]
    return rigmod.Rig(names, list(range(len(a))), centres, R, model)


def _slow(rig, X):
    """Reference: one point, a plain loop over camera pairs."""
    vis = rig.visible(X[None])[0]
    idx = np.flatnonzero(vis)
    best, best_raw, effs = -1.0, np.nan, []
    for p in range(len(idx)):
        for q in range(p + 1, len(idx)):
            u = rig.C[idx[p]] - X
            v = rig.C[idx[q]] - X
            c = u @ v / (np.linalg.norm(u) * np.linalg.norm(v))
            raw = np.degrees(np.arccos(min(1.0, max(-1.0, c))))
            eff = min(raw, 180 - raw)
            effs.append(eff)
            if eff > best:
                best, best_raw = eff, raw
    if len(idx) < 2:
        return len(idx), np.nan, np.nan, np.nan
    return len(idx), best, best_raw, float(np.median(effs))


def self_test(layout_path=DEFAULT_LAYOUT, verbose=True):
    """Returns True if every check passes. Prints one line per check."""
    rig, info = load_rig(layout_path, quiet=True)
    if verbose and not info["measured"]:
        print("  note: layout is unmeasured (stale pre-remount geometry). These checks test "
              "the maths\n        and hold regardless; they say nothing about the rig as mounted.")
    model = rig.model
    results = []

    def check(name, value, limit, unit=""):
        ok = bool(value < limit)                     # NaN fails
        results.append(ok)
        if verbose:
            print(f"  [{'PASS' if ok else 'FAIL'}] {name:<58} {value:10.3g} {unit:<3} (limit {limit:g})")

    origin = np.zeros((1, 3))

    def one(r):
        res = coverage(r, origin)
        return (int(res["count"][0]), res["best_eff_deg"][0], res["best_raw_deg"][0],
                res["median_eff_deg"][0])

    # 1-3. Two cameras on a circle round the point: the angle at the point is
    # the angle between them on the circle, by construction.
    n, best, raw, _ = one(_ring(model, [0, 60]))
    check("2 cams 60 deg apart: count 2, raw 60, effective 60",
          abs(n - 2) + abs(raw - 60) + abs(best - 60), 1e-9, "deg")
    n, best, raw, _ = one(_ring(model, [0, 120]))
    check("2 cams 120 deg apart: raw 120, effective 60 (same as 60)",
          abs(n - 2) + abs(raw - 120) + abs(best - 60), 1e-9, "deg")
    n, best, raw, _ = one(_ring(model, [0, 180]))
    check("2 cams facing each other: raw 180, effective 0",
          abs(n - 2) + abs(raw - 180) + abs(best - 0), 1e-6, "deg")

    # 4. Three cameras at 0, 40, 100 deg: raw pairs 40, 100, 60 -> effective
    # 40, 80, 60 -> best 80 (the 100 deg pair, raw 100), median 60.
    n, best, raw, med = one(_ring(model, [0, 40, 100]))
    check("3 cams 0/40/100 deg: best 80 (raw 100), median 60",
          abs(n - 3) + abs(best - 80) + abs(raw - 100) + abs(med - 60), 1e-9, "deg")

    # 5. Every camera looks +x from the x = 0 plane; a point at x < 0 is behind all.
    up = np.array([0, 0, 1.0])
    cs = np.array([[0, 0, 0], [0, 500, 0], [0, 0, 500], [0, -500, 300.0]])
    R = np.stack([rigmod.look_at(c, c + [3000, 0, 0], up) for c in cs])
    back = rigmod.Rig(["a", "b", "c", "d"], [0, 1, 2, 3], cs, R, model)
    res = coverage(back, np.array([[-1000.0, 0, 0]]))
    check("point behind every camera: count 0 (+1 if angle not NaN)",
          res["count"][0] + (0 if np.isnan(res["best_eff_deg"][0]) else 1), 0.5)

    # 6. The layout's aim point is on every optical axis: all cameras see it.
    res = coverage(rig, info["aim"][None])
    check(f"layout aim point seen by all {len(rig.names)} cameras (shortfall)",
          len(rig.names) - res["count"][0], 0.5)

    # 7. Vectorised maths agrees with a plain per-point loop.
    rng = np.random.default_rng(1)
    pts = rng.uniform(info["volume_min"], info["volume_max"], (500, 3))
    fast = coverage(rig, pts)
    worst = 0.0
    for k, X in enumerate(pts):
        n, b, r, m = _slow(rig, X)
        if n != fast["count"][k]:
            worst = np.inf
            break
        if n >= 2:
            worst = max(worst, abs(b - fast["best_eff_deg"][k]), abs(r - fast["best_raw_deg"][k]),
                        abs(m - fast["median_eff_deg"][k]))
    ge2 = int((fast["count"] >= 2).sum())
    check(f"vectorised vs per-point loop, 500 pts ({ge2} with 2+ cams)", worst, 1e-9, "deg")

    # 8. Batching does not change any result.
    small = coverage(rig, pts, chunk=37)
    same = all(np.array_equal(fast[k], small[k], equal_nan=k != "visible" and k != "count")
               for k in fast)
    check("chunk size 37 vs one batch: identical (count of mismatches)", 0 if same else 1, 0.5)

    # 9. Voxel grid: centres strictly inside, edge <= spacing, even split.
    lo, hi, sp = info["volume_min"], info["volume_max"], 2 * MM_PER_IN
    g, axes = voxel_centres(lo, hi, sp)
    edges = [ax[1] - ax[0] for ax in axes]
    bad = (int((g <= lo).any() or (g >= hi).any())
           + sum(e > sp + 1e-9 for e in edges)
           + int(len(g) != np.prod([len(a) for a in axes]))
           + sum(abs((ax[0] - l) * 2 - e) > 1e-9 for ax, l, e in zip(axes, lo, edges)))
    check("voxel centres inside volume, edge <= spacing, half-edge margin", bad, 0.5)

    passed = all(results)
    if verbose:
        print(f"  self-test: {sum(results)}/{len(results)} passed")
    return passed


# ---------------------------------------------------------------- main

def main(argv=None):
    ap = argparse.ArgumentParser(description="Camera coverage and triangulation angle over the volume.")
    ap.add_argument("layout", nargs="?", default=DEFAULT_LAYOUT, help="default config/layout.yaml")
    ap.add_argument("coverage", nargs="?", default=DEFAULT_COVERAGE, help="default config/coverage.yaml")
    ap.add_argument("--self-test", action="store_true", help="run the checks only; exit 0 if all pass")
    ap.add_argument("--allow-unmeasured", action="store_true",
                    help="run on a layout with measured: 0; warns and records layout_measured false")
    a = ap.parse_args(argv)

    if a.self_test:
        print("volume_coverage.py self-test")
        return 0 if self_test(a.layout) else 1

    t0 = time.time()
    # Before the self-test and before anything is written.
    _, info0 = load_rig(a.layout)
    if not info0["measured"]:
        if not a.allow_unmeasured:
            sys.exit(f"refusing to run: {a.layout}: {rigmod.UNMEASURED_NOTE}\n"
                     "Re-measure the rig (or wait for extrinsics) and set `measured: 1`, or pass "
                     "--allow-unmeasured to run on the stale geometry.")
        print(f"WARNING: --allow-unmeasured: {rigmod.UNMEASURED_NOTE}", file=sys.stderr)

    print("=" * 78)
    print("PANOPTES volume coverage")
    print("=" * 78)
    print("\nStep 0: self-test")
    if not self_test(a.layout):
        sys.exit("self-test failed: refusing to report numbers")

    rig, info = load_rig(a.layout)
    cfg = load_config(a.coverage)
    os.makedirs(cfg["out_dir"], exist_ok=True)
    C = len(rig.names)
    vol_in = (info["volume_max"] - info["volume_min"]) / MM_PER_IN
    print(f"\nRig: {C} cameras; volume {' x '.join(f'{v:g}' for v in vol_in)} in; "
          f"seen = >= {cfg['min_seen']} cameras; good = seen and a pair >= {cfg['min_angle_deg']:g} deg")

    pts, axes = voxel_centres(info["volume_min"], info["volume_max"], cfg["spacing_mm"])
    edge = [(ax[1] - ax[0]) / MM_PER_IN if len(ax) > 1 else 0 for ax in axes]
    print(f"Grid: {' x '.join(str(len(ax)) for ax in axes)} = {len(pts)} voxels, "
          f"edge {' x '.join(f'{e:.3g}' for e in edge)} in (<= {cfg['spacing_mm'] / MM_PER_IN:g})")

    print("\nStep 1: coverage over the volume ...", flush=True)
    t1 = time.time()
    res = coverage(rig, pts, cfg["chunk"])
    print(f"  {len(pts)} voxels in {time.time() - t1:.2f} s")
    S = summarise(res, cfg, rig.names)
    print(f"\n  cameras per voxel   0: {S['frac_count_0']:.1%}   1: {S['frac_count_1']:.1%}   "
          f"2: {S['frac_count_2']:.1%}   3+: {S['frac_count_3plus']:.1%}")
    print(f"  seen (>= {cfg['min_seen']} cams):  {S['frac_seen']:.1%} of the volume")
    print(f"  good (seen, angle >= {cfg['min_angle_deg']:g}):  {S['frac_good']:.1%}   "
          f"(seen but poor angle: {S['frac_seen_poor_angle']:.1%})")
    print("  histogram: " + "  ".join(f"{k}:{v}" for k, v in enumerate(S["count_histogram"]) if v))
    b = S["best_angle_deg_over_seen"]
    if b:
        print("  best effective angle over seen voxels (deg): "
              + "  ".join(f"{k} {v:.1f}" for k, v in b.items()))
    print("  per camera, share of volume visible: "
          + "  ".join(f"{k} {v:.0%}" for k, v in S["per_camera_frac_visible"].items()))

    # By height: where coverage ends.
    seen, good = flags(res, cfg)
    print(f"\n  by height  {'>=2 cams':>9}{'seen':>8}{'good':>8}{'best med':>10}")
    layers = []
    z = pts[:, 2]
    for zv in axes[2]:
        m = z == zv
        sm = m & seen
        med = float(np.median(res["best_eff_deg"][sm])) if sm.any() else None
        layers.append({"z_in": round(zv / MM_PER_IN, 2), "frac_ge2": float((res["count"][m] >= 2).mean()),
                       "frac_seen": float(seen[m].mean()), "frac_good": float(good[m].mean()),
                       "best_angle_median_deg": med})
        L = layers[-1]
        print(f"  {L['z_in']:6.1f} in {L['frac_ge2']:>9.0%}{L['frac_seen']:>8.0%}{L['frac_good']:>8.0%}"
              f"{'-' if med is None else format(med, '.1f'):>10}")
    S["by_height"] = layers

    pr = coverage(rig, info["perch"][None])
    S["perch"] = {"at_in": (info["perch"] / MM_PER_IN).round(2).tolist(), "count": int(pr["count"][0]),
                  "best_eff_deg": float(pr["best_eff_deg"][0]),
                  "cameras": [nm for nm, v in zip(rig.names, pr["visible"][0]) if v]}
    print(f"\n  perch {S['perch']['at_in']} in: {S['perch']['count']} cameras "
          f"({''.join(S['perch']['cameras'])}), best angle {S['perch']['best_eff_deg']:.1f} deg")

    print("\nStep 2: heat-map slices ...", flush=True)
    files, rows = slice_maps(rig, info, cfg)
    for r in rows:
        print(f"  {r['plane']:<14} seen {r['frac_seen']:>4.0%}  good {r['frac_good']:>4.0%}  {r['file']}")
    S["slices"] = rows

    shape = tuple(len(ax) for ax in axes)
    doc = {"provenance": provenance([a.layout, a.coverage], cfg, info, rig),
           "results": S, "runtime_s": round(time.time() - t0, 2)}
    jpath = os.path.join(cfg["out_dir"], "volume_coverage.json")
    with open(jpath, "w") as f:
        json.dump(doc, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    np.savez_compressed(
        os.path.join(cfg["out_dir"], "volume_coverage.npz"),
        x_mm=axes[0], y_mm=axes[1], z_mm=axes[2],
        count=res["count"].reshape(shape).astype(np.int8),
        visible=res["visible"].reshape(shape + (C,)),
        best_eff_deg=res["best_eff_deg"].reshape(shape).astype(np.float32),
        best_raw_deg=res["best_raw_deg"].reshape(shape).astype(np.float32),
        median_eff_deg=res["median_eff_deg"].reshape(shape).astype(np.float32),
        seen=seen.reshape(shape), good=good.reshape(shape),
        camera_names=np.array(rig.names), camera_serials=np.array(rig.serials),
        camera_centres_mm=rig.C, min_cameras_seen=cfg["min_seen"],
        min_angle_deg=cfg["min_angle_deg"], layout_measured=info["measured"])
    print(f"\nWrote {jpath} (+ .npz grids, {len(files)} PNGs) in {time.time() - t0:.1f} s")
    if not info["measured"]:
        print(f"\nWARNING: {rigmod.UNMEASURED_NOTE}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
