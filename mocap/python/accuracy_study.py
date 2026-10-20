"""
Synthetic triangulation accuracy study for the PANOPTES rig.

Predicts 3D accuracy across the capture volume from the mounted layout,
before the rig is calibrated: at every grid point, project into the cameras
that can actually see it, add measured centroid jitter, triangulate, compare
with truth, many times over.

WHAT THIS IS NOT. Every run assumes PERFECT calibration: the triangulator
knows the exact intrinsics and extrinsics that generated the data. Real
extrinsics carry their own error, which adds directly to these numbers. The
result is a FLOOR on achievable error, not a forecast of measured error.

Steps, in order:
  0. self-test (triangulate.py); abort if it fails
  1. error map over the volume grid, split by axis
  2. perch box on its own denser grid
  3. heat maps: horizontal slices plus the y = perch plane
  4. camera subsets: all, upper ring, lower ring, random sixes
  5. sensitivity: rerun on randomly perturbed layouts

Usage: python accuracy_study.py [layout.yaml] [study.yaml]
Outputs into mocap/analysis/: PNGs, accuracy_study.json, accuracy_study.npz
"""

import os
# One BLAS thread per worker; the parallelism is across processes.
for v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(v, "1")

import sys, json, time, hashlib, platform, subprocess, datetime
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import cv2

import rig as rigmod
from rig import MM_PER_IN
from triangulate import triangulate, predicted_cov, volume_frame, self_test

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "..", "config")

FLOOR_NOTE = ("These numbers assume PERFECT calibration (exact intrinsics and "
              "extrinsics). Real accuracy will be worse because calibrated "
              "extrinsics carry their own error. This is a floor, not a forecast. "
              "For scale: the previous rig measured 13.8 mm loop closure over a "
              "9 m circuit.")


# ---------------------------------------------------------------- sampling

def grid(lo, hi, spacing):
    """Even grid from lo to hi inclusive, spacing at most `spacing` per axis."""
    axes = [np.linspace(a, b, int(np.ceil((b - a) / spacing - 1e-9)) + 1) if b > a
            else np.array([a]) for a, b in zip(lo, hi)]
    pts = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
    return pts, axes


def perch_box(info, half):
    lo = np.maximum(info["perch"] - half, info["volume_min"])
    hi = np.minimum(info["perch"] + half, info["volume_max"])
    return lo, hi


# ---------------------------------------------------------------- Monte Carlo

def _chunk(args):
    """Worker: Monte Carlo for a block of points. Top level so it pickles."""
    rig, pts, vis, trials, sigma, seed, center, scale, iters = args
    rng = np.random.default_rng(seed)
    uv, _, _ = rig.project(pts)
    k, C = vis.shape
    obs = uv[:, None] + rng.normal(0, sigma, (k, trials, C, 2))
    mask = np.broadcast_to(vis[:, None], (k, trials, C)).reshape(-1, C)
    X, _, _ = triangulate(rig, obs.reshape(-1, C, 2), mask, center, scale, iters)
    err = X.reshape(k, trials, 3) - pts[:, None]
    return np.sqrt((err ** 2).mean(1))                 # per-axis RMS, (k,3)


def evaluate(pool, rig, pts, cams, trials, S, seed, frame):
    """
    Monte Carlo error at each point using only cameras in `cams` (bool, C).
    Returns per-point observer count, per-axis RMS (NaN where < 2 observers),
    total RMS and the first-order predicted total RMS.
    """
    vis = rig.visible(pts) & cams
    n_obs = vis.sum(1)
    rms_axis = np.full((len(pts), 3), np.nan)
    ok = np.flatnonzero(n_obs >= 2)
    per = max(1, S["refine"]["chunk_obs"] // (trials * len(rig.names)))
    blocks = [ok[i:i + per] for i in range(0, len(ok), per)]
    seeds = np.random.SeedSequence(seed).spawn(len(blocks))
    jobs = [(rig, pts[b], vis[b], trials, S["noise_px"], s, *frame, S["refine"]["iterations"])
            for b, s in zip(blocks, seeds)]
    for b, r in zip(blocks, pool.map(_chunk, jobs)):
        rms_axis[b] = r
    pred = np.full(len(pts), np.nan)
    pred[ok] = np.sqrt(np.trace(predicted_cov(rig, pts[ok], vis[ok], S["noise_px"]), axis1=1, axis2=2))
    return {"pts": pts, "n_obs": n_obs, "rms_axis": rms_axis,
            "rms": np.sqrt((rms_axis ** 2).sum(1)), "pred": pred}


def summarise(res, flag):
    """Headline numbers over the points that could be triangulated."""
    rms, n = res["rms"], res["n_obs"]
    ok = np.isfinite(rms)
    good = ok & (n >= flag)
    if not ok.any():
        return {"points": int(len(rms)), "triangulable_frac": 0.0}
    q = lambda a, p: float(np.percentile(a, p))
    worst = np.flatnonzero(ok)[np.argmax(rms[ok])]
    out = {
        "points": int(len(rms)),
        "triangulable_frac": float(ok.mean()),          # >= 2 observers
        "flagged_frac": float((n < flag).mean()),       # < min_observers_flag
        "observers_min": int(n.min()), "observers_median": float(np.median(n)),
        "median_mm": q(rms[ok], 50), "p90_mm": q(rms[ok], 90), "max_mm": float(rms[ok].max()),
        "max_at_in": (res["pts"][worst] / MM_PER_IN).round(1).tolist(),
        "max_at_observers": int(n[worst]),
        # Worst case is usually a 2-camera point at a wall; this is the
        # worst case once those are excluded.
        "max_mm_3plus": float(rms[good].max()) if good.any() else None,
        "axis_median_mm": np.median(res["rms_axis"][ok], 0).tolist(),
        "axis_p90_mm": np.percentile(res["rms_axis"][ok], 90, 0).tolist(),
        "mc_over_predicted_median": float(np.median(rms[ok] / res["pred"][ok])),
    }
    return out


# ---------------------------------------------------------------- printing

def table(title, rows, cols):
    """rows: list of (label, summary dict). cols: (header, key, fmt)."""
    print(f"\n{title}")
    head = f"  {'':<18}" + "".join(f"{h:>11}" for h, _, _ in cols)
    print(head + "\n  " + "-" * (len(head) - 2))
    for label, s in rows:
        cells = []
        for _, key, fmt in cols:
            v = key(s) if callable(key) else s.get(key)
            cells.append(f"{'-' if v is None else format(v, fmt):>11}")
        print(f"  {label:<18}" + "".join(cells))


HEADLINE = [("median", "median_mm", ".4f"), ("p90", "p90_mm", ".4f"),
            ("max", "max_mm", ".4f"), ("max(>=3)", "max_mm_3plus", ".4f"),
            ("x med", lambda s: s["axis_median_mm"][0], ".4f"),
            ("y med", lambda s: s["axis_median_mm"][1], ".4f"),
            ("z med", lambda s: s["axis_median_mm"][2], ".4f"),
            (">=2 cams", "triangulable_frac", ".1%"), ("<3 cams", "flagged_frac", ".1%"),
            ("MC/pred", "mc_over_predicted_median", ".3f")]


# ---------------------------------------------------------------- heat maps

def _hex(h):
    h = h.lstrip("#")
    return np.array([int(h[i:i + 2], 16) for i in (4, 2, 0)], float)   # BGR


def _ramp(hexes, n=256):
    c = np.stack([_hex(h) for h in hexes])
    s = np.linspace(0, len(c) - 1, n)
    i = np.minimum(s.astype(int), len(c) - 2)
    f = (s - i)[:, None]
    return (c[i] * (1 - f) + c[i + 1] * f).astype(np.uint8)


# Single-hue sequential ramps, light = low (dataviz skill reference palette).
BLUE = _ramp(["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"])
GREY = _ramp(["#ecebe7", "#c3c2b7", "#8a8980", "#52514e", "#2b2a28"])
SURFACE, INK, INK2 = _hex("#fcfcfb"), _hex("#0b0b0b"), _hex("#52514e")
FLAG = _hex("#eb6834")                                  # < min_observers outline
NONE = _hex("#e3e2de")                                  # < 2 observers, hatched
FONT = cv2.FONT_HERSHEY_SIMPLEX


def _text(img, s, xy, scale=0.42, color=INK, thick=1, anchor="l"):
    (w, h), _ = cv2.getTextSize(s, FONT, scale, thick)
    x, y = xy
    x = x - w if anchor == "r" else x - w // 2 if anchor == "c" else x
    cv2.putText(img, s, (int(x), int(y + h // 2)), FONT, scale, color.tolist(), thick, cv2.LINE_AA)


def _nice(v):
    e = 10 ** np.floor(np.log10(v))
    return next(m * e for m in (1, 2, 2.5, 5, 10) if m * e >= v)


def panel(values, n_obs, h_ax, v_ax, kind, vmax, flag, marks, box, labels):
    """
    One map panel. values/n_obs are (nh, nv) on the h/v axes (mm).
    kind 'error' colours by value with BLUE; 'count' by observers with GREY.
    Cells under 2 observers are hatched; the < flag region is outlined.
    """
    nh, nv = len(h_ax), len(v_ax)
    cell = max(3, int(round(600 / max(nv, nh))))
    if kind == "error":
        idx = np.clip(np.nan_to_num(values / vmax), 0, 1) * 255
        col = BLUE[idx.astype(int)]
    else:
        col = GREY[(n_obs / max(1, len(marks)) * 255).astype(int)]
    col[n_obs < 2] = NONE
    # raster with v increasing upward
    ras = np.repeat(np.repeat(col.transpose(1, 0, 2)[::-1], cell, 0), cell, 1).astype(np.uint8)
    none = np.repeat(np.repeat((n_obs < 2).T[::-1], cell, 0), cell, 1)
    yy, xx = np.indices(none.shape)
    ras[none & ((xx + yy) % 7 == 0)] = INK2.astype(np.uint8)
    lowm = np.repeat(np.repeat((n_obs < flag).T[::-1], cell, 0), cell, 1).astype(np.uint8)
    cnts, _ = cv2.findContours(lowm, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(ras, cnts, -1, FLAG.tolist(), 2)

    H, W = ras.shape[:2]
    L, T, R, B = 52, 34, 150, 44
    img = np.empty((H + T + B, W + L + R, 3), np.uint8)
    img[:] = SURFACE
    img[T:T + H, L:L + W] = ras
    cv2.rectangle(img, (L - 1, T - 1), (L + W, T + H), INK2.tolist(), 1)
    to_px = lambda h, v: (L + (h - h_ax[0]) / (h_ax[-1] - h_ax[0] + 1e-9) * (W - cell) + cell / 2,
                          T + H - cell / 2 - (v - v_ax[0]) / (v_ax[-1] - v_ax[0] + 1e-9) * (H - cell))

    # ticks every 20 in
    for ax, horiz in ((h_ax, True), (v_ax, False)):
        for t in np.arange(0, ax[-1] / MM_PER_IN + 1e-6, 20):
            x, y = to_px(t * MM_PER_IN, v_ax[0]) if horiz else to_px(h_ax[0], t * MM_PER_IN)
            if horiz:
                _text(img, f"{t:g}", (x, T + H + 12), 0.38, INK2, anchor="c")
            else:
                _text(img, f"{t:g}", (L - 6, y), 0.38, INK2, anchor="r")
    _text(img, labels[0], (L + W / 2, T + H + 30), 0.42, INK2, anchor="c")
    _text(img, labels[1], (8, T - 14), 0.42, INK2)

    if box is not None:
        (h0, v0), (h1, v1), (hp, vp) = box
        p0, p1 = to_px(h0, v0), to_px(h1, v1)
        cv2.rectangle(img, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), INK.tolist(), 1, cv2.LINE_AA)
        cv2.drawMarker(img, tuple(int(c) for c in to_px(hp, vp)), INK.tolist(), cv2.MARKER_CROSS, 12, 2)
    for h, v, name in marks:
        if h is None:
            continue
        x, y = to_px(h, v)
        cv2.circle(img, (int(x), int(y)), 6, INK.tolist(), -1, cv2.LINE_AA)
        _text(img, name, (x, y), 0.36, SURFACE, anchor="c")

    # legend / colour bar
    bx = L + W + 18
    if kind == "error":
        bh = H - 90                                     # room for the legend below
        bar = BLUE[np.linspace(255, 0, bh).astype(int)][:, None].repeat(14, 1)
        img[T:T + bh, bx:bx + 14] = bar
        for f in np.linspace(0, 1, 6):
            _text(img, f"{f * vmax:.3g}", (bx + 20, T + bh - f * bh), 0.36, INK2)
        _text(img, "RMS mm", (bx, T - 14), 0.42, INK2)
        _text(img, f"clips at {vmax:.3g}", (bx, T + bh + 14), 0.34, INK2)
    else:
        C = len(marks)
        for i, c in enumerate(range(C, -1, -1)):
            y = T + i * 18
            img[y:y + 14, bx:bx + 14] = GREY[int(c / max(1, C) * 255)] if c >= 2 else NONE
            _text(img, f"{c}", (bx + 22, y + 7), 0.36, INK2)
        _text(img, "cameras", (bx, T - 14), 0.42, INK2)
    y = T + H - 34
    cv2.rectangle(img, (bx, y), (bx + 14, y + 12), FLAG.tolist(), 2)
    _text(img, f"< {flag} cams", (bx + 20, y + 6), 0.34, INK2)
    img[y + 18:y + 30, bx:bx + 14] = NONE
    _text(img, "< 2: no fix", (bx + 20, y + 24), 0.34, INK2)
    return img


def titled(img, title, sub=""):
    head = np.empty((52 if sub else 32, img.shape[1], 3), np.uint8)
    head[:] = SURFACE
    _text(head, title, (14, 16), 0.55, INK, 1)
    if sub:
        _text(head, sub, (14, 38), 0.40, INK2)
    return np.vstack([head, img])


def hcat(imgs):
    h = max(i.shape[0] for i in imgs)
    pad = lambda i: np.vstack([i, np.full((h - i.shape[0], i.shape[1], 3), SURFACE, np.uint8)])
    return np.hstack([pad(i) for i in imgs])


# ---------------------------------------------------------------- provenance

def provenance(paths, S, info, rig):
    def git(*a):
        try:
            return subprocess.run(["git", *a], cwd=HERE, capture_output=True, text=True).stdout.strip()
        except OSError:
            return None
    return {
        "timestamp": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "git_commit": git("rev-parse", "HEAD"),
        # ":/" anchors the pathspec at the repo root; untracked study files count.
        "git_dirty": bool(git("status", "--porcelain", "--", ":/mocap/python",
                              ":/mocap/config/layout.yaml", ":/mocap/config/study.yaml")),
        "config_sha256": {os.path.basename(p): hashlib.sha256(open(p, "rb").read()).hexdigest()
                          for p in paths},
        "versions": {"python": platform.python_version(), "numpy": np.__version__,
                     "opencv": cv2.__version__, "platform": platform.platform()},
        "layout_inches": info["raw"],
        "study": S["raw"],
        "units_internal": "mm",
        "noise_px_sigma_per_axis": S["noise_px"],
        "trials": S["trials"],
        "intrinsics": {k: rig.model[k] for k in ("fx", "fy", "cx", "cy", "dist", "resolution")},
        "assumed": {
            "distortion": "coefficients are plausible placeholders, not measured",
            "intrinsics": "33661's values used for all ten cameras",
            "noise": "independent zero-mean Gaussian per image axis, same on every camera",
            "visibility": "geometric only: on sensor, in front, inside FOV. No occlusion, "
                          "no marker size/brightness falloff with range",
            "calibration": "perfect; see floor_note",
        },
        "floor_note": FLOOR_NOTE,
    }


# ---------------------------------------------------------------- main

def main(layout_path, study_path):
    t0 = time.time()
    print("=" * 78)
    print("PANOPTES synthetic triangulation study")
    print("=" * 78)
    print("\nStep 0: triangulation self-test (zero noise must recover truth)")
    if not self_test(layout_path):
        sys.exit("self-test failed: triangulation is wrong, refusing to report numbers")

    rig, info = rigmod.load_layout(layout_path)
    S = rigmod.load_study(study_path)
    os.makedirs(S["output"]["dir"], exist_ok=True)
    out_dir, flag, frame = S["output"]["dir"], S["min_observers_flag"], volume_frame(info)
    seed = S["seed"]
    C = len(rig.names)
    every = np.ones(C, bool)
    print(f"\nRig: {C} cameras, volume {info['volume_max'] / MM_PER_IN} in "
          f"= {np.round(info['volume_max']).astype(int)} mm; noise {S['noise_px']} px; "
          f"{S['trials']} trials/point")
    print(f"Field of view from model: corner ray at {np.degrees(np.arctan(rig.r_max)):.1f} deg "
          f"off axis; edge-centre H x V = "
          + " x ".join(f"{2 * np.degrees(np.arctan(np.abs(rig.undistort(np.array(p, float))).max())):.1f}"
                       for p in ([0, rig.cy], [rig.cx, 0])) + " deg")

    vol_pts, _ = grid(info["volume_min"], info["volume_max"], S["grid"]["spacing_mm"])
    p_lo, p_hi = perch_box(info, S["perch"]["half_size_mm"])
    perch_pts, _ = grid(p_lo, p_hi, S["perch"]["spacing_mm"])
    print(f"Grid: {len(vol_pts)} volume points (<= {S['grid']['spacing_mm'] / MM_PER_IN:g} in), "
          f"{len(perch_pts)} perch-box points (<= {S['perch']['spacing_mm'] / MM_PER_IN:g} in)")
    print(f"\nNOTE: {FLOOR_NOTE}")

    results = {}
    with ProcessPoolExecutor() as pool:

        # ---- 1-2. nominal
        print("\nStep 1-2: nominal layout, all cameras ...", flush=True)
        vol = evaluate(pool, rig, vol_pts, every, S["trials"], S, [seed, 1], frame)
        per = evaluate(pool, rig, perch_pts, every, S["trials"], S, [seed, 2], frame)
        nominal = {"volume": summarise(vol, flag), "perch": summarise(per, flag)}
        table("Accuracy, nominal layout (RMS 3D error, mm)",
              [("whole volume", nominal["volume"]), ("perch box", nominal["perch"])], HEADLINE)
        hist = np.bincount(vol["n_obs"], minlength=C + 1)
        print("\n  observers per volume point: " +
              "  ".join(f"{k}:{v}" for k, v in enumerate(hist) if v))
        # Coverage by height: where the usable volume actually ends.
        print(f"\n  by height  {'>=2 cams':>9}{'>=3 cams':>9}{'median mm':>11}{'z med mm':>10}")
        layers = []
        for z in np.unique(vol_pts[:, 2]):
            m = vol_pts[:, 2] == z
            f2, f3 = (vol["n_obs"][m] >= 2).mean(), (vol["n_obs"][m] >= flag).mean()
            ok = m & np.isfinite(vol["rms"])
            med = float(np.median(vol["rms"][ok])) if ok.any() else None
            zm = float(np.median(vol["rms_axis"][ok, 2])) if ok.any() else None
            layers.append({"z_in": round(z / MM_PER_IN, 2), "frac_ge2": f2, "frac_ge3": f3,
                           "median_mm": med, "z_median_mm": zm})
            print(f"  {z / MM_PER_IN:6.1f} in {f2:>9.0%}{f3:>9.0%}"
                  f"{'-' if med is None else format(med, '.4f'):>11}{'-' if zm is None else format(zm, '.4f'):>10}")
        perch_low = perch_pts[per["n_obs"] < flag]
        print(f"  perch-box points with < {flag} cameras: {len(perch_low)} of {len(perch_pts)}")
        results["nominal"] = nominal
        results["nominal"]["observer_histogram"] = hist.tolist()
        results["nominal"]["by_height"] = layers
        results["nominal"]["perch_points_below_flag_in"] = (perch_low / MM_PER_IN).round(1).tolist()

        # ---- 3. heat maps
        print("\nStep 3: heat-map slices ...", flush=True)
        sl = S["slices"]
        lo, hi = info["volume_min"], info["volume_max"]
        planes = [("z", z) for z in sl["z_mm"]] + [("y", sl["perch_plane_y_mm"])]
        maps = []
        for k, (axis, val) in enumerate(planes):
            a = "xyz".index(axis)
            l2, h2 = lo.copy(), hi.copy()
            l2[a] = h2[a] = val
            pts, axes = grid(l2, h2, sl["spacing_mm"])
            r = evaluate(pool, rig, pts, every, sl["trials"], S, [seed, 3, k], frame)
            maps.append((axis, val, pts, axes, r))
        allv = np.concatenate([m[4]["rms"] for m in maps])
        vmax = _nice(np.nanpercentile(allv, sl["vmax_percentile"]))
        overview, slice_files = [], []
        for axis, val, pts, axes, r in maps:
            keep = [i for i in range(3) if "xyz"[i] != axis]
            h_ax, v_ax = axes[keep[0]], axes[keep[1]]
            shape = (len(h_ax), len(v_ax))
            rms, n = r["rms"].reshape(shape), r["n_obs"].reshape(shape)
            marks = [(c[keep[0]], c[keep[1]], name) for c, name in zip(rig.C, rig.names)]
            half = S["perch"]["half_size_mm"]
            near = abs(info["perch"]["xyz".index(axis)] - val) <= half
            box = ((p_lo[keep[0]], p_lo[keep[1]]), (p_hi[keep[0]], p_hi[keep[1]]),
                   (info["perch"][keep[0]], info["perch"][keep[1]])) if near else None
            labels = (f"{'xyz'[keep[0]]} (in)", f"{'xyz'[keep[1]]} (in)")
            pe = panel(rms, n, h_ax, v_ax, "error", vmax, flag, marks, box, labels)
            pc = panel(rms, n, h_ax, v_ax, "count", vmax, flag, marks, box, labels)
            ok = np.isfinite(r["rms"])
            name = f"{axis} = {val / MM_PER_IN:g} in"
            stats = (f"median {np.median(r['rms'][ok]):.4f} mm, p90 {np.percentile(r['rms'][ok], 90):.4f} mm; "
                     if ok.any() else "NO POINT on this slice is seen by 2+ cameras; ")
            sub = (stats + f"{(r['n_obs'] < flag).mean():.0%} of slice under {flag} cameras. "
                   f"Perfect-calibration floor. + = perch, box = perch region.")
            img = titled(hcat([pe, pc]), f"Predicted RMS error and camera coverage, {name}", sub)
            fn = f"slice_{axis}{val / MM_PER_IN:g}".replace(".", "p") + "in.png"
            cv2.imwrite(os.path.join(out_dir, fn), img)
            slice_files.append(fn)
            if axis == "z":
                overview.append(titled(pe, name))
        ov = titled(hcat(overview), "Predicted RMS 3D error, horizontal slices (shared scale)",
                    f"{S['noise_px']} px noise, perfect calibration, all {C} cameras")
        cv2.imwrite(os.path.join(out_dir, "slices_overview.png"), ov)
        slice_files.append("slices_overview.png")
        print(f"  wrote {len(slice_files)} PNGs to {out_dir}; colour scale 0..{vmax:g} mm")
        results["heatmaps"] = {"files": slice_files, "vmax_mm": vmax}

        # ---- 4. camera subsets
        print("\nStep 4: camera subsets ...", flush=True)
        sub = S["subsets"]
        pick = lambda names: np.isin(rig.names, names)
        sets = [("all 10", every), (f"upper {len(sub['upper'])}", pick(sub["upper"])),
                (f"lower {len(sub['lower'])}", pick(sub["lower"]))]
        rng = np.random.default_rng([seed, 4])
        seen = {tuple(pick(sub["upper"]))}
        while len(sets) < 3 + sub["random_count"]:
            m = np.zeros(C, bool)
            m[rng.choice(C, sub["random_size"], replace=False)] = True
            if tuple(m) not in seen:
                seen.add(tuple(m))
                sets.append(("rand " + "".join(np.array(rig.names)[m]), m))
        rows_v, rows_p, results["subsets"] = [], [], {}
        for k, (label, m) in enumerate(sets):
            if k == 0:
                sv, sp = nominal["volume"], nominal["perch"]
            else:
                sv = summarise(evaluate(pool, rig, vol_pts, m, S["trials"], S, [seed, 5, k], frame), flag)
                sp = summarise(evaluate(pool, rig, perch_pts, m, S["trials"], S, [seed, 6, k], frame), flag)
            rows_v.append((label, sv))
            rows_p.append((label, sp))
            results["subsets"][label] = {"cameras": list(np.array(rig.names)[m]),
                                         "volume": sv, "perch": sp}
        table("Camera subsets, whole volume (stats over points each subset can triangulate)",
              rows_v, HEADLINE)
        table("Camera subsets, perch box", rows_p, HEADLINE)

        # ---- 5. sensitivity
        se = S["sensitivity"]
        print(f"\nStep 5: sensitivity, {se['layouts']} layouts perturbed by "
              f"+/-{se['position_mm'] / MM_PER_IN:g} in, "
              f"{se['aim_deg']:g} deg aim, {se['roll_deg']:g} deg roll ...", flush=True)
        rng = np.random.default_rng([seed, 7])
        runs = []
        for k in range(se["layouts"]):
            pr = rigmod.perturb(rig, rng, se["position_mm"], se["aim_deg"], se["roll_deg"])
            # Truth and reconstruction both use the perturbed rig: the question
            # is how good a calibrated rig would be if the true layout is this one.
            sv = summarise(evaluate(pool, pr, vol_pts, every, se["trials"], S, [seed, 8, k], frame), flag)
            sp = summarise(evaluate(pool, pr, perch_pts, every, se["trials"], S, [seed, 9, k], frame), flag)
            runs.append({"volume": sv, "perch": sp})
        # Nominal rerun at the sensitivity trial count: separates Monte Carlo
        # sampling noise from genuine layout dependence.
        ref = {"volume": summarise(evaluate(pool, rig, vol_pts, every, se["trials"], S, [seed, 10], frame), flag),
               "perch": summarise(evaluate(pool, rig, perch_pts, every, se["trials"], S, [seed, 11], frame), flag)}

    metrics = [("volume", "median_mm"), ("volume", "p90_mm"), ("volume", "max_mm_3plus"),
               ("volume", "triangulable_frac"), ("volume", "flagged_frac"),
               ("perch", "median_mm"), ("perch", "p90_mm"), ("perch", "max_mm"),
               ("perch", "flagged_frac")]
    spread, verdict_rows = {}, []
    print(f"\nSensitivity over {len(runs)} perturbed layouts (nominal at {se['trials']} trials for reference)")
    print(f"  {'metric':<26}{'nominal':>10}{'min':>10}{'median':>10}{'max':>10}{'(max-min)/nom':>15}")
    for region, key in metrics:
        vals = np.array([r[region][key] for r in runs], float)
        nom = ref[region][key]
        rel = (vals.max() - vals.min()) / nom if nom else float(vals.max() - vals.min())
        spread[f"{region}.{key}"] = {"nominal": nom, "min": float(vals.min()),
                                     "median": float(np.median(vals)), "max": float(vals.max()),
                                     "rel_spread": float(rel)}
        fmt = ".1%" if "frac" in key else ".4f"
        print(f"  {region + ' ' + key:<26}{nom:>10{fmt}}{vals.min():>10{fmt}}"
              f"{np.median(vals):>10{fmt}}{vals.max():>10{fmt}}{rel:>15.1%}")
        if key in ("median_mm", "p90_mm"):
            verdict_rows.append(rel)
    # Coverage that matters: does any plausible layout leave perch points short?
    perch_short = max(r["perch"]["flagged_frac"] for r in runs)
    robust = max(verdict_rows) < se["robust_rel_spread"] and perch_short == 0
    verdict = (
        f"ROBUST: median/p90 error varies by at most {max(verdict_rows):.0%} across layouts "
        f"(threshold {se['robust_rel_spread']:.0%}) and no perturbed layout leaves any perch-box "
        f"point under {flag} cameras. Tape-measure positions are good enough for this conclusion."
        if robust else
        f"SENSITIVE: median/p90 error varies by up to {max(verdict_rows):.0%} across layouts "
        f"(threshold {se['robust_rel_spread']:.0%}); worst perturbed layout leaves "
        f"{perch_short:.0%} of perch-box points under {flag} cameras. The layout is marginal; "
        f"the headline numbers depend on where the cameras really are.")
    print(f"\n  {verdict}")
    results["sensitivity"] = {"layouts": runs, "nominal_same_trials": ref, "spread": spread,
                              "robust": robust, "verdict": verdict}

    # ---- outputs
    paths = [layout_path, study_path]
    doc = {"provenance": provenance(paths, S, info, rig), "results": results,
           "runtime_s": round(time.time() - t0, 1)}
    jpath = os.path.join(out_dir, "accuracy_study.json")
    with open(jpath, "w") as f:
        json.dump(doc, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    np.savez_compressed(os.path.join(out_dir, "accuracy_study.npz"),
                        vol_pts_mm=vol["pts"], vol_n_obs=vol["n_obs"], vol_rms_axis_mm=vol["rms_axis"],
                        vol_pred_mm=vol["pred"], perch_pts_mm=per["pts"], perch_n_obs=per["n_obs"],
                        perch_rms_axis_mm=per["rms_axis"], camera_names=np.array(rig.names))
    print(f"\nWrote {jpath} (+ .npz per-point arrays) in {time.time() - t0:.0f} s")
    print(f"\nNOTE: {FLOOR_NOTE}")


if __name__ == "__main__":
    a = sys.argv[1:]
    main(a[0] if len(a) > 0 else os.path.join(CONFIG, "layout.yaml"),
         a[1] if len(a) > 1 else os.path.join(CONFIG, "study.yaml"))
