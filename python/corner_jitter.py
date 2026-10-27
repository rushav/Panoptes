"""
Static corner repeatability ("jitter") of ChArUco detection.

Board and camera both fixed, N frames. For each corner present in every
kept frame: its mean position and the standard deviation of its detected
position across frames. This is the figure behind FINDINGS §4 (0.054 px at
gain 7, 0.056 px at gain 0, old 9x6 board) and the corner noise the
accuracy study assumes. The script that produced that figure was lost; this
is its rebuild.

DEFINITIONS
  std_x, std_y   sample standard deviation (ddof=1) of x and of y.
  radial std     sqrt(std_x^2 + std_y^2): RMS distance from the mean
                 position. For isotropic noise of sigma per axis it is
                 sigma * sqrt(2).
  per-axis std   radial / sqrt(2): the per-axis sigma of the same noise.
                 Which of the two the lost script reported is NOT recorded,
                 so both are printed. Compare like with like.
  fit            homography from the corners' board coordinates to their
                 detected positions; max and mean residual, px. Same
                 quantity as `fit` in 03_detect (max). About 1 px when
                 labels are right, hundreds when the layout is wrong.

DETECTION is the fit_intrinsics path: CharucoDetector.detectBoard, then
board.matchImagePoints, with the board (legacy flag included) resolved by
board.py.

FRAME REJECTION, in this order, each with its reason:
  no_board    fewer than 6 corners
  duplicate   byte-identical image to an earlier frame
  settings    camera settings differ from the most common (session.json)
  fit         max fit residual above --max-fit (default 3 px)
  count       corner count differs from the most common count

REGIONS use the coverage grid (5 x 6 cells, compare_models.GRID_X/GRID_Y,
the same cell rule as lib/coverage.cpp): "edge" is the outer ring of cells,
"centre" the rest, by each corner's mean position.

Usage:
  python corner_jitter.py <session_dir>                 # 04_capture session
  python corner_jitter.py <png_dir> --board FILE        # plain PNG folder
  python corner_jitter.py ... [--max-fit PX] [--out DIR]
  python corner_jitter.py --self-test

Writes corner_jitter.json and corner_jitter.png into the input folder (or
--out).
"""

import os, sys, json, glob, hashlib, platform, subprocess, datetime, argparse
import numpy as np
import cv2

import board as board_config
from compare_models import load_board, GRID_X, GRID_Y

HERE = os.path.dirname(os.path.abspath(__file__))
MIN_CORNERS = 6
MIN_FRAMES = 3


# ---------------------------------------------------------------- detection

def fit_residual(obj_xy, img_xy):
    """(max, mean) px residual of a homography board -> image."""
    H, _ = cv2.findHomography(obj_xy.astype(np.float64), img_xy.astype(np.float64))
    if H is None:
        return float("inf"), float("inf")
    proj = cv2.perspectiveTransform(obj_xy.reshape(-1, 1, 2).astype(np.float64), H).reshape(-1, 2)
    r = np.linalg.norm(proj - img_xy, axis=1)
    return float(r.max()), float(r.mean())


def detect_frames(files, board, detector):
    """One record per PNG: ids, image points, board points, image hash."""
    records, size = [], None
    for f in files:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        rec = {"file": os.path.basename(f), "ids": None,
               "hash": hashlib.sha256(img.tobytes()).hexdigest() if img is not None else None}
        records.append(rec)
        if img is None:
            continue
        size = size or (img.shape[1], img.shape[0])
        corners, ids, _, _ = detector.detectBoard(img)
        if ids is None or len(ids) < MIN_CORNERS:
            continue
        op, ip = board.matchImagePoints(corners, ids)
        if op is None or len(op) < MIN_CORNERS:
            continue
        rec["ids"] = ids.ravel().astype(int)
        rec["img"] = ip.reshape(-1, 2).astype(np.float64)
        rec["obj"] = op.reshape(-1, 3)[:, :2].astype(np.float64)
    return records, size


# ---------------------------------------------------------------- analysis

def _mode(values):
    vals, counts = np.unique(np.asarray(values), return_counts=True)
    return vals[np.argmax(counts)]


def _cell(x, y, size):
    return (min(max(int(x * GRID_X / size[0]), 0), GRID_X - 1),
            min(max(int(y * GRID_Y / size[1]), 0), GRID_Y - 1))


def _stats(r):
    r = np.asarray(r)
    if len(r) == 0:
        return {"corners": 0}
    return {"corners": int(len(r)),
            "radial_median": float(np.median(r)), "radial_p90": float(np.percentile(r, 90)),
            "radial_max": float(r.max()),
            "per_axis_median": float(np.median(r) / np.sqrt(2))}


def analyse(records, size, max_fit, settings=None, total_corners=None):
    """
    records: dicts with file, hash, and (if detected) ids, img (N,2), obj (N,2).
    settings: optional {file: settings tuple}. Returns the result dict.
    """
    seen, frames = set(), []
    for rec in records:
        fr = {"file": rec["file"], "corners": 0, "fit_max": None, "fit_mean": None,
              "settings": None, "rejected": None}
        frames.append(fr)
        if settings is not None:
            fr["settings"] = settings.get(rec["file"])
        if rec.get("ids") is None:
            fr["rejected"] = "no_board"
            continue
        fr["corners"] = int(len(rec["ids"]))
        fr["fit_max"], fr["fit_mean"] = fit_residual(rec["obj"], rec["img"])
        if rec["hash"] in seen:
            fr["rejected"] = "duplicate"
        seen.add(rec["hash"])

    def live():
        return [(f, r) for f, r in zip(frames, records) if f["rejected"] is None]

    if settings is not None and live():
        modal = _mode([json.dumps(f["settings"]) for f, _ in live()])
        for f, _ in live():
            if json.dumps(f["settings"]) != modal:
                f["rejected"] = "settings"
    for f, _ in live():
        if f["fit_max"] > max_fit:
            f["rejected"] = "fit"
    if live():
        modal_count = int(_mode([f["corners"] for f, _ in live()]))
        for f, _ in live():
            if f["corners"] != modal_count:
                f["rejected"] = "count"

    kept = live()
    rejected = {}
    for f in frames:
        if f["rejected"]:
            rejected[f["rejected"]] = rejected.get(f["rejected"], 0) + 1
    result = {"frames_total": len(frames), "frames_kept": len(kept),
              "rejected": rejected, "max_fit_px": max_fit, "frames": frames,
              "image_size": list(size) if size else None}
    if len(kept) < MIN_FRAMES:
        result["error"] = f"only {len(kept)} frames kept; need at least {MIN_FRAMES}"
        return result

    # Corners present in every kept frame.
    id_sets = [set(r["ids"].tolist()) for _, r in kept]
    common = sorted(set.intersection(*id_sets))
    union = sorted(set.union(*id_sets))
    dropped = sorted(set(union) - set(common))
    never = sorted(set(range(total_corners)) - set(union)) if total_corners else []

    P = np.stack([np.stack([r["img"][np.flatnonzero(r["ids"] == i)[0]] for i in common])
                  for _, r in kept])                        # frames x corners x 2
    mean = P.mean(axis=0)
    sd = P.std(axis=0, ddof=1)
    radial = np.sqrt((sd ** 2).sum(axis=1))

    corners, edge_r, centre_r = [], [], []
    for k, cid in enumerate(common):
        cx, cy = _cell(mean[k, 0], mean[k, 1], size)
        edge = cx in (0, GRID_X - 1) or cy in (0, GRID_Y - 1)
        (edge_r if edge else centre_r).append(radial[k])
        corners.append({"id": int(cid), "x": float(mean[k, 0]), "y": float(mean[k, 1]),
                        "std_x": float(sd[k, 0]), "std_y": float(sd[k, 1]),
                        "radial_std": float(radial[k]), "cell": [cx, cy],
                        "region": "edge" if edge else "centre"})

    fits = np.array([f["fit_max"] for f, _ in kept])
    fit_means = np.array([f["fit_mean"] for f, _ in kept])
    result.update({
        "corners_used": len(common), "corners_on_board": total_corners,
        "corners_dropped": dropped, "corners_never_seen": never,
        "summary": _stats(radial),
        "regions": {"grid": [GRID_X, GRID_Y], "centre": _stats(centre_r), "edge": _stats(edge_r)},
        "fit_kept": {"max_median": float(np.median(fits)), "max_max": float(fits.max()),
                     "mean_median": float(np.median(fit_means))},
        "corners": corners,
    })
    return result


# ---------------------------------------------------------------- output

def git_commit():
    def git(*a):
        try:
            p = subprocess.run(["git", *a], cwd=HERE, capture_output=True, text=True)
            return p.stdout.strip() if p.returncode == 0 else None
        except OSError:
            return None
    commit = git("rev-parse", "HEAD")
    status = git("status", "--porcelain", "--", ":/python", ":/config/charuco_board.yaml")
    return commit, (None if status is None else bool(status))


def draw(result, path):
    """Per-corner radial std over the image, on the coverage grid."""
    w, h = result["image_size"]
    s = 0.75
    img = np.full((int(h * s) + 70, int(w * s), 3), 255, np.uint8)
    for i in range(1, GRID_X):
        x = int(i * w / GRID_X * s)
        cv2.line(img, (x, 0), (x, int(h * s)), (215, 215, 215), 1)
    for j in range(1, GRID_Y):
        y = int(j * h / GRID_Y * s)
        cv2.line(img, (0, y), (int(w * s), y), (215, 215, 215), 1)
    cv2.rectangle(img, (0, 0), (int(w * s) - 1, int(h * s) - 1), (150, 150, 150), 1)

    r = np.array([c["radial_std"] for c in result["corners"]])
    hi = max(float(np.percentile(r, 99)), 1e-9)
    colours = cv2.applyColorMap(np.clip(r / hi * 255, 0, 255).astype(np.uint8).reshape(-1, 1),
                                cv2.COLORMAP_VIRIDIS).reshape(-1, 3)
    for c, col in zip(result["corners"], colours):
        cv2.circle(img, (int(c["x"] * s), int(c["y"] * s)), 6, tuple(int(v) for v in col), -1)

    # Colour bar and caption.
    y0 = int(h * s) + 12
    bar = cv2.applyColorMap(np.tile(np.arange(256, dtype=np.uint8), (14, 1)), cv2.COLORMAP_VIRIDIS)
    img[y0:y0 + 14, 20:276] = bar
    cv2.putText(img, "0", (14, y0 + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 60), 1)
    cv2.putText(img, f"{hi:.3f} px", (250, y0 + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 60), 1)
    sm = result["summary"]
    cv2.putText(img, f"radial std: median {sm['radial_median']:.4f}  p90 {sm['radial_p90']:.4f}  "
                     f"max {sm['radial_max']:.4f} px   {result['corners_used']} corners, "
                     f"{result['frames_kept']} frames",
                (300, y0 + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (30, 30, 30), 1)
    cv2.imwrite(path, img)


def report(result):
    print(f"\nframes: {result['frames_total']} total, {result['frames_kept']} kept")
    for reason, n in sorted(result["rejected"].items()):
        files = [f["file"] for f in result["frames"] if f["rejected"] == reason]
        more = f" ... (+{len(files) - 5})" if len(files) > 5 else ""
        print(f"  rejected {reason:<9} {n:>4}: {', '.join(files[:5])}{more}")
    if "error" in result:
        return
    print(f"corners used: {result['corners_used']}"
          + (f" of {result['corners_on_board']} on the board" if result["corners_on_board"] else ""))
    if result["corners_dropped"]:
        print(f"  dropped (not in every kept frame): {result['corners_dropped']}")
    if result["corners_never_seen"]:
        print(f"  never detected: {result['corners_never_seen']}")
    fk = result["fit_kept"]
    print(f"fit (kept frames): max residual median {fk['max_median']:.2f} px, worst "
          f"{fk['max_max']:.2f} px; mean residual median {fk['mean_median']:.2f} px")
    print(f"\n  {'':<8}{'corners':>8}{'radial median':>15}{'p90':>9}{'max':>9}{'per-axis median':>17}")
    for name, st in [("all", result["summary"]), ("centre", result["regions"]["centre"]),
                     ("edge", result["regions"]["edge"])]:
        if st["corners"]:
            print(f"  {name:<8}{st['corners']:>8}{st['radial_median']:>15.4f}{st['radial_p90']:>9.4f}"
                  f"{st['radial_max']:>9.4f}{st['per_axis_median']:>17.4f}")
        else:
            print(f"  {name:<8}{0:>8}")
    print("  (px; radial = sqrt(std_x^2 + std_y^2), per-axis = radial / sqrt(2))")
    if result["summary"]["radial_max"] > 1.0:
        print("\nWARNING: a corner moved more than 1 px (std). The board or camera was not"
              "\nstatic, so this is not a repeatability measurement.")


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("folder", nargs="?")
    ap.add_argument("--board", help="board yaml, for a PNG folder without session.json")
    ap.add_argument("--max-fit", type=float, default=3.0, help="reject frames above this fit, px")
    ap.add_argument("--out", help="output folder (default: the input folder)")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args(argv)
    if a.self_test:
        return 0 if self_test() else 1
    if not a.folder:
        ap.error("folder required")

    folder = os.path.normpath(a.folder)
    meta_path = os.path.join(folder, "session.json")
    settings = None
    if os.path.exists(meta_path):
        if a.board:
            sys.exit(f"{meta_path} already records the board; --board is only for PNG folders")
        meta = json.load(open(meta_path))
        board, detector, spec = load_board(meta, folder)
        how = board_config.for_session(meta, folder)[1]
        settings = {f["file"]: [f.get(k) for k in ("exposure", "gain", "intensity",
                                                    "frame_rate", "ir_filter")]
                    for f in meta.get("frames", [])}
    elif a.board:
        spec = board_config.load(a.board)
        how = "from --board"
        print(f"{board_config.describe(spec)}  [{how}]")
        board, detector = board_config.make(spec)
    else:
        sys.exit(f"no session.json in {folder}; pass --board FILE")

    files = sorted(glob.glob(os.path.join(folder, "*.png")))
    files = [f for f in files if os.path.basename(f) != "corner_jitter.png"]
    if not files:
        sys.exit(f"no PNGs in {folder}")
    records, size = detect_frames(files, board, detector)
    total = (spec["rows"] - 1) * (spec["cols"] - 1)
    result = analyse(records, size, a.max_fit, settings, total)
    report(result)

    commit, dirty = git_commit()
    if commit is None:
        print("\nwarning: git commit unavailable; recorded as null")
    result = {
        "script": "python/corner_jitter.py",
        "timestamp": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "git_commit": commit, "git_dirty": dirty,
        "versions": {"python": platform.python_version(), "numpy": np.__version__,
                     "opencv": cv2.__version__},
        "input": folder, "board": spec, "board_resolved": how,
        "definitions": {"radial_std": "sqrt(std_x^2 + std_y^2), ddof=1",
                        "per_axis": "radial / sqrt(2)",
                        "fit": "homography board->image residual, px; frames rejected on max"},
        **result,
    }
    out = a.out or folder
    os.makedirs(out, exist_ok=True)
    jpath = os.path.join(out, "corner_jitter.json")
    with open(jpath, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nwritten {jpath}")
    if "error" in result:
        print(f"error: {result['error']}")
        return 1
    ppath = os.path.join(out, "corner_jitter.png")
    draw(result, ppath)
    print(f"written {ppath}")
    return 0


# ---------------------------------------------------------------- self-test

def self_test():
    """
    Synthetic frames with known Gaussian corner noise through rig.py's camera
    model, then the same analyse() as real data. Plus one rendered image
    through the real detection path.
    """
    import rig as rigmod
    rng = np.random.default_rng(7)
    sigma, n_frames, tol = 0.05, 200, 0.10
    cols, rows, sq = 12, 9, 60.0
    results = []

    def check(name, ok, detail):
        results.append(ok)
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<52} {detail}")

    model = rigmod.read_yaml(os.path.join(HERE, "..", "config", "layout.yaml"))["camera_model"]
    cam = rigmod.Rig(["synthetic"], [0], [[0, 0, 0]], [np.eye(3)], model)
    size = (int(cam.width), int(cam.height))

    # Interior corners in board mm, ids row-major like OpenCV's.
    gx, gy = np.meshgrid(np.arange(1, cols) * sq, np.arange(1, rows) * sq)
    obj = np.stack([gx.ravel(), gy.ravel()], -1)
    ids = np.arange(len(obj))

    # Board 1 m out, tilted 20 deg about camera y, off centre so its
    # corners reach the edge ring of the coverage grid.
    a = np.radians(20)
    Ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    centre = obj.mean(0)
    X = (Ry @ np.c_[obj - centre, np.zeros(len(obj))].T).T + [-330, -170, 1000]
    truth = cam.project(X)[0][:, 0, :]

    def rec(name, i, o, p):
        return {"file": name, "hash": name, "ids": i, "obj": o, "img": p}

    records = [rec(f"f{k:03d}", ids, obj, truth + rng.normal(0, sigma, truth.shape))
               for k in range(n_frames)]
    clean = analyse(records, size, 3.0, None, len(obj))
    print(f"\ncorner_jitter.py self-test (sigma {sigma} px per axis, {n_frames} frames, "
          f"{len(obj)} corners)")
    st = clean["summary"]
    check("all frames kept, all corners used",
          clean["frames_kept"] == n_frames and clean["corners_used"] == len(obj),
          f"{clean['frames_kept']} frames, {clean['corners_used']} corners")
    check("clean fit below 3 px (rig distortion only)", clean["fit_kept"]["max_max"] < 3.0,
          f"worst {clean['fit_kept']['max_max']:.3f} px")
    err = st["radial_median"] / (sigma * np.sqrt(2)) - 1
    check("radial median within 10% of sigma*sqrt(2)", abs(err) < tol,
          f"{st['radial_median']:.4f} vs {sigma * np.sqrt(2):.4f} ({err:+.1%})")
    err = st["per_axis_median"] / sigma - 1
    check("per-axis median within 10% of sigma", abs(err) < tol,
          f"{st['per_axis_median']:.4f} vs {sigma:.4f} ({err:+.1%})")
    rg = clean["regions"]
    check("regions: both populated, sum to corners used",
          rg["centre"]["corners"] > 0 and rg["edge"]["corners"] > 0
          and rg["centre"]["corners"] + rg["edge"]["corners"] == clean["corners_used"],
          f"centre {rg['centre']['corners']}, edge {rg['edge']['corners']}")

    # Faults, each of which must be rejected for its own reason.
    bad = list(records[:50])
    wrong = np.roll(ids, 5)                                             # IDs on wrong corners;
    bad.append(rec("mislabelled", wrong, obj[wrong], truth))           # board points follow IDs
    bad.append(rec("partial", ids[:-5], obj[:-5], truth[:-5]))         # 83 corners
    bad.append(dict(records[0], file="dup"))                            # same image hash
    bad.append({"file": "blank", "hash": "blank", "ids": None})
    faulty = analyse(bad, size, 3.0, None, len(obj))
    why = {f["file"]: f["rejected"] for f in faulty["frames"]}
    expected = {"mislabelled": "fit", "partial": "count", "dup": "duplicate", "blank": "no_board"}
    check("faults rejected for the right reason",
          all(why[k] == v for k, v in expected.items()) and faulty["frames_kept"] == 50,
          ", ".join(f"{k}->{why[k]}" for k in expected))
    mis = next(f for f in faulty["frames"] if f["file"] == "mislabelled")
    check("mislabelled frame shows in its fit", mis["fit_max"] > 20,
          f"{mis['fit_max']:.1f} px")

    sets = {f"f{k:03d}": [8133, 7, 15, 30, True] for k in range(20)}
    sets["f019"] = [8133, 0, 15, 30, True]
    mixed = analyse(records[:20], size, 3.0, sets, len(obj))
    check("frame with different settings rejected",
          mixed["rejected"] == {"settings": 1}, str(mixed["rejected"]))

    # The real detection path, on a rendered 12x9 board.
    spec = {"source": "self-test", "name": "synthetic", "rows": rows, "cols": cols,
            "square_mm": {"nominal": 60.0, "measured": 60.0},
            "marker_mm": {"nominal": 45.0, "measured": 45.0},
            "dictionary": "DICT_5X5_100", "legacy_pattern": False,
            "provenance": {k: "" for k in board_config.PROVENANCE}}
    board, detector = board_config.make(spec)
    img = board.generateImage((cols * 60, rows * 60), marginSize=40)
    h, w = img.shape
    Hw = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]),
                                     np.float32([[260, 180], [1040, 230], [1000, 860], [300, 800]]))
    img = cv2.warpPerspective(img, Hw, size, borderValue=128)
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        frames = []
        for k in range(5):
            noisy = np.clip(img + rng.normal(0, 2, img.shape), 0, 255).astype(np.uint8)
            frames.append(os.path.join(d, f"frame_{k:04d}.png"))
            cv2.imwrite(frames[-1], noisy)
        recs, sz = detect_frames(frames, board, detector)
        det = analyse(recs, sz, 3.0, None, (rows - 1) * (cols - 1))
    ok = det.get("frames_kept") == 5 and det.get("corners_used") == (rows - 1) * (cols - 1)
    check("rendered board through detectBoard + matchImagePoints", ok,
          f"{det.get('frames_kept')} frames, {det.get('corners_used')} corners, "
          f"fit {det.get('fit_kept', {}).get('max_max', float('nan')):.2f} px")

    print(f"  self-test: {sum(results)}/{len(results)} passed")
    return all(results)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
