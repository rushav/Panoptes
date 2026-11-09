"""
Final intrinsics fit, and repeatability across sessions.

Fits Brown-Conrady on every frame of each session (no held-out split -
that was for model selection; here we want the best estimate each
session can give) and compares parameters across sessions.

The spread across independent sessions IS the accuracy figure. Held-out
reprojection error says how well the model fits its own data; it says
nothing about whether a fresh capture would land on the same numbers.

Usage: python fit_intrinsics.py <session_dir> [<session_dir> ...]
       python fit_intrinsics.py --write-camera [--force] [--status TEXT] <session_dir> [...]

--write-camera: after the per-session fits, one joint Brown-Conrady fit over
every frame of all the sessions, written to config/intrinsics/<serial>.yaml
with its provenance. The sessions must share camera, image size, board and
camera settings (read back). The cross-session range of the per-session
fits is stored as the uncertainty; with one session there is none.
--status TEXT is written verbatim as the file's `status` (e.g. "done", or
"done, marginal (...)" when a range misses the ~1 px target); omitted, the
file has no status line.
"""

import argparse, datetime, sys, os, json
import numpy as np
import cv2

from compare_models import load_board, detect_all


CAMERA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "intrinsics")
PARAMS = ("fx", "fy", "cx", "cy")


def calibrate(obj_pts, img_pts, size):
    # Brown-Conrady (flags=0): chosen after held-out testing showed the
    # rational model's three extra coefficients bought nothing at 82 deg.
    return cv2.calibrateCamera(obj_pts, img_pts, size, None, None, flags=0)


def fit(session, keep=None):
    """Fit one session and write <session>/intrinsics.json. keep: a dict
    that receives the session's points, metadata and board spec."""
    meta = json.load(open(os.path.join(session, "session.json")))
    board, detector, spec = load_board(meta, session)
    img_pts, obj_pts, size = detect_all(session, board, detector)
    if keep is not None:
        keep.update(meta=meta, spec=spec, img_pts=img_pts, obj_pts=obj_pts, size=size)

    rms, K, dist, rvecs, tvecs = calibrate(obj_pts, img_pts, size)

    # Per-view reprojection error, so a bad capture can be spotted.
    per_view = []
    for op, ip, rv, tv in zip(obj_pts, img_pts, rvecs, tvecs):
        proj, _ = cv2.projectPoints(op, rv, tv, K, dist)
        d = proj.reshape(-1, 2) - ip.reshape(-1, 2)
        per_view.append(float(np.sqrt((d ** 2).sum(axis=1)).mean()))

    result = {
        "session": os.path.basename(session),
        "views": len(img_pts),
        "model": "Brown-Conrady (5)",
        "rms": float(rms),
        "fx": K[0, 0], "fy": K[1, 1],
        "cx": K[0, 2], "cy": K[1, 2],
        "K": K.tolist(),
        "dist": dist.ravel().tolist(),
        "per_view_error": per_view,
        "opencv_version": cv2.__version__,
        "image_size": list(size),
        "board": spec,
    }

    with open(os.path.join(session, "intrinsics.json"), "w") as f:
        json.dump(result, f, indent=2)

    return result


def _settings(meta):
    """Camera settings as read back at the session's start."""
    b = meta.get("baseline_settings") or {}
    return b.get("readback") or b.get("requested")


def _q(v):
    return json.dumps(str(v))


def write_camera(sessions, results, kept, force=False, out_dir=CAMERA_DIR, status=None):
    """The joint fit over every frame of all sessions, written to
    out_dir/<serial>.yaml (cv2.FileStorage-readable). Returns (path, dict)."""
    why = []
    for key, get in (("camera serial", lambda k: k["meta"].get("camera_serial")),
                     ("image size", lambda k: tuple(k["size"])),
                     ("board", lambda k: json.dumps(k["spec"], sort_keys=True)),
                     ("camera settings", lambda k: json.dumps(_settings(k["meta"]), sort_keys=True))):
        vals = {get(k) for k in kept}
        if len(vals) != 1:
            why.append(f"the sessions differ in {key}: {sorted(map(str, vals))}")
    serial = kept[0]["meta"].get("camera_serial")
    if serial is None:
        why.append("session.json has no camera_serial")
    if _settings(kept[0]["meta"]) is None:
        why.append("session.json has no baseline_settings")
    if why:
        sys.exit("fit_intrinsics: not writing a camera file:\n  " + "\n  ".join(why))
    path = os.path.join(out_dir, f"{serial}.yaml")
    if os.path.exists(path) and not force:
        sys.exit(f"fit_intrinsics: {os.path.relpath(path)} exists; --force to replace it")

    obj = [p for k in kept for p in k["obj_pts"]]
    img = [p for k in kept for p in k["img_pts"]]
    size = kept[0]["size"]
    rms, K, dist, _, _ = calibrate(obj, img, size)
    dist = dist.ravel()
    rng = {p: max(r[p] for r in results) - min(r[p] for r in results) for p in PARAMS}
    dates = sorted({os.path.basename(s.rstrip("/")).split("_")[2] for s in sessions})
    dates = [f"{d[:4]}-{d[4:6]}-{d[6:]}" for d in dates]
    spec, st = kept[0]["spec"], _settings(kept[0]["meta"])
    now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    fl = lambda v: f"{v:.6f}"
    L = ["%YAML:1.0", "---",
         f"# Intrinsics of camera {serial}. Written by python/fit_intrinsics.py --write-camera;",
         "# refit rather than edit. One joint Brown-Conrady fit over every frame of the",
         "# sessions below. uncertainty_px: the range of the per-session fits of the",
         "# same sessions (their spread is the repeatability, FINDINGS §16). Pixels;",
         "# dist is OpenCV's order k1 k2 p1 p2 k3.",
         f"camera_serial: {serial}",
         f"model: {_q('Brown-Conrady (5)')}",
         f"image_size: [ {size[0]}, {size[1]} ]",
         *[f"{p}: {fl(v)}" for p, v in zip(PARAMS, (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))],
         "K: [ " + ", ".join(fl(v) for v in K.ravel()) + " ]   # row-major 3 x 3",
         "dist: [ " + ", ".join(f"{v:.8f}" for v in dist) + " ]",
         f"rms_px: {fl(rms)}",
         f"views: {len(img)}",
         "uncertainty_px: { " + ", ".join(f"{p}: {rng[p]:.4f}" for p in PARAMS) + " }",
         f"uncertainty_kind: {_q(f'range of {len(results)} per-session fits' if len(results) > 1 else 'none: one session')}",
         *([f"status: {_q(status)}"] if status else []),
         "sessions:"]
    for r in results:
        L.append(f"  - {{ name: {_q(r['session'])}, views: {r['views']}, rms_px: {r['rms']:.4f}, "
                 + ", ".join(f"{p}: {r[p]:.4f}" for p in PARAMS) + " }")
    L += ["board: { " + ", ".join([
              f"name: {_q(spec.get('name'))}", f"source: {_q(spec.get('source'))}",
              f"rows: {spec.get('rows')}", f"cols: {spec.get('cols')}",
              f"square_mm: {spec['square_mm']['measured']}", f"marker_mm: {spec['marker_mm']['measured']}",
              f"dictionary: {_q(spec.get('dictionary'))}",
              f"legacy_pattern: {int(bool(spec.get('legacy_pattern')))}"]) + " }",
          "settings: { " + ", ".join(f"{k}: {int(v) if isinstance(v, bool) else v}"
                                     for k, v in st.items()) + " }   # read back from the camera",
          f"captured_date: {_q(', '.join(dates))}",
          f"fitted_utc: {_q(now)}",
          f"opencv_version: {_q(cv2.__version__)}"]
    os.makedirs(out_dir, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write("\n".join(L) + "\n")
    os.replace(tmp, path)
    return path, {"rms": float(rms), "views": len(img), "K": K, "dist": dist, "range": rng}


def main(argv):
    ap = argparse.ArgumentParser(description="Final intrinsics fit, and repeatability across sessions.")
    ap.add_argument("sessions", nargs="+", help="session folders of one camera")
    ap.add_argument("--write-camera", action="store_true",
                    help="also fit all sessions' frames jointly and write config/intrinsics/<serial>.yaml")
    ap.add_argument("--force", action="store_true", help="with --write-camera: replace an existing file")
    ap.add_argument("--status", metavar="TEXT",
                    help="with --write-camera: written verbatim as the file's status, "
                         "e.g. 'done, marginal (fx/fy 1.4 px)'")
    args = ap.parse_args(argv)
    sessions = args.sessions
    if args.status and not args.write_camera:
        ap.error("--status needs --write-camera")
    if args.write_camera and not args.force:      # refuse before the slow fits
        serial = json.load(open(os.path.join(sessions[0], "session.json"))).get("camera_serial")
        if serial is not None and os.path.exists(os.path.join(CAMERA_DIR, f"{serial}.yaml")):
            sys.exit(f"fit_intrinsics: {os.path.relpath(os.path.join(CAMERA_DIR, f'{serial}.yaml'))} "
                     "exists; --force to replace it")
    kept = [{} for _ in sessions]
    results = [fit(s, k) for s, k in zip(sessions, kept)]

    print(f"\n{'session':<34} {'views':>6} {'RMS':>8} "
          f"{'fx':>9} {'fy':>9} {'cx':>9} {'cy':>9}")
    print("-" * 90)
    for r in results:
        print(f"{r['session']:<34} {r['views']:>6} {r['rms']:>8.4f} "
              f"{r['fx']:>9.2f} {r['fy']:>9.2f} {r['cx']:>9.2f} {r['cy']:>9.2f}")

    if len(results) > 1:
        print(f"\n{'spread across sessions':<34}")
        for p in ("fx", "fy", "cx", "cy"):
            vals = [r[p] for r in results]
            rng = max(vals) - min(vals)
            print(f"  {p:<6} range {rng:>7.3f} px   "
                  f"mean {np.mean(vals):>9.3f}   "
                  f"({rng / np.mean(vals) * 100:.3f}% of mean)")
        print("\nThis spread is the repeatability of the whole procedure,")
        print("and it bounds how accurate the ground truth can be.")

    if args.write_camera:
        path, j = write_camera(sessions, results, kept, args.force, status=args.status)
        K = j["K"]
        print(f"\njoint fit, all sessions: {j['views']} views, RMS {j['rms']:.4f} px, "
              f"fx {K[0, 0]:.2f} fy {K[1, 1]:.2f} cx {K[0, 2]:.2f} cy {K[1, 2]:.2f}")
        print(f"wrote {os.path.relpath(path)}")


if __name__ == "__main__":
    main(sys.argv[1:])