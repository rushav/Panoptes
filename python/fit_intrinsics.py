"""
Final intrinsics fit, and repeatability across sessions.

Fits Brown-Conrady on every frame of each session (no held-out split -
that was for model selection; here we want the best estimate each
session can give) and compares parameters across sessions.

The spread across independent sessions IS the accuracy figure. Held-out
reprojection error says how well the model fits its own data; it says
nothing about whether a fresh capture would land on the same numbers.

Usage: python fit_intrinsics.py <session_dir> [<session_dir> ...]
"""

import sys, os, json
import numpy as np
import cv2

from compare_models import load_board, detect_all


def fit(session):
    meta = json.load(open(os.path.join(session, "session.json")))
    board, detector, spec = load_board(meta, session)
    img_pts, obj_pts, size = detect_all(session, board, detector)

    # Brown-Conrady (flags=0): chosen after held-out testing showed the
    # rational model's three extra coefficients bought nothing at 82 deg.
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts, img_pts, size, None, None, flags=0)

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


def main(sessions):
    results = [fit(s) for s in sessions]

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


if __name__ == "__main__":
    main(sys.argv[1:])