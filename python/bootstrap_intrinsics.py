"""
Within-session variability: how much do intrinsics move when you refit
the SAME session on different random subsets of its frames?

WHY

Across-session spread mixes two causes: genuine physical change (focus,
a knocked lens) and the sampling noise of which poses happened to be
captured. This isolates the second. If resampling one session
reproduces the across-session spread, sampling explains it and nothing
physical changed. If subsets cluster tightly, the difference is real.

This matters because cx and cy are the weakest-determined parameters in
any calibration - shifting the principal point can be largely
compensated by shifting every board pose, so the optimizer sits in a
shallow valley rather than a sharp minimum. Different pose sets land at
different points in that valley, and fx follows cx.

Usage: python bootstrap_intrinsics.py <session_dir> [n_subsets] [subset_size]
"""

import sys, os, json
import numpy as np
import cv2

from compare_models import load_board, detect_all

SEED = 0


def main(session, n_subsets=20, subset_size=50):
    meta = json.load(open(os.path.join(session, "session.json")))
    board, detector = load_board(meta)
    img_pts, obj_pts, size = detect_all(session, board, detector)

    n = len(img_pts)
    subset_size = min(subset_size, n)
    print(f"{os.path.basename(session)}: {n} views, "
          f"{n_subsets} random subsets of {subset_size}\n")

    rng = np.random.default_rng(SEED)
    rows = []

    for i in range(n_subsets):
        pick = rng.choice(n, subset_size, replace=False)
        op = [obj_pts[j] for j in pick]
        ip = [img_pts[j] for j in pick]

        rms, K, dist, _, _ = cv2.calibrateCamera(op, ip, size, None, None, flags=0)
        rows.append((K[0, 0], K[1, 1], K[0, 2], K[1, 2], rms))

    rows = np.array(rows)
    names = ("fx", "fy", "cx", "cy", "RMS")

    print(f"{'param':<6} {'mean':>10} {'std':>8} {'min':>10} {'max':>10} {'range':>8}")
    print("-" * 56)
    for i, name in enumerate(names):
        col = rows[:, i]
        print(f"{name:<6} {col.mean():>10.3f} {col.std():>8.3f} "
              f"{col.min():>10.3f} {col.max():>10.3f} "
              f"{col.max() - col.min():>8.3f}")

    print("\nCompare these ranges against the across-session spread.")
    print("If they are similar, pose sampling explains the difference")
    print("and no physical change needs to be invoked.")


if __name__ == "__main__":
    session = sys.argv[1]
    n_subsets = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    subset_size = int(sys.argv[3]) if len(sys.argv) > 3 else 50
    main(session, n_subsets, subset_size)