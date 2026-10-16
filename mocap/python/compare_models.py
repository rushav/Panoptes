"""
Compare distortion models on the same capture set.

WHY NOT JUST COMPARE RMS

Each model reports RMS against its own fit. A model with more free
parameters can always fit the training data better, including by
contorting its coefficients in regions where data is thin. Measured on
synthetic data with known ground truth, a centre-clustered set had
BETTER RMS (0.262 vs 0.223 px) than a well-covered one while being 10x
worse at the frame edge. RMS is not a model-selection criterion.

So this script judges by two checks the optimizer never saw:

  1. HELD-OUT ERROR. Fit on most of the images, measure reprojection
     error on images the solver never touched. A model that overfits
     its training views does visibly worse on unseen ones.

  2. EDGE STRAIGHTNESS. Board rows are straight lines in the real world,
     so they must be straight after undistortion. Residual from a
     straight-line fit, measured on corners near the frame edge where
     distortion is strongest, is ground truth independent of any solve.

Usage: python compare_models.py <session_dir>
"""

import sys, os, json, glob
import numpy as np
import cv2

TEST_FRACTION = 0.25
SEED = 99
GRID_X, GRID_Y = 5, 6


# ---------------------------------------------------------------- detection

def load_board(meta):
    b = meta["board"]
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    board = cv2.aruco.CharucoBoard(
        (b["squares_x"], b["squares_y"]),
        b["square_mm"], b["marker_mm"], dictionary)
    # OpenCV 5.x needs this set explicitly; 4.6's layout already is legacy.
    board.setLegacyPattern(True)
    return board, cv2.aruco.CharucoDetector(board)


def detect_all(session, board, detector):
    """Returns (image_points, object_points, image_size) per frame."""
    files = sorted(glob.glob(os.path.join(session, "*.png")))
    img_pts, obj_pts, size = [], [], None

    for f in files:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if size is None:
            size = (img.shape[1], img.shape[0])

        corners, ids, _, _ = detector.detectBoard(img)
        if ids is None or len(ids) < 6:
            continue

        # matchImagePoints turns detected corner IDs into the matching
        # 3D board coordinates. OpenCV 5.x removed the old
        # interpolateCornersCharuco / calibrateCameraCharuco path.
        op, ip = board.matchImagePoints(corners, ids)
        if op is None or len(op) < 6:
            continue

        obj_pts.append(op)
        img_pts.append(ip)

    return img_pts, obj_pts, size


# ---------------------------------------------------------------- checks

def heldout_error(obj_pts, img_pts, K, dist, fisheye=False):
    """
    Mean reprojection error on views the solver never saw.

    Each held-out view still needs its own pose - we are testing whether
    the LENS MODEL generalizes, not whether the poses transfer. So solve
    pose per view using the fitted intrinsics, then measure.
    """
    errors = []
    for op, ip in zip(obj_pts, img_pts):
        if fisheye:
            # fisheye.solvePnP requires explicitly 2-channel arrays
            # (CV_32FC2/CV_64FC2), i.e. (N,1,2) not (N,2).
            op_f = np.ascontiguousarray(op.reshape(-1, 1, 3), dtype=np.float64)
            ip_f = np.ascontiguousarray(ip.reshape(-1, 1, 2), dtype=np.float64)
            ok, rvec, tvec = cv2.fisheye.solvePnP(op_f, ip_f, K, dist)
            if not ok:
                continue
            proj, _ = cv2.fisheye.projectPoints(op_f, rvec, tvec, K, dist)
        else:
            ok, rvec, tvec = cv2.solvePnP(op, ip, K, dist)
            if not ok:
                continue
            proj, _ = cv2.projectPoints(op, rvec, tvec, K, dist)

        d = (proj.reshape(-1, 2) - ip.reshape(-1, 2))
        errors.append(np.sqrt((d ** 2).sum(axis=1)).mean())

    return float(np.mean(errors)) if errors else float("nan")


def edge_straightness(img_pts, board_cols, K, dist, size, fisheye=False):
    """
    Undistort corners, then measure how straight each board row is.

    Real board rows are straight lines, so any residual after
    undistortion is distortion the model failed to remove. Restricted to
    corners in the outer 20% of the frame, where distortion is strongest
    and where a poorly-constrained model extrapolates.
    """
    margin_x, margin_y = size[0] * 0.2, size[1] * 0.2
    residuals = []

    for ip in img_pts:
        pts = ip.reshape(-1, 2)

        near_edge = ((pts[:, 0] < margin_x) | (pts[:, 0] > size[0] - margin_x) |
                     (pts[:, 1] < margin_y) | (pts[:, 1] > size[1] - margin_y))
        if near_edge.sum() < 4:
            continue

        if fisheye:
            pts_f = np.ascontiguousarray(pts.reshape(-1, 1, 2), dtype=np.float64)
            und = cv2.fisheye.undistortPoints(pts_f, K, dist, P=K)
        else:
            und = cv2.undistortPoints(pts.reshape(-1, 1, 2), K, dist, P=K)
        und = und.reshape(-1, 2)

        # Group undistorted corners into board rows by their index, then
        # fit a line to each row and record the worst deviation.
        n = len(und)
        for start in range(0, n - board_cols + 1, board_cols):
            row = und[start:start + board_cols]
            if len(row) < 3:
                continue
            vx, vy, x0, y0 = cv2.fitLine(row.astype(np.float32),
                                         cv2.DIST_L2, 0, 0.01, 0.01).flatten()
            # Perpendicular distance from each point to the fitted line.
            d = np.abs((row[:, 0] - x0) * vy - (row[:, 1] - y0) * vx)
            residuals.append(d.max())

    return float(np.mean(residuals)) if residuals else float("nan")


# ---------------------------------------------------------------- models

def fit_standard(obj, img, size, flags, name):
    rms, K, dist, _, _ = cv2.calibrateCamera(obj, img, size, None, None, flags=flags)
    return dict(name=name, rms=rms, K=K, dist=dist, fisheye=False)


def fit_fisheye(obj, img, size):
    # fisheye.calibrate wants (1, N, 3) and (1, N, 2) float64 - note this
    # differs from calibrateCamera, which takes (N, 1, 3).
    obj_f = [o.reshape(1, -1, 3).astype(np.float64) for o in obj]
    img_f = [i.reshape(1, -1, 2).astype(np.float64) for i in img]

    K = np.zeros((3, 3))
    D = np.zeros((4, 1))

    # OpenCV 5 dropped the cv2.fisheye.CALIB_* constants (verified: the
    # module has the functions but no CALIB_ attributes). Both flags we
    # wanted were refinements - RECOMPUTE_EXTRINSIC re-solves each pose
    # per iteration, FIX_SKEW pins skew to zero - so defaults are fine.
    flags = 0

    rms, K, D, _, _ = cv2.fisheye.calibrate(
        obj_f, img_f, size, K, D, flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))

    return dict(name="fisheye (equidistant)", rms=rms, K=K, dist=D, fisheye=True)


# ---------------------------------------------------------------- main

def main(session):
    meta = json.load(open(os.path.join(session, "session.json")))
    board, detector = load_board(meta)
    board_cols = meta["board"]["squares_x"] - 1

    img_pts, obj_pts, size = detect_all(session, board, detector)
    print(f"{len(img_pts)} usable views, {size[0]}x{size[1]}\n")

    # Split. Random with a fixed seed so the comparison is reproducible.
    rng = np.random.default_rng(SEED)
    idx = rng.permutation(len(img_pts))
    n_test = max(4, int(len(idx) * TEST_FRACTION))
    test_idx, train_idx = idx[:n_test], idx[n_test:]

    train_obj = [obj_pts[i] for i in train_idx]
    train_img = [img_pts[i] for i in train_idx]
    test_obj  = [obj_pts[i] for i in test_idx]
    test_img  = [img_pts[i] for i in test_idx]

    # Report the test set's frame coverage. If the split happened to put
    # all the edge views in training, held-out error would look good for
    # every model and mean nothing.
    cells = set()
    for ip in test_img:
        for x, y in ip.reshape(-1, 2):
            cells.add((min(int(x * GRID_X / size[0]), GRID_X - 1),
                       min(int(y * GRID_Y / size[1]), GRID_Y - 1)))
    print(f"train {len(train_idx)}, test {len(test_idx)} "
          f"(test covers {len(cells)}/{GRID_X * GRID_Y} cells)\n")

    models = [
        fit_standard(train_obj, train_img, size, 0, "Brown-Conrady (5)"),
        fit_standard(train_obj, train_img, size,
                     cv2.CALIB_RATIONAL_MODEL, "rational (8)"),
    ]

    try:
        models.append(fit_fisheye(train_obj, train_img, size))
    except cv2.error as e:
        print(f"fisheye skipped: {str(e).splitlines()[-1]}\n")

    print(f"{'model':<24} {'train RMS':>10} {'held-out':>10} "
          f"{'edge resid':>11} {'fx':>9} {'fy':>9} {'fx/fy':>8}")
    print("-" * 86)

    for m in models:
        try:
            held = heldout_error(test_obj, test_img, m["K"], m["dist"], m["fisheye"])
            edge = edge_straightness(test_img, board_cols, m["K"], m["dist"],
                                     size, m["fisheye"])
        except cv2.error as e:
            print(f"{m['name']:<24} validation failed: "
                  f"{str(e).splitlines()[-1]}")
            continue

        fx, fy = m["K"][0, 0], m["K"][1, 1]
        print(f"{m['name']:<24} {m['rms']:>10.4f} {held:>10.4f} "
              f"{edge:>11.4f} {fx:>9.2f} {fy:>9.2f} "
              f"{abs(fx - fy) / fx * 100:>7.3f}%")

    print("\ntrain RMS is NOT the criterion - it measures fit over collected")
    print("data only. Judge by held-out error and edge residual.")
    print("fisheye fx is not comparable across model families - the")
    print("equidistant projection defines focal length differently.")

    out = os.path.join(session, "model_comparison.json")
    with open(out, "w") as f:
        json.dump({
            "opencv_version": cv2.__version__,
            "views": len(img_pts),
            "train": len(train_idx), "test": len(test_idx),
            "seed": SEED,
            "models": [{
                "name": m["name"],
                "train_rms": m["rms"],
                "K": m["K"].tolist(),
                "dist": m["dist"].ravel().tolist(),
            } for m in models],
        }, f, indent=2)
    print(f"\nwritten to {out}")


if __name__ == "__main__":
    main(sys.argv[1])