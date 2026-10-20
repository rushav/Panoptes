"""
Multi-camera point triangulation: linear DLT seed, then Levenberg-Marquardt
on total reprojection error in distorted pixel space.

Everything is batched: M independent triangulations at once, each with up to
C observations selected by a boolean mask (M x C). A masked-out camera
contributes zero rows, so one code path handles every observer count.

Run directly for the self-test. It must pass (zero-noise recovery to
sub-micron) before any number from accuracy_study.py means anything.

Usage: python triangulate.py [layout.yaml]
"""

import os, sys
import numpy as np
import cv2

import rig as rigmod

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LAYOUT = os.path.join(HERE, "..", "config", "layout.yaml")


# ---------------------------------------------------------------- DLT

def dlt(rig, xn, mask, center, scale):
    """
    xn: (M,C,2) undistorted normalised coords; mask: (M,C).
    Each observation gives two rows of A X = 0 (x P3 - P1, y P3 - P2) with
    P = [R|t]; K is already gone because the observations are normalised.

    Conditioning: world points are ~3000 mm from the origin, so A mixes unit
    rotation entries with thousand-mm translations. World coordinates are
    shifted and scaled to ~unit size (Hartley) and each row normalised, so
    the SVD's null vector is not swamped by one column.
    """
    T = np.eye(4)
    T[:3, :3] *= scale
    T[:3, 3] = center                                   # X = T X'
    P = np.concatenate([rig.R, rig.t[:, :, None]], 2) @ T   # C x 3 x 4
    x, y = xn[..., 0, None], xn[..., 1, None]
    rows = np.stack([x * P[:, 2] - P[:, 0], y * P[:, 2] - P[:, 1]], 2)  # M,C,2,4
    rows /= np.maximum(np.linalg.norm(rows, axis=-1, keepdims=True), 1e-300)
    rows *= mask[..., None, None]
    A = rows.reshape(len(xn), -1, 4)
    _, _, Vh = np.linalg.svd(A, full_matrices=False)
    h = Vh[:, -1]
    return h[:, :3] / h[:, 3:] * scale + center


# ---------------------------------------------------------------- refinement

def _residual(rig, X, uv, mask, jacobian=False):
    out = rig.project(X, jacobian)
    r = np.where(mask[..., None], out[0] - uv, 0.0)
    cost = (r ** 2).sum((1, 2))
    # A step that puts the point behind an observing camera is not a
    # solution, whatever the polynomial says; make it unacceptable.
    cost[((out[1] <= 0) & mask).any(1)] = np.inf
    return (r, cost, out[3] * mask[..., None, None]) if jacobian else (r, cost)


def refine(rig, X0, uv, mask, iters=10, tol_mm=1e-10):
    """
    Minimise sum of squared pixel residuals over each point, from X0.
    Per-point damping: shrink on an accepted step, grow on a rejected one.
    Points drop out of the batch once their step is below tol_mm.
    """
    X = X0.copy()
    _, cost = _residual(rig, X, uv, mask)
    lam = np.full(len(X), 1e-3)
    active = np.isfinite(cost)
    for _ in range(iters):
        a = np.flatnonzero(active)
        if a.size == 0:
            break
        r, _, J = _residual(rig, X[a], uv[a], mask[a], jacobian=True)
        H = np.einsum("mcki,mckj->mij", J, J)
        g = np.einsum("mcki,mck->mi", J, r)
        Hd = H + lam[a, None, None] * (np.eye(3) * H)   # Marquardt: scale by diag(H)
        step = -np.linalg.solve(Hd, g[..., None])[..., 0]
        Xn = X[a] + step
        _, cn = _residual(rig, Xn, uv[a], mask[a])
        ok = cn <= cost[a]
        X[a[ok]], cost[a[ok]] = Xn[ok], cn[ok]
        lam[a] = np.where(ok, lam[a] * 0.1, lam[a] * 10)
        active[a] = ~(ok & (np.abs(step).max(1) < tol_mm)) & (lam[a] < 1e8)
    return X, cost


def triangulate(rig, uv, mask, center, scale, iters=10):
    """
    uv: (M,C,2) observed pixels (distorted); mask: (M,C) which are real.
    Returns X (M,3) with NaN rows where fewer than 2 cameras observed,
    the DLT seed, and the observer count so callers can report shortfalls.
    """
    n_obs = mask.sum(1)
    X = np.full((len(uv), 3), np.nan)
    seed = X.copy()
    ok = n_obs >= 2
    if ok.any():
        xn = rig.undistort(uv[ok])                      # rays from undistorted points
        seed[ok] = dlt(rig, xn, mask[ok], center, scale)
        X[ok], _ = refine(rig, seed[ok], uv[ok], mask[ok], iters)
    return X, seed, n_obs


def predicted_cov(rig, X, mask, sigma_px):
    """
    First-order covariance of the refined estimate under isotropic pixel
    noise: sigma^2 (J^T J)^-1, J the stacked projection Jacobian at truth.
    Independent of the Monte Carlo, so agreement between the two checks the
    Monte Carlo machinery as well as the linearisation.
    """
    _, _, _, J = rig.project(X, jacobian=True)
    J = J * mask[..., None, None]
    H = np.einsum("mcki,mckj->mij", J, J)
    cov = np.full(H.shape, np.nan)
    ok = mask.sum(1) >= 2
    cov[ok] = sigma_px ** 2 * np.linalg.inv(H[ok])
    return cov


def volume_frame(info):
    """Centre and scale for DLT conditioning."""
    lo, hi = info["volume_min"], info["volume_max"]
    return (lo + hi) / 2, np.linalg.norm(hi - lo) / 2


# ---------------------------------------------------------------- self-test

def self_test(layout_path=DEFAULT_LAYOUT, verbose=True):
    """Returns True if every check passes. Prints one line per check."""
    rig, info = rigmod.load_layout(layout_path)
    center, scale = volume_frame(info)
    rng = np.random.default_rng(1)
    results = []

    def check(name, value, limit, unit):
        ok = bool(value < limit)
        results.append(ok)
        if verbose:
            print(f"  [{'PASS' if ok else 'FAIL'}] {name:<52} {value:10.3g} {unit:<3} (limit {limit:g})")

    pts = rng.uniform(info["volume_min"], info["volume_max"], (4000, 3))
    vis = rig.visible(pts)

    # 1. Our projection agrees with OpenCV's, so the model is the standard one.
    uv, _, _ = rig.project(pts)
    worst = 0.0
    for c in range(len(rig.names)):
        p = pts[vis[:, c]]
        ref, _ = cv2.projectPoints(p, cv2.Rodrigues(rig.R[c])[0], rig.t[c], rig.K, rig.dist)
        worst = max(worst, np.abs(ref.reshape(-1, 2) - uv[vis[:, c], c]).max())
    check("projection matches cv2.projectPoints", worst, 1e-9, "px")

    # 2. Analytic Jacobian matches central differences.
    p = pts[:200]
    _, _, _, J = rig.project(p, jacobian=True)
    h = 1e-3
    Jfd = np.stack([(rig.project(p + h * e)[0] - rig.project(p - h * e)[0]) / (2 * h)
                    for e in np.eye(3)], -1)
    vm = vis[:200]
    check("Jacobian vs finite difference (relative)",
          np.abs(J - Jfd)[vm].max() / np.abs(J[vm]).max(), 1e-6, "")

    # 3. Undistortion inverts distortion.
    _, _, xn_true = rig.project(pts)
    err = np.abs(rig.undistort(uv) - xn_true)[vis].max()
    check("undistort(distort(x)) round trip, normalised", err, 1e-12, "")

    # 4. Look-at frames: proper rotations, aim point on the optical axis.
    ortho = np.abs(np.einsum("cij,ckj->cik", rig.R, rig.R) - np.eye(3)).max()
    det = np.abs(np.linalg.det(rig.R) - 1).max()
    check("rotations orthonormal, det +1", max(ortho, det), 1e-12, "")
    a_uv, _, _ = rig.project(info["aim"])
    check("aim point lands on principal point",
          np.abs(a_uv - [rig.cx, rig.cy]).max(), 1e-9, "px")
    Rv = rigmod.look_at(np.array([0, 0, 3000.0]), np.zeros(3), np.array([0, 0, 1.0]))
    check("look_at fallback when view is parallel to up",
          np.abs(Rv @ Rv.T - np.eye(3)).max() + abs(np.linalg.det(Rv) - 1)
          + np.isnan(Rv).sum(), 1e-12, "")

    # 5. Zero noise: recovery to numerical precision, every observer.
    ok = vis.sum(1) >= 2
    X, seed, _ = triangulate(rig, uv[ok], vis[ok], center, scale)
    check(f"zero-noise DLT, {ok.sum()} pts, all observers",
          np.linalg.norm(seed - pts[ok], axis=1).max(), 1e-3, "mm")
    check("zero-noise refined, all observers",
          np.linalg.norm(X - pts[ok], axis=1).max(), 1e-3, "mm")

    # 6. Minimum case: exactly two observers per point.
    two = np.zeros_like(vis)
    for i in np.flatnonzero(ok):
        two[i, rng.choice(np.flatnonzero(vis[i]), 2, replace=False)] = True
    X2, _, n2 = triangulate(rig, uv[ok], two[ok], center, scale)
    check("zero-noise refined, exactly 2 observers",
          np.linalg.norm(X2 - pts[ok], axis=1).max(), 1e-3, "mm")

    # 7. Fewer than two: reported, not guessed.
    one = np.zeros_like(vis[:10])
    one[:, 0] = True
    X1, _, n1 = triangulate(rig, uv[:10], one, center, scale)
    check("single observer -> NaN and n_obs=1 (count of violations)",
          (~np.isnan(X1).all(1)).sum() + (n1 != 1).sum(), 0.5, "")

    # 8. With noise, refinement never ends with worse reprojection than DLT.
    noisy = uv[ok] + rng.normal(0, 0.054, uv[ok].shape)
    Xn, sn, _ = triangulate(rig, noisy, vis[ok], center, scale)
    _, c_ref = _residual(rig, Xn, noisy, vis[ok])
    _, c_dlt = _residual(rig, sn, noisy, vis[ok])
    check("noisy: refined cost - DLT cost (max, should be <= 0)",
          (c_ref - c_dlt).max(), 1e-12, "px2")

    # 9. Monte Carlo RMS agrees with the first-order prediction.
    i = np.flatnonzero(vis.sum(1) == vis.sum(1)[ok].max())[0]
    T = 4000
    obs = np.repeat(uv[i:i + 1], T, 0) + rng.normal(0, 0.054, (T,) + uv[i].shape)
    m = np.repeat(vis[i:i + 1], T, 0)
    Xmc, _, _ = triangulate(rig, obs, m, center, scale)
    mc = np.sqrt(((Xmc - pts[i]) ** 2).sum(1).mean())
    pred = np.sqrt(np.trace(predicted_cov(rig, pts[i:i + 1], vis[i:i + 1], 0.054)[0]))
    # 4000 trials: RMS sampling error ~1/sqrt(2*4000) ~ 1.1%, so 5% is ~4 sigma.
    check(f"MC RMS / first-order prediction - 1 ({mc*1000:.1f} vs {pred*1000:.1f} um)",
          abs(mc / pred - 1), 0.05, "")

    passed = all(results)
    if verbose:
        print(f"  self-test: {sum(results)}/{len(results)} passed")
    return passed


if __name__ == "__main__":
    print("triangulate.py self-test")
    sys.exit(0 if self_test(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LAYOUT) else 1)
