#!/usr/bin/env python3
"""Global bundle adjustment over camera poses and board poses.

Refines all camera extrinsics and all per-capture board poses jointly by
minimising reprojection error across every observed ChArUco corner.
Intrinsics are held fixed. The reference camera is held at the origin to
fix the gauge freedom.
"""
import sys, glob, os, re
from collections import defaultdict
import numpy as np, cv2
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

SQ_X, SQ_Y = 9, 6
SQUARE_M, MARKER_M = 0.090, 0.067
LEGACY = True
MIN_CORNERS = 15
REFERENCE = '33661'

capdir   = sys.argv[1] if len(sys.argv) > 1 else 'captures'
calibdir = sys.argv[2] if len(sys.argv) > 2 else 'calib'

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
board = cv2.aruco.CharucoBoard((SQ_X, SQ_Y), SQUARE_M, MARKER_M, d)
board.setLegacyPattern(LEGACY)
detector = cv2.aruco.CharucoDetector(board)
BOARD_PTS = board.getChessboardCorners()          # (N,3) in board frame

def pose_to_vec(T):
    return np.concatenate([cv2.Rodrigues(T[:3,:3])[0].ravel(), T[:3,3]])

def vec_to_pose(v):
    T = np.eye(4)
    T[:3,:3] = cv2.Rodrigues(v[:3])[0]
    T[:3, 3] = v[3:]
    return T

# ---- load intrinsics -----------------------------------------------------
def load_intr(sn):
    fs = cv2.FileStorage(os.path.join(calibdir, f'cam{sn}.yml'), cv2.FILE_STORAGE_READ)
    K = fs.getNode('camera_matrix').mat()
    D = fs.getNode('distortion_coefficients').mat()
    fs.release()
    return K, D

files = sorted(glob.glob(os.path.join(capdir, '*.pgm')))
cams  = sorted({re.search(r'cam(\d+)', f).group(1) for f in files})
intr  = {sn: load_intr(sn) for sn in cams}

# ---- collect observations -----------------------------------------------
obs = []                                   # (cap_idx, cam_idx, ids, pts)
cam_index = {sn: i for i, sn in enumerate(cams)}
pnp = defaultdict(dict)

for f in files:
    m = re.search(r'pose(\d+)_cam(\d+)', f)
    if not m: continue
    cap, sn = m.group(1), m.group(2)

    img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
    cc, ci, _, _ = detector.detectBoard(img)
    if cc is None or len(cc) < MIN_CORNERS: continue

    objp, imgp = board.matchImagePoints(cc, ci)
    if objp is None or len(objp) < MIN_CORNERS: continue

    K, D = intr[sn]
    ok, rvec, tvec = cv2.solvePnP(objp, imgp, K, D)
    if not ok: continue

    ids = ci.ravel().astype(int)
    obs.append((cap, sn, ids, imgp.reshape(-1,2)))
    T = np.eye(4); T[:3,:3] = cv2.Rodrigues(rvec)[0]; T[:3,3] = tvec.ravel()
    pnp[cap][sn] = T

caps = sorted(pnp.keys())
cap_index = {c: i for i, c in enumerate(caps)}
print(f"{len(cams)} cameras, {len(caps)} captures, {len(obs)} observations, "
      f"{sum(len(o[2]) for o in obs)} corner measurements")

# ---- initial values ------------------------------------------------------
ext = np.load(os.path.join(calibdir, 'extrinsics.npz'))
T_cam_world = {sn: ext[f'cam{sn}'] for sn in cams}    # world -> camera

T_world_board = {}
for c in caps:
    sn = next(iter(pnp[c]))
    T_world_board[c] = np.linalg.inv(T_cam_world[sn]) @ pnp[c][sn]

n_cam, n_cap = len(cams), len(caps)
x0 = np.concatenate(
    [pose_to_vec(T_cam_world[sn]) for sn in cams] +
    [pose_to_vec(T_world_board[c]) for c in caps])

ref_i = cam_index[REFERENCE]

def residuals(x):
    cam_T = [vec_to_pose(x[6*i:6*i+6]) for i in range(n_cam)]
    cap_T = [vec_to_pose(x[6*n_cam+6*j : 6*n_cam+6*j+6]) for j in range(n_cap)]
    out = []
    for cap, sn, ids, pts in obs:
        i, j = cam_index[sn], cap_index[cap]
        T = cam_T[i] @ cap_T[j]
        K, D = intr[sn]
        proj, _ = cv2.projectPoints(BOARD_PTS[ids],
                                    cv2.Rodrigues(T[:3,:3])[0], T[:3,3], K, D)
        out.append((proj.reshape(-1,2) - pts).ravel())
    return np.concatenate(out)

def sparsity():
    rows = sum(2*len(o[2]) for o in obs)
    S = lil_matrix((rows, len(x0)), dtype=int)
    r = 0
    for cap, sn, ids, _ in obs:
        n = 2*len(ids)
        i, j = cam_index[sn], cap_index[cap]
        if i != ref_i:
            S[r:r+n, 6*i:6*i+6] = 1
        S[r:r+n, 6*n_cam+6*j : 6*n_cam+6*j+6] = 1
        r += n
    return S

r0 = residuals(x0)
print(f"\ninitial rms reprojection: {np.sqrt(np.mean(r0**2)):.4f} px")

res = least_squares(residuals, x0, jac_sparsity=sparsity(),
                    verbose=2, x_scale='jac', ftol=1e-8, method='trf',
                    loss='huber', f_scale=1.0, max_nfev=200)

print(f"\nfinal rms reprojection:   {np.sqrt(np.mean(res.fun**2)):.4f} px")

# per-observation error, to find which captures are bad
r = res.fun.reshape(-1, 2)
e = np.linalg.norm(r, axis=1)
print(f"corner error: median={np.median(e):.3f} p90={np.percentile(e,90):.3f} "
      f"p99={np.percentile(e,99):.3f} max={e.max():.3f} px")

k, per = 0, []
for cap, sn, ids, _ in obs:
    n = len(ids)
    per.append((np.linalg.norm(r[k:k+n], axis=1).mean(), cap, sn, n))
    k += n
per.sort(reverse=True)
print("\nworst observations:")
for err, cap, sn, n in per[:12]:
    print(f"  pose{cap} cam{sn}: {err:6.3f} px ({n} corners)")

# --- reject outlier observations and re-solve ---------------------------
THRESH = 2.0
bad = {(cap, sn) for err, cap, sn, _ in per if err > THRESH}
if bad:
    print(f"\nrejecting {len(bad)}/{len(obs)} observations above {THRESH} px, re-solving")
    obs = [o for o in obs if (o[0], o[1]) not in bad]
    caps = sorted({o[0] for o in obs})
    cap_index = {c: i for i, c in enumerate(caps)}
    n_cap = len(caps)
    x_prev = res.x
    x0 = np.concatenate(
        [x_prev[6*i:6*i+6] for i in range(n_cam)] +
        [x_prev[6*n_cam+6*j : 6*n_cam+6*j+6]
         for j in [list(pnp.keys()).index(c) if False else k
                   for k, c in enumerate(sorted(pnp.keys())) if c in cap_index]])
    res = least_squares(residuals, x0, jac_sparsity=sparsity(),
                        verbose=0, x_scale='jac', ftol=1e-8, method='trf',
                        max_nfev=200)
    r = res.fun.reshape(-1, 2)
    e = np.linalg.norm(r, axis=1)
    print(f"after rejection: rms={np.sqrt(np.mean(res.fun**2)):.4f} px  "
          f"median={np.median(e):.3f}  p99={np.percentile(e,99):.3f}  max={e.max():.3f}")

# ---- report --------------------------------------------------------------
x = res.x
T_ref = vec_to_pose(x[6*ref_i:6*ref_i+6])
out = {}
print("\ncamera positions (world frame, reference at origin):")
for sn in cams:
    i = cam_index[sn]
    T = vec_to_pose(x[6*i:6*i+6]) @ np.linalg.inv(T_ref)   # re-anchor on reference
    out[f'cam{sn}'] = T
    p = np.linalg.inv(T)[:3,3]
    d0 = np.linalg.inv(T_cam_world[sn])[:3,3]
    print(f"  {sn}: [{p[0]:+.4f} {p[1]:+.4f} {p[2]:+.4f}] m   "
          f"moved {np.linalg.norm(p-d0)*1000:5.1f} mm")

print("\npairwise baselines:")
for a in range(len(cams)):
    for b in range(a+1, len(cams)):
        pa = np.linalg.inv(out[f'cam{cams[a]}'])[:3,3]
        pb = np.linalg.inv(out[f'cam{cams[b]}'])[:3,3]
        print(f"  {cams[a]} - {cams[b]}: {np.linalg.norm(pa-pb):.4f} m")

np.savez(os.path.join(calibdir, 'extrinsics_ba.npz'), **out)
print(f"\nwrote {calibdir}/extrinsics_ba.npz")
