#!/usr/bin/env python3
"""Pairwise extrinsics from shared ChArUco views, chained to a common frame.

Board pose is solved per camera per capture. For any two cameras seeing the
same capture, the relative transform is T_BA = T_B_board @ inv(T_A_board).
These are averaged over all shared poses, then chained from a reference camera.
"""
import sys, glob, os, re, itertools
from collections import defaultdict
import numpy as np, cv2

SQ_X, SQ_Y = 9, 6
SQUARE_M, MARKER_M = 0.090, 0.067
LEGACY = True
MIN_CORNERS = 15
REFERENCE = '33661'                       # world origin
EDGES = [('33661','33663'), ('33663','33659'),
         ('33659','33277'), ('33277','33661')]

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
board = cv2.aruco.CharucoBoard((SQ_X, SQ_Y), SQUARE_M, MARKER_M, d)
board.setLegacyPattern(LEGACY)
detector = cv2.aruco.CharucoDetector(board)

capdir = sys.argv[1] if len(sys.argv) > 1 else 'captures'
calibdir = sys.argv[2] if len(sys.argv) > 2 else 'calib'

def load_intrinsics(sn):
    fs = cv2.FileStorage(os.path.join(calibdir, f'cam{sn}.yml'), cv2.FILE_STORAGE_READ)
    K = fs.getNode('camera_matrix').mat()
    D = fs.getNode('distortion_coefficients').mat()
    fs.release()
    return K, D

def rt_to_T(rvec, tvec):
    T = np.eye(4)
    T[:3,:3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    return T

def average_transforms(Ts):
    """Mean rotation via SVD projection onto SO(3); median translation."""
    R = np.mean([T[:3,:3] for T in Ts], axis=0)
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    t = np.median([T[:3,3] for T in Ts], axis=0)
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = t
    return T

# --- solve board pose for every (capture, camera) -------------------------
files = sorted(glob.glob(os.path.join(capdir, '*.pgm')))
cams = sorted({re.search(r'cam(\d+)', f).group(1) for f in files})
intr = {sn: load_intrinsics(sn) for sn in cams}

poses = defaultdict(dict)          # poses[capture_index][serial] = T_cam_board
for f in files:
    m = re.search(r'pose(\d+)_cam(\d+)', f)
    if not m: continue
    idx, sn = m.group(1), m.group(2)

    img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
    cc, ci, _, _ = detector.detectBoard(img)
    if cc is None or len(cc) < MIN_CORNERS: continue

    objp, imgp = board.matchImagePoints(cc, ci)
    if objp is None or len(objp) < MIN_CORNERS: continue

    K, D = intr[sn]
    ok, rvec, tvec = cv2.solvePnP(objp, imgp, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if ok:
        poses[idx][sn] = rt_to_T(rvec, tvec)

print(f"solved board pose in {sum(len(v) for v in poses.values())} images\n")

# --- pairwise relative transforms ----------------------------------------
rel = {}
for a, b in itertools.combinations(cams, 2):
    Ts = [poses[i][b] @ np.linalg.inv(poses[i][a])
          for i in poses if a in poses[i] and b in poses[i]]
    if len(Ts) < 5: continue

    T = average_transforms(Ts)
    # spread of the individual estimates about the mean
    dt = np.array([np.linalg.norm(x[:3,3] - T[:3,3]) for x in Ts])
    dr = np.array([np.degrees(np.linalg.norm(cv2.Rodrigues(x[:3,:3] @ T[:3,:3].T)[0]))
                   for x in Ts])
    rel[(a,b)] = T
    rel[(b,a)] = np.linalg.inv(T)
    print(f"{a} -> {b}: n={len(Ts):3d}  baseline={np.linalg.norm(T[:3,3]):.4f} m  "
          f"scatter t={dt.mean()*1000:5.1f} mm  r={dr.mean():5.2f} deg")

# --- chain to a common frame ---------------------------------------------
print(f"\nchaining from {REFERENCE}")
world = {REFERENCE: np.eye(4)}
order = [REFERENCE]
for _ in range(len(cams)):
    for a, b in EDGES:
        for x, y in ((a,b), (b,a)):
            if x in world and y not in world and (x,y) in rel:
                world[y] = rel[(x,y)] @ world[x]
                order.append(y)

for sn in cams:
    if sn not in world:
        print(f"  {sn}: UNREACHABLE"); continue
    T = np.linalg.inv(world[sn])       # camera position in world frame
    print(f"  {sn}: position [{T[0,3]:+.4f} {T[1,3]:+.4f} {T[2,3]:+.4f}] m")

# --- loop closure ---------------------------------------------------------
print("\nloop closure A->B->C->D->A")
T = np.eye(4)
for a, b in EDGES:
    if (a,b) not in rel:
        print(f"  missing edge {a}->{b}"); T = None; break
    T = rel[(a,b)] @ T
if T is not None:
    dt = np.linalg.norm(T[:3,3])
    dr = np.degrees(np.linalg.norm(cv2.Rodrigues(T[:3,:3])[0]))
    print(f"  translation error {dt*1000:.1f} mm")
    print(f"  rotation error    {dr:.3f} deg")

np.savez(os.path.join(calibdir, 'extrinsics.npz'),
         **{f'cam{sn}': world[sn] for sn in world})
print(f"\nwrote {calibdir}/extrinsics.npz")
