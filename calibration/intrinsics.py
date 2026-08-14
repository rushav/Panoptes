#!/usr/bin/env python3
"""Per-camera intrinsics from ChArUco captures. OpenCV 5.x API."""
import sys, glob, os, re
import numpy as np, cv2

SQ_X, SQ_Y = 9, 6
SQUARE_M, MARKER_M = 0.090, 0.067
LEGACY = True             # confirmed empirically: board matches legacy layout
MIN_CORNERS = 15

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

def make_board(legacy):
    b = cv2.aruco.CharucoBoard((SQ_X, SQ_Y), SQUARE_M, MARKER_M, d)
    b.setLegacyPattern(legacy)
    return b

def collect(files, board):
    det = cv2.aruco.CharucoDetector(board)
    objp_all, imgp_all, names, size = [], [], [], None
    for f in files:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if img is None: continue
        size = img.shape[::-1]
        cc, ci, mc, mi = det.detectBoard(img)
        if cc is None or len(cc) < MIN_CORNERS: continue
        objp, imgp = board.matchImagePoints(cc, ci)
        if objp is None or len(objp) < MIN_CORNERS: continue
        objp_all.append(objp); imgp_all.append(imgp); names.append(os.path.basename(f))
    return objp_all, imgp_all, names, size

def calibrate(objp, imgp, size):
    return cv2.calibrateCamera(objp, imgp, size, None, None,
                               flags=cv2.CALIB_RATIONAL_MODEL)

capdir = sys.argv[1] if len(sys.argv) > 1 else 'captures'
outdir = sys.argv[2] if len(sys.argv) > 2 else 'calib'
os.makedirs(outdir, exist_ok=True)

files = sorted(glob.glob(os.path.join(capdir, '*.pgm')))
cams  = sorted({re.search(r'cam(\d+)', f).group(1) for f in files})

for sn in cams:
    imgs = [f for f in files if f'cam{sn}' in f]
    board = make_board(LEGACY)
    objp, imgp, names, size = collect(imgs, board)

    print(f"\ncam{sn}: {len(objp)}/{len(imgs)} usable poses")
    if len(objp) < 12:
        print("  too few"); continue

    for it in range(6):
        rms, K, dist, rv, tv = calibrate(objp, imgp, size)
        errs = []
        for j in range(len(objp)):
            proj, _ = cv2.projectPoints(objp[j], rv[j], tv[j], K, dist)
            errs.append(np.linalg.norm(
                proj.reshape(-1,2) - imgp[j].reshape(-1,2), axis=1).mean())
        errs = np.array(errs)
        print(f"  iter {it}: n={len(objp):3d} rms={rms:7.4f} "
              f"worst={errs.max():7.4f} median={np.median(errs):6.4f}")

        if errs.max() < 1.0 or len(objp) < 20: break
        keep = errs < max(1.0, np.median(errs) * 3)
        if keep.all(): break
        objp  = [objp[k]  for k in range(len(objp))  if keep[k]]
        imgp  = [imgp[k]  for k in range(len(imgp))  if keep[k]]
        names = [names[k] for k in range(len(names)) if keep[k]]

    print(f"  FINAL rms={rms:.4f}  fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
          f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}  poses={len(objp)}")

    fs = cv2.FileStorage(os.path.join(outdir, f'cam{sn}.yml'), cv2.FILE_STORAGE_WRITE)
    fs.write('serial', int(sn)); fs.write('image_width', size[0])
    fs.write('image_height', size[1]); fs.write('camera_matrix', K)
    fs.write('distortion_coefficients', dist)
    fs.write('rms_reprojection_error', rms); fs.write('poses_used', len(objp))
    fs.write('legacy_pattern', int(LEGACY))
    fs.release()
