#!/usr/bin/env python3
import sys, glob, re
import numpy as np, cv2

SQ_X, SQ_Y = 9, 6
SQUARE_M, MARKER_M = 0.090, 0.067
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

files = sorted(glob.glob('captures/*cam33661*.pgm'))[:25]

for label, sx, sy in [("9x6", SQ_X, SQ_Y), ("6x9", SQ_Y, SQ_X)]:
    board = cv2.aruco.CharucoBoard_create(sx, sy, SQUARE_M, MARKER_M, d)
    cs, ids_, size = [], [], None
    for f in files:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        size = img.shape[::-1]
        c, i, _ = cv2.aruco.detectMarkers(img, d)
        if i is None or len(i) < 6: continue
        n, cc, ci = cv2.aruco.interpolateCornersCharuco(c, i, img, board)
        if n is not None and n >= 12:
            cs.append(cc); ids_.append(ci)
    if len(cs) < 8:
        print(f"{label}: only {len(cs)} poses"); continue
    rms, K, dist, _, _ = cv2.aruco.calibrateCameraCharuco(cs, ids_, board, size, None, None)
    print(f"{label}: poses={len(cs):3d}  rms={rms:8.4f}  fx={K[0,0]:7.1f} fy={K[1,1]:7.1f} "
          f"cx={K[0,2]:6.1f} cy={K[1,2]:6.1f}")
