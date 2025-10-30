import os
import sys
import glob
import numpy as np

import cv2

# settings
imgs_glob = os.path.expanduser('./dataset/*.jpg')
CHECKERBOARD = (10, 7)  # number of inner corners per row and column of the checkerboard
SQUARE_SIZE = 1.0
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
img_size = None

# cumulative vectors
objpoints = []  # 3D points in real world space, shape (1, N, 3)
imgpoints = []  # 2D points in image plane, shape (1, N, 2)
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), dtype=np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# iterate over images to detect and refine checkerboard corners
images = sorted(glob.glob(imgs_glob))
if not images:
    raise FileNotFoundError(f'No images found for glob: {imgs_glob}')

for fn in images:
    img = cv2.imread(fn)
    if img is None:
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if img_size is None:
        img_size = (gray.shape[1], gray.shape[0])  # (w, h)

    found, corners = cv2.findChessboardCorners(
        gray, CHECKERBOARD,
        flags=(cv2.CALIB_CB_ADAPTIVE_THRESH |
               cv2.CALIB_CB_FAST_CHECK     |
               cv2.CALIB_CB_NORMALIZE_IMAGE)
    )

    if not found:
        print(f'[X] Corners not found: {fn}')
        continue

    corners_refined = cv2.cornerSubPix(
        gray, corners, winSize=(11, 11), zeroZone=(-1, -1), criteria=criteria
    )

    # fisheye module prefers a (1, N, 2) shape
    imgpoints.append(corners_refined.reshape(1, -1, 2))
    objpoints.append(objp.copy())

    # visualization (optional)
    vis = cv2.drawChessboardCorners(img.copy(), CHECKERBOARD, corners_refined, found)
    cv2.imshow('corners', vis)
    cv2.waitKey(1)

cv2.destroyAllWindows()

if len(imgpoints) == 0:
    raise RuntimeError('No chessboard was detected in any image.')

# calibration for fisheye
K = np.eye(3, dtype=np.float64)         # 3x3 intrinsic matrix
D = np.zeros((4, 1), dtype=np.float64)  # 4x1 distortion coefficients (k1, k2, k3, k4)

rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(imgpoints))]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(imgpoints))]

flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
         cv2.fisheye.CALIB_CHECK_COND         |
         cv2.fisheye.CALIB_FIX_SKEW)

rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
    objectPoints=objpoints,
    imagePoints=imgpoints,
    image_size=img_size,
    K=K,
    D=D,
    rvecs=rvecs,
    tvecs=tvecs,
    flags=flags,
    criteria=criteria
)

print("retval (RMS reprojection error): \n", rms)
print("K: \n", K)
print("D: \n", D.ravel())
