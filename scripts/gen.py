"""Render ChArUco views through a KNOWN camera -- exactly.

A planar target under a PINHOLE camera maps to the image by an exact
homography, so warpPerspective renders it with no approximation. The earlier
version of this file fitted the homography to points that had been pushed
through a distortion model first, which is not a consistent image of anything:
it made the target unrecoverable and looked like a solver bug. Distortion is
therefore zero here, and the test is whether K comes back.
"""
import cv2, numpy as np, os, sys
out = sys.argv[1]
legacy = (len(sys.argv) > 2 and sys.argv[2] == "legacy")
os.makedirs(out, exist_ok=True)
for f in os.listdir(out):
    os.remove(os.path.join(out, f))
W, H = 640, 480
K_true = np.array([[560.0, 0, 322.0], [0, 558.0, 238.0], [0, 0, 1]])
ZERO = np.zeros(5)
# Portable across the 4.6 API break, so the SAME synthetic data can be fed to
# the Jetson's 4.5.4 (legacy-only) and a desktop's 4.11.
if hasattr(cv2.aruco, "getPredefinedDictionary"):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
else:
    d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
if hasattr(cv2.aruco, "CharucoBoard_create"):
    board = cv2.aruco.CharucoBoard_create(9, 11, 0.022, 0.016, d)
    raw = board.draw((900, 1100))
else:
    board = cv2.aruco.CharucoBoard((9, 11), 0.022, 0.016, d)
    if hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(legacy)
    raw = board.generateImage((900, 1100))
img = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
bw, bh = 9 * 0.022, 11 * 0.022
src = np.float32([[0, 0], [img.shape[1], 0],
                  [img.shape[1], img.shape[0]], [0, img.shape[0]]])
obj = np.float32([[0, 0, 0], [bw, 0, 0], [bw, bh, 0], [0, bh, 0]])
rng = np.random.default_rng(7)
n = 0
for _ in range(400):
    if n >= 40:
        break
    rvec = rng.uniform(-0.55, 0.55, 3).astype(np.float64)
    tvec = np.array([rng.uniform(-0.05, 0.05), rng.uniform(-0.04, 0.04),
                     rng.uniform(0.30, 0.75)])
    dst, _ = cv2.projectPoints(obj, rvec, tvec, K_true, ZERO)
    dst = dst.reshape(-1, 2).astype(np.float32)
    # Keep views that are mostly inside the frame; a board half out of view
    # is realistic but makes the ground-truth comparison noisy.
    if dst[:, 0].min() < -60 or dst[:, 1].min() < -60:
        continue
    if dst[:, 0].max() > W + 60 or dst[:, 1].max() > H + 60:
        continue
    Hm = cv2.getPerspectiveTransform(src, dst)
    warp = cv2.warpPerspective(img, Hm, (W, H), borderValue=(255, 255, 255),
                               flags=cv2.INTER_LINEAR)
    cv2.imwrite(os.path.join(out, f"view_{n:03d}.png"), warp)
    n += 1
print(f"generated {n} views, pattern={'legacy' if legacy else 'new'}")
print("TRUTH  fx 560.000  fy 558.000  cx 322.000  cy 238.000  D = 0")
