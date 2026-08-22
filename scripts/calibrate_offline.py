#!/usr/bin/env python3
"""Solve a camera calibration from a directory of ChArUco images.

    python3 scripts/calibrate_offline.py --images calib_images

The half of `cameracalibrator` that is slow, run wherever you like. On a Tegra
X1 the in-GUI solve was measured at 18+ minutes and still going; the same work
on a desktop is seconds. It also prints the numbers in the exact form
`sources_real.launch.py` wants.

TWO THINGS IT WORKS OUT RATHER THAN TRUSTING YOU

--size, columns x rows, counting SQUARES. Corner count cannot distinguish 9x11
from 11x9 -- both give (9-1)*(11-1) = 80 -- so each candidate is scored by
fitting a homography from its object points to the detected image points. The
board is planar, so the right answer fits to a fraction of a pixel and the
transpose does not.

The LEGACY PATTERN. OpenCV 4.6 changed which squares of a ChArUco board carry
markers. A board printed to the old convention, detected under 4.6+ without
`setLegacyPattern(True)`, maps its corner IDs to the WRONG object points. It
still detects, still calibrates, and returns a plausible-looking answer that is
wrong. Since the Jetson runs 4.5.4 (legacy) and a desktop is likely on 4.11
(not), this cannot be assumed either way -- so it is scored, same as the size.
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

DICTS = {"4x4_250": cv2.aruco.DICT_4X4_250, "5x5_250": cv2.aruco.DICT_5X5_250,
         "6x6_250": cv2.aruco.DICT_6X6_250, "7x7_250": cv2.aruco.DICT_7X7_250,
         "aruco_orig": cv2.aruco.DICT_ARUCO_ORIGINAL}


def get_dictionary(name):
    d = DICTS[name]
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(d)
    return cv2.aruco.Dictionary_get(d)


def make_board(cols, rows, square, marker, dictionary, legacy):
    if hasattr(cv2.aruco, "CharucoBoard_create"):        # OpenCV < 4.7
        # The pre-4.6 constructor only ever produced the legacy layout, so a
        # request for the new one cannot be honoured here.
        if not legacy:
            return None
        return cv2.aruco.CharucoBoard_create(cols, rows, square, marker,
                                             dictionary)
    board = cv2.aruco.CharucoBoard((cols, rows), square, marker, dictionary)
    if hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(legacy)
    elif legacy:
        return None
    return board


def board_object_points(board):
    return np.asarray(board.getChessboardCorners()
                      if hasattr(board, "getChessboardCorners")
                      else board.chessboardCorners)


def detect_markers(gray, dictionary):
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        corners, ids, _ = det.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=cv2.aruco.DetectorParameters_create())
    return corners, ids


def interpolate(gray, marker_corners, marker_ids, board):
    if marker_ids is None or len(marker_ids) == 0:
        return None, None
    if hasattr(cv2.aruco, "interpolateCornersCharuco"):
        _, c, i = cv2.aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, board, minMarkers=1)
        return c, i
    det = cv2.aruco.CharucoDetector(board)
    c, i, _, _ = det.detectBoard(gray)
    return c, i


def homography_residual(corners, ids, board):
    """Mean reprojection error of a planar homography fit. Sub-pixel = right."""
    if corners is None or len(corners) < 8:
        return float("inf")
    obj = board_object_points(board)
    pts_obj = np.array([obj[int(i)][:2] for i in ids.flatten()], np.float32)
    pts_img = corners.reshape(-1, 2).astype(np.float32)
    H, _ = cv2.findHomography(pts_obj, pts_img, 0)
    if H is None:
        return float("inf")
    proj = cv2.perspectiveTransform(pts_obj.reshape(-1, 1, 2), H).reshape(-1, 2)
    return float(np.mean(np.linalg.norm(proj - pts_img, axis=1)))


def _solve(all_corners, all_ids, board, size):
    """Calibrate, across the OpenCV API break.

    `cv2.aruco.calibrateCameraCharuco` is a contrib-era wrapper. It exists in
    4.5.4 (the Jetson) and in some 4.11 builds, and is GONE from newer
    opencv-python -- "module 'cv2.aruco' has no attribute
    'calibrateCameraCharuco'". The portable replacement is
    `board.matchImagePoints`, which turns charuco corners plus their IDs into
    ordinary 3D/2D correspondences for `cv2.calibrateCamera`. Preferred where
    available, because it is the API that is not going away.
    """
    if hasattr(board, "matchImagePoints"):
        obj_pts, img_pts = [], []
        for corners, ids in zip(all_corners, all_ids):
            o, p = board.matchImagePoints(corners, ids)
            if o is None or len(o) < 6:
                continue
            obj_pts.append(o.astype(np.float32))
            img_pts.append(p.astype(np.float32))
        if len(obj_pts) < 6:
            sys.exit("too few usable views after matching")
        print(f"  (matchImagePoints + calibrateCamera, {len(obj_pts)} views)")
        err, K, D, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size,
                                              None, None)
        return err, K, D

    print(f"  (legacy calibrateCameraCharuco, {len(all_corners)} views)")
    err, K, D, _, _ = cv2.aruco.calibrateCameraCharuco(
        all_corners, all_ids, board, size, None, None)
    return err, K, D


def _coverage_report(all_corners, size):
    """Say what the images actually cover, before believing the answer.

    A calibration is only as good as the spread of views behind it, and a
    tight cluster still solves -- returning confident numbers with no support.
    Same three axes the capture script bins on.
    """
    W, H = size
    pos, fill_bins, tilt_bins = set(), set(), set()
    for c in all_corners:
        pts = c.reshape(-1, 2)
        cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
        w = pts[:, 0].max() - pts[:, 0].min()
        h = pts[:, 1].max() - pts[:, 1].min()
        fill = (w * h) / float(W * H)
        aspect = max(w, h) / max(1.0, min(w, h))
        pos.add((int(cx / (W / 3.0)), int(cy / (H / 3.0))))
        fill_bins.add(0 if fill < 0.25 else (1 if fill < 0.55 else 2))
        tilt_bins.add(0 if aspect < 1.25 else 1)
    print(f"\ncoverage: position {len(pos)}/9   size {len(fill_bins)}/3   "
          f"tilt {len(tilt_bins)}/2")
    if len(pos) < 5:
        print("  position < 5/9: cx and cy are weakly determined -- the board")
        print("  needs to reach all four corners of the frame.")
    if len(fill_bins) < 2:
        print("  size < 2/3: fx and fy cannot be separated from distance.")
        print("  Capture the board both close AND far.")
    if len(tilt_bins) < 2:
        print("  tilt < 2/2: distortion is weakly constrained. Tilt 30-45 deg.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True)
    ap.add_argument("--square", type=float, default=0.022)
    ap.add_argument("--marker", type=float, default=0.016)
    ap.add_argument("--dict", default="4x4_250")
    ap.add_argument("--try-sizes", default="9x11,11x9")
    ap.add_argument("--min-corners", type=int, default=12)
    args = ap.parse_args()

    paths = sorted(sum([glob.glob(os.path.join(args.images, e))
                        for e in ("*.png", "*.jpg", "*.jpeg")], []))
    if not paths:
        sys.exit(f"no images in {args.images}")
    print(f"{len(paths)} images, cv2 {cv2.__version__}")

    dictionary = get_dictionary(args.dict)
    probe = [cv2.imread(p) for p in paths[:min(6, len(paths))]]
    grays = [cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) for im in probe if im is not None]
    if not grays:
        sys.exit("could not read any image")
    size = (grays[0].shape[1], grays[0].shape[0])

    # ── Which (size, legacy) describes this board ───────────────────────────
    print("\nscoring board layout (homography residual, lower is right):")
    best = None
    for spec in args.try_sizes.split(","):
        cols, rows = (int(v) for v in spec.strip().split("x"))
        for legacy in (True, False):
            board = make_board(cols, rows, args.square, args.marker,
                               dictionary, legacy)
            if board is None:
                continue
            res, seen = [], 0
            for gray in grays:
                mc, mi = detect_markers(gray, dictionary)
                c, i = interpolate(gray, mc, mi, board)
                if c is not None and len(c) >= args.min_corners:
                    seen += 1
                    res.append(homography_residual(c, i, board))
            score = float(np.median(res)) if res else float("inf")
            tag = "legacy" if legacy else "new   "
            print(f"  --size {spec.strip():<6} {tag}  "
                  f"{seen}/{len(grays)} views  residual {score:8.3f} px")
            if best is None or score < best[0]:
                best = (score, cols, rows, legacy, spec.strip())

    if best is None or not np.isfinite(best[0]):
        sys.exit("no layout detected the board -- wrong --dict, or bad images")
    score, cols, rows, legacy, spec = best
    if score > 2.0:
        print(f"\nWARNING: best residual is {score:.2f} px, which is high. The")
        print("board parameters may be wrong; the result below is suspect.")
    print(f"\nusing --size {spec}, "
          f"{'LEGACY' if legacy else 'new'} pattern ({score:.3f} px)")

    board = make_board(cols, rows, args.square, args.marker, dictionary, legacy)

    # ── Collect corners from every image ────────────────────────────────────
    all_corners, all_ids, used = [], [], []
    for path in paths:
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mc, mi = detect_markers(gray, dictionary)
        c, i = interpolate(gray, mc, mi, board)
        if c is not None and len(c) >= args.min_corners:
            all_corners.append(c)
            all_ids.append(i)
            used.append(os.path.basename(path))
    print(f"{len(used)}/{len(paths)} images usable "
          f"(>= {args.min_corners} corners)")
    if len(used) < 6:
        sys.exit("too few usable views -- capture more")

    _coverage_report(all_corners, size)

    print("solving...")
    err, K, D = _solve(all_corners, all_ids, board, size)

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    if err < 0.5:
        verdict = "good"
    elif err < 1.0:
        verdict = "acceptable"
    else:
        verdict = "HIGH -- suspect board flatness, see docs/CALIBRATION.md 5"
    print(f"\nreprojection error: {err:.4f} px ({verdict})")
    print(f"resolution        : {size[0]}x{size[1]}")
    print(f"fx {fx:.3f}  fy {fy:.3f}  cx {cx:.3f}  cy {cy:.3f}")
    print("distortion        : " +
          " ".join(f"{v:.6f}" for v in D.flatten()[:5]))

    d = D.flatten()
    print("\nLaunch arguments:\n")
    print(f"  ./scripts/jetson_up.sh \\")
    print(f"      down_cam_calibrated:=true \\")
    print(f"      down_cam_fx:={fx:.4f} down_cam_fy:={fy:.4f} \\")
    print(f"      down_cam_cx:={cx:.4f} down_cam_cy:={cy:.4f} \\")
    print(f"      down_cam_distortion:=\"[{', '.join(f'{v:.6f}' for v in d[:5])}]\"")


if __name__ == "__main__":
    main()
