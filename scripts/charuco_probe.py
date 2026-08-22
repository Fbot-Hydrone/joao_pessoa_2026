#!/usr/bin/env python3
"""Work out a ChArUco board's parameters from what the camera actually sees.

    ros2 run  -- no.  Just:  python3 scripts/charuco_probe.py [--device PATH]

`cameracalibrator` needs four numbers about the board and gets no feedback if
any of them is wrong -- it simply never registers a sample, which looks exactly
like a camera or lighting problem. This checks them against the real thing
first:

  --aruco_dict          by trying every 4x4/5x5/6x6/7x7 predefined dictionary
                        and reporting which one detects markers at all
  --size                by counting squares. NOTE this is the footgun: for
                        `-p charuco` the size is the number of SQUARES, not
                        the interior corners a plain chessboard wants.
                        camera_calibration passes it straight to
                        cv2.aruco.CharucoBoard (calibrator.py:93) and then
                        expects (x-1)*(y-1) corners (calibrator.py:125).
  --square / -m         cannot be measured from an image -- they set the scale,
                        which is unobservable monocularly. Read them off the
                        board's own footer, or a ruler.

It also reports how much of the board is visible, because a board detected in
one frame at one angle proves the parameters and nothing else.
"""
import argparse
import sys

import cv2
import numpy as np

# The dictionaries cameracalibrator will accept, mapped to its own spelling.
CANDIDATES = [
    ("4x4_250", cv2.aruco.DICT_4X4_250),
    ("5x5_250", cv2.aruco.DICT_5X5_250),
    ("6x6_250", cv2.aruco.DICT_6X6_250),
    ("7x7_250", cv2.aruco.DICT_7X7_250),
    ("aruco_orig", cv2.aruco.DICT_ARUCO_ORIGINAL),
]


def get_dictionary(dict_id):
    """cv2.aruco's API changed name between 4.5 and 4.7; support both."""
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


def detect(gray, dictionary):
    """Return (corners, ids) across the 4.5 / 4.7+ API split."""
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        corners, ids, _ = det.detectMarkers(gray)
    else:
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary,
                                                  parameters=params)
    return corners, ids


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--image", default="",
                    help="read a still image instead of the camera; also how "
                         "this script is self-tested against a synthetic board")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--frames", type=int, default=40,
                    help="frames to sample; the best one is reported")
    ap.add_argument("--save", default="",
                    help="write the best frame here, annotated")
    ap.add_argument("--square", type=float, default=0.022,
                    help="checker square side, metres (from the board footer)")
    ap.add_argument("--marker", type=float, default=0.016,
                    help="ArUco marker side, metres (from the board footer)")
    ap.add_argument("--try-sizes", default="9x11,11x9",
                    help="candidate --size values to score against the image")
    args = ap.parse_args()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            sys.exit(f"cannot read {args.image}")
        args.width, args.height = frame.shape[1], frame.shape[0]
        frames = [frame]
    else:
        frames = None

    cap = None
    if frames is None:
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            sys.exit(f"cannot open {args.device}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    best = {"n": -1}
    tally = {name: 0 for name, _ in CANDIDATES}
    ids_seen = set()

    for i in range(1 if frames else args.frames):
        if frames:
            frame = frames[0]
        else:
            ok, frame = cap.read()
            if not ok:
                continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for name, dict_id in CANDIDATES:
            corners, ids = detect(gray, get_dictionary(dict_id))
            n = 0 if ids is None else len(ids)
            if n:
                tally[name] += n
                if name == max(tally, key=tally.get):
                    ids_seen.update(int(i) for i in ids.flatten())
                if n > best["n"]:
                    best = {"n": n, "frame": frame, "corners": corners,
                            "ids": ids, "dict": name}
    if cap is not None:
        cap.release()

    if best["n"] <= 0:
        print("No ArUco markers detected in any dictionary.")
        print("  * is the board in view and roughly in focus?")
        print("  * is there enough light? MJPG under-exposes badly indoors.")
        sys.exit(1)

    print(f"resolution      : {args.width}x{args.height}")
    print("dictionary votes: " + ", ".join(
        f"{k}={v}" for k, v in sorted(tally.items(), key=lambda kv: -kv[1])))
    print(f"--aruco_dict    : {best['dict']}   <-- use this")
    print(f"markers, best frame: {best['n']}")

    if ids_seen:
        lo, hi = min(ids_seen), max(ids_seen)
        print(f"marker IDs seen : {lo}..{hi}  ({len(ids_seen)} distinct)")
        # A ChArUco board carries one marker per light square, so the total
        # square count is about twice the highest ID. That is enough to tell
        # 9x11 (99 squares, IDs 0..48) from a differently-shaped board, and
        # it is a check on the number printed on the target rather than a
        # replacement for it.
        implied = (hi + 1) * 2
        print(f"implied squares : ~{implied} "
              f"(2 x (max ID + 1)); 9x11 = 99, 8x11 = 88, 7x9 = 63")

    # ── Which --size actually interpolates corners ──────────────────────────
    # The decisive test. cameracalibrator builds cv2.aruco.CharucoBoard from
    # --size and then interpolates chessboard corners from the detected
    # markers; if the geometry is wrong the interpolation yields few or no
    # corners and the GUI silently never registers a sample. Scoring the
    # candidates here costs one frame and settles the argument.
    gray = cv2.cvtColor(best["frame"], cv2.COLOR_BGR2GRAY)
    dict_id = dict(CANDIDATES)[best["dict"]]
    dictionary = get_dictionary(dict_id)
    print()
    print("--size candidates. Corner COUNT does not discriminate -- 9x11 and")
    print("11x9 both yield (9-1)*(11-1) = 80 -- so each candidate is scored by")
    print("fitting a HOMOGRAPHY from its object points to the detected image")
    print("points. The board is planar, so the correct layout fits to well")
    print("under a pixel and a transposed one scrambles the correspondence:")
    results = []
    for spec in args.try_sizes.split(","):
        cols, rows = (int(v) for v in spec.strip().split("x"))
        expected = (cols - 1) * (rows - 1)
        try:
            if hasattr(cv2.aruco, "CharucoBoard_create"):
                board = cv2.aruco.CharucoBoard_create(
                    cols, rows, args.square, args.marker, dictionary)
            else:
                board = cv2.aruco.CharucoBoard(
                    (cols, rows), args.square, args.marker, dictionary)
            n_corners = 0
            if best["ids"] is not None and len(best["ids"]):
                if hasattr(cv2.aruco, "interpolateCornersCharuco"):
                    ok, corners, ids = cv2.aruco.interpolateCornersCharuco(
                        best["corners"], best["ids"], gray, board, minMarkers=1)
                    n_corners = 0 if corners is None else len(corners)
                else:
                    det = cv2.aruco.CharucoDetector(board)
                    corners, ids, _, _ = det.detectBoard(gray)
                    n_corners = 0 if corners is None else len(corners)
            residual = float("inf")
            if n_corners >= 8:
                obj = np.asarray(board.chessboardCorners
                                 if hasattr(board, "chessboardCorners")
                                 else board.getChessboardCorners())
                pts_obj = np.array([obj[int(i)][:2] for i in ids.flatten()],
                                   dtype=np.float32)
                pts_img = corners.reshape(-1, 2).astype(np.float32)
                H, _ = cv2.findHomography(pts_obj, pts_img, 0)
                if H is not None:
                    proj = cv2.perspectiveTransform(
                        pts_obj.reshape(-1, 1, 2), H).reshape(-1, 2)
                    residual = float(
                        np.mean(np.linalg.norm(proj - pts_img, axis=1)))
            results.append((spec.strip(), n_corners, expected, residual))
            print(f"  --size {spec.strip():<6} {n_corners:3d} / {expected} "
                  f"corners,  homography residual "
                  f"{residual:8.3f} px")
        except Exception as exc:                       # noqa: BLE001
            print(f"  --size {spec.strip():<6} FAILED: "
                  f"{type(exc).__name__}: {str(exc)[:80]}")
    winner = None
    if results:
        ranked = sorted(results, key=lambda r: r[3])
        winner = ranked[0]
        print()
        if winner[3] < 2.0 and (len(ranked) == 1 or ranked[1][3] > 4 * winner[3]):
            print(f"  --size {winner[0]} wins: {winner[3]:.3f} px vs "
                  f"{ranked[1][3]:.3f} px" if len(ranked) > 1 else
                  f"  --size {winner[0]}: {winner[3]:.3f} px")
        else:
            print("  INCONCLUSIVE -- no candidate fits clearly better. Get the")
            print("  whole board in view, roughly filling the frame, and rerun.")
            winner = None

    print()
    print("Then, with the winning size:")
    size_str = winner[0] if winner else "<winner>"
    print("  ros2 run camera_calibration cameracalibrator \\")
    print(f"      --pattern charuco --size {size_str} "
          f"--square {args.square} \\")
    print(f"      --charuco_marker_size {args.marker} "
          f"--aruco_dict {best['dict']} \\")
    print("      --no-service-check \\")
    print("      image:=/down_cam/image_raw camera:=/down_cam")

    if args.save:
        annotated = best["frame"].copy()
        cv2.aruco.drawDetectedMarkers(annotated, best["corners"], best["ids"])
        cv2.imwrite(args.save, annotated)
        print(f"wrote {args.save}")


if __name__ == "__main__":
    main()
