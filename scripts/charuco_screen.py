#!/usr/bin/env python3
"""Show the calibration ChArUco board full screen, so nothing gets printed.

    python3 scripts/charuco_screen.py --watch cal_imgs

The SAME board as the printed calib.io one (9x11, 4x4_250, marker/square
16/22), so capture_charuco.py and calibrate_offline.py run on their defaults.
Its size on the screen does not matter: scaling the object points changes only
the translations, never fx, fy, cx, cy or distortion (CALIBRATION.md §5). A
screen is also flatter than any paper print.

Drawn pixel for pixel at the screen's height, turned to landscape. With
--watch it counts the frames capture_charuco.py has kept and shows it in the
spare margin, because the board covers the terminal. q or Esc to quit.
"""
import argparse
import glob
import os

import cv2
import numpy as np


def board_image(height_px, cols=9, rows=11, legacy=True):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    # the short side (cols) goes along the screen height, with half a square
    # of white all round so the edge markers have a quiet zone
    sq = height_px // (cols + 1)
    margin = sq // 2
    board = cv2.aruco.CharucoBoard((cols, rows), 0.022, 0.016, d)
    if hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(legacy)   # 9x11: same image either way
    img = board.generateImage((cols * sq + 2 * margin, rows * sq + 2 * margin),
                              marginSize=margin)
    return np.rot90(img).copy(), sq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--screen", default="1920x1080",
                    help="screen resolution WxH (xrandr)")
    ap.add_argument("--watch", default="",
                    help="capture_charuco.py --out dir, to show its count")
    ap.add_argument("--target", type=int, default=45)
    args = ap.parse_args()
    W, H = (int(v) for v in args.screen.split("x"))

    board, sq = board_image(H)
    canvas = np.full((H, W), 255, np.uint8)
    canvas[:board.shape[0], :board.shape[1]] = board
    print(f"board {board.shape[1]}x{board.shape[0]} px, square {sq} px. "
          "Screen brightness up, screensaver off (xset s off -dpms).")

    cv2.namedWindow("charuco", cv2.WINDOW_NORMAL)
    cv2.setWindowProperty("charuco", cv2.WND_PROP_FULLSCREEN,
                          cv2.WINDOW_FULLSCREEN)
    x0 = board.shape[1] + 20
    while True:
        frame = canvas.copy()
        if args.watch:
            n = len(glob.glob(os.path.join(args.watch, "view_*.png")))
            # grey text in the margin, nothing a marker detector could use
            cv2.putText(frame, f"{n}/{args.target}", (x0, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 3.0, 128, 6)
        cv2.imshow("charuco", frame)
        if cv2.waitKey(500) in (27, ord("q")):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
