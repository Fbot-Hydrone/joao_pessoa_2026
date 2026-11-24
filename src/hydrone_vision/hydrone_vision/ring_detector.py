"""ring_detector — the base's yellow RING, found by fitting a circle to its arcs.

ROS-free. Fallback for the Phase 2 DELIVERY descent, where the kit hanging in
the gripper covers the middle of the belly image — exactly where the base is
once the vehicle is centred. MEASURED 2026-10-16, seed 6, base C: the YOLO
'base_pouso' gave 0 pads from 2.5 m down to 1.47 m while the blue square and
both sides of the ring were plainly visible beside the kit. A circle fitted to
whatever yellow survives still lands on the centre.

The fit is gated by size: the ring's radius in pixels times the height above
the base top is the constant `ring_k` (= ring radius in m x fx). Measured in
the sim: 145-155 at 0.6-1.6 m. Anything far from it (another base seen from
cruise, the floor read by the rangefinder, a stray yellow pixel) is dropped.
Below ~0.5 m the ring overflows the frame and the fit degrades; the descent
hands over to LAND long before that.
"""

import cv2
import numpy as np

YELLOW_HSV_LOW = (18, 110, 90)       # same band as pad_detector's sim mode
YELLOW_HSV_HIGH = (38, 255, 255)


def fit_ring(bgr, height_m, *, ring_k=150.0, k_tol=0.25, min_px=80,
             hsv_low=YELLOW_HSV_LOW, hsv_high=YELLOW_HSV_HIGH):
    """(u, v, radius_px, confidence) of the ring, or None.

    height_m  camera height above the base top (rangefinder)
    """
    if height_m is None or height_m <= 0.0:
        return None
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_low, hsv_high)
    pts = np.column_stack(np.nonzero(mask))[:, ::-1].astype(float)
    if len(pts) < min_px:
        return None
    # Algebraic (Kasa) fit, then refit on the points near the first circle so
    # the cross and stray pixels stop pulling it.
    for _ in range(2):
        a = np.column_stack([2.0 * pts, np.ones(len(pts))])
        (cx, cy, c), *_ = np.linalg.lstsq(a, (pts ** 2).sum(1), rcond=None)
        r = float(np.sqrt(max(c + cx * cx + cy * cy, 0.0)))
        d = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r)
        pts = pts[d < max(4.0, 0.15 * r)]
        if len(pts) < min_px:
            return None
    expected = ring_k / height_m
    off = abs(r - expected) / expected
    if off > k_tol:
        return None
    return float(cx), float(cy), r, 1.0 - off / k_tol
