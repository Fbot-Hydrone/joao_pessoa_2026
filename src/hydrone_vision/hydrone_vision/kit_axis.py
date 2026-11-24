"""kit_axis — which way a kit lies, from the pixels around its detection.

ROS-free. The kit YOLO gives an upright box, which cannot tell +30 deg from
-30 deg. The kit itself can: it is the one NEUTRAL thing (silver bag, black
trim) on a coloured base (blue paint, yellow ring and lines). So:

  1. Lab chroma over the detection, Otsu: neutral pixels = kit (tried with
     and without its holes filled: a label is a hole)
  2. an opening sized to the kit cuts off pale lines touching it
  3. the blob at the centre, its principal axis = the long axis

On the real C922 photos (kit_dataset_mix, 66 kits on the lab base with pale
yellow lines) this got 60 coherent and visually right; the other 6 were
motion blurred and came out incoherent, which means "don't turn". An edge
structure tensor alone was fooled by those lines and by the crumpled bag, so
it is only the fallback for a kit that is not neutral (the sim's renders dark
and blue), trusted when clearly one-directional (edge_min).

    angle      long axis in the image, rad from +u towards +v, [-pi/2, pi/2)
    coherence  (l1 - l2) / (l1 + l2) of the blob: 0 = round/square, a clean
               19.5 x 8 cm kit is ~0.5-0.7
"""

import math

import cv2
import numpy as np


def kit_axis(img, u, v, radius_px, *, pad=1.6, open_k=0.3, min_half_px=12,
             area_range=(0.3, 1.3), edge_min=0.35):
    """(angle, coherence) of the kit detected at (u, v), or None.

    `radius_px` is PadDetection's sqrt(box area / pi); the crop half-size is
    `pad` times it, enough for the box's corners at any angle. A blob whose
    area is outside `area_range` x the box area leaked into the background
    (or is a sliver), and is refused.
    """
    h, w = img.shape[:2]
    half = int(max(pad * radius_px, min_half_px))
    x0, y0 = int(u) - half, int(v) - half
    if x0 < 0 or y0 < 0 or x0 + 2 * half > w or y0 + 2 * half > h:
        return None          # cut by the frame: half a kit lies about its axis
    crop = img[y0:y0 + 2 * half, x0:x0 + 2 * half]
    lab = cv2.cvtColor(cv2.GaussianBlur(crop, (5, 5), 0), cv2.COLOR_BGR2LAB)
    chroma = np.hypot(lab[:, :, 1].astype(np.float32) - 128.0,
                      lab[:, :, 2].astype(np.float32) - 128.0)
    c8 = np.clip(chroma * 4.0, 0, 255).astype(np.uint8)
    _, mask = cv2.threshold(c8, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    # A label (yellow on Figure 8's kit) is a hole in the mask, and the
    # opening then cuts the strips beside it and splits the kit. Filling the
    # holes fixes that but glues on the base's lines when they enclose a
    # pocket. Both ways; the cleaner blob (higher coherence) wins.
    filled = mask.copy()
    outer, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(filled, outer, -1, 255, cv2.FILLED)
    k = max(3, int(open_k * radius_px) | 1)
    found = [r for r in (_blob_axis(m, half, k, radius_px, area_range)
                         for m in (mask, filled)) if r is not None]
    best = max(found, key=lambda r: r[1]) if found else None
    if best is not None and best[1] >= 0.3:
        return best
    # Not a neutral kit (the sim's renders dark and blue): fall back to the
    # edges, colour-blind, trusted only when clearly one-directional.
    edge = _edge_axis(lab[:, :, 0].astype(np.float32), half)
    if edge is not None and edge[1] >= edge_min:
        return edge
    return best


def _edge_axis(gray, half):
    """Gradient structure tensor under a tight Gaussian: the long edges
    dominate, so the strongest gradient direction is ACROSS the kit."""
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    g = cv2.getGaussianKernel(gray.shape[0], 0.33 * half)
    wgt = (g @ g.T).astype(np.float32)
    jxx = float((wgt * gx * gx).sum())
    jyy = float((wgt * gy * gy).sum())
    jxy = float((wgt * gx * gy).sum())
    if jxx + jyy <= 1e-6:
        return None
    across = 0.5 * math.atan2(2.0 * jxy, jxx - jyy)
    angle = (across + math.pi) % math.pi - math.pi / 2.0
    return angle, math.hypot(jxx - jyy, 2.0 * jxy) / (jxx + jyy)


def _blob_axis(mask, half, k, radius_px, area_range):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (k, k)))
    n, labels, _, centres = cv2.connectedComponentsWithStats(mask)
    if n < 2:
        return None
    # the component under the centre, else the nearest one
    best = min(range(1, n), key=lambda i: (labels[half, half] != i, math.hypot(
        centres[i][0] - half, centres[i][1] - half)))
    ys, xs = np.nonzero(labels == best)
    area = len(xs) / (math.pi * radius_px * radius_px)
    if len(xs) < 30 or not area_range[0] <= area <= area_range[1]:
        return None
    ev, evec = np.linalg.eigh(np.cov(np.vstack([xs, ys]).astype(np.float64)))
    if ev[1] <= 0.0:
        return None
    angle = math.atan2(evec[1, 1], evec[0, 1])
    angle = (angle + math.pi / 2.0) % math.pi - math.pi / 2.0
    return angle, float((ev[1] - ev[0]) / (ev[1] + ev[0]))


def draw(img, u, v, angle, length_px, colour=(0, 255, 255)):
    """The axis as a line through (u, v), for the debug image."""
    dx, dy = math.cos(angle) * length_px, math.sin(angle) * length_px
    cv2.line(img, (int(u - dx), int(v - dy)), (int(u + dx), int(v + dy)),
             colour, 2)
    return img
