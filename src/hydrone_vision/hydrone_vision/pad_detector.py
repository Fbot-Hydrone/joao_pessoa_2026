#!/usr/bin/env python3
"""
pad_detector — classic (non-learned) detection of the competition landing pad.

The pad
-------
A strongly saturated BLUE field carrying a YELLOW ring with a YELLOW cross
through its centre. Everything on it is far more saturated than the arena floor,
which is what makes plain HSV thresholding viable here — no training, no model
files, deterministic cost per frame.

    +-----------------+
    |     .-----.     |     blue   = pad field
    |    /   |   \    |     yellow = ring + cross
    |   |----+----|   |
    |    \   |   /    |
    |     '-----'     |
    +-----------------+

Why not just "find the blue blob"
---------------------------------
Anything blue-ish and saturated in the arena would pass: a tarp, a team shirt, a
painted line, the sky reflected in something wet. The structure IS the identity
of the pad, so the detector spends most of its effort proving that a blue blob
really carries a concentric yellow ring-and-cross, and reports a confidence
built from those checks rather than a bare yes/no.

The checks, cheapest first
--------------------------
1. COLOUR       blue and yellow masks by HSV range, with a high saturation floor.
2. SHAPE        the blue blob is big enough, solid (convex-ish) and not a sliver.
3. COEXISTENCE  a plausible fraction of the blue blob's interior is yellow.
4. CONCENTRIC   the yellow centroid sits near the blue centroid. Rotation- and
                perspective-invariant, and the single strongest cheap cue.
5. RING         cast rays out from the centre: a ring is yellow in EVERY
                direction. A yellow smear on one side is not.
6. CROSS        halfway out along each ray, yellow appears in exactly four
                angular lobes — the arms. A solid yellow disc is yellow at every
                angle there; a bare ring with no cross is yellow at none.

1-4 are hard gates (a blob failing them is not a pad under any lighting). 5 and
6 gate too, but only once the blob is big enough on screen for the ring and the
arms to survive thresholding; below that they merely score, so a pad seen small
and far away still registers with lower confidence and gets confirmed as the
drone closes in.

Checks 5 and 6 measure each ray against the ring radius FOUND ALONG THAT RAY,
never against a global circle, which is what makes them hold when perspective
squashes the pad into an ellipse.

Everything is expressed in fractions of the blob's own size, so a pad 8 m ahead
of the forward camera and a pad 2 m below the down camera go through identical
code with identical thresholds.

This module is deliberately ROS-free: it takes a BGR ndarray and returns
dataclasses. pad_detector_node.py wraps it; test_pad_detector.py exercises it
directly against synthetic renders.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np


# ── Defaults ─────────────────────────────────────────────────────────────────
# OpenCV hue is 0..179 (degrees/2). Blue ~ 100 (=200 deg), yellow ~ 27 (=54 deg).
# Neither band wraps around 0, so a plain inRange is enough for both.
DEFAULT_BLUE_HSV_LOW = (95, 110, 50)
DEFAULT_BLUE_HSV_HIGH = (135, 255, 255)
DEFAULT_YELLOW_HSV_LOW = (18, 110, 90)
DEFAULT_YELLOW_HSV_HIGH = (38, 255, 255)


@dataclass
class PadDetection2D:
    """One pad found in one image. Pure image-space — no world geometry."""

    u: float                      # centroid column, px
    v: float                      # centroid row, px
    radius_px: float              # sqrt(area/pi) of the blue region, px
    area_px: float                # blue region area, px^2
    confidence: float             # 0..1
    contour: np.ndarray           # outer contour of the blue region
    scores: dict = field(default_factory=dict)   # per-check scores, for debug

    @property
    def center(self) -> tuple[float, float]:
        return (self.u, self.v)


class PadDetector:
    """Stateless detector. Build once, call :meth:`detect` per frame.

    Every threshold is a constructor argument so the ROS node can expose them as
    parameters and they can be retuned against the real arena lighting without
    touching this file.
    """

    def __init__(
        self,
        blue_hsv_low=DEFAULT_BLUE_HSV_LOW,
        blue_hsv_high=DEFAULT_BLUE_HSV_HIGH,
        yellow_hsv_low=DEFAULT_YELLOW_HSV_LOW,
        yellow_hsv_high=DEFAULT_YELLOW_HSV_HIGH,
        min_area_px: float = 150.0,
        max_area_frac: float = 0.85,
        min_solidity: float = 0.80,
        max_aspect: float = 4.5,
        yellow_frac_min: float = 0.02,
        yellow_frac_max: float = 0.80,
        max_center_offset: float = 0.40,
        ring_cov_min: float = 0.55,
        mid_occ_min: float = 0.02,
        mid_occ_max: float = 0.85,
        structure_radius_px: float = 22.0,
        min_confidence: float = 0.50,
        max_detections: int = 8,
    ):
        self.blue_lo = np.array(blue_hsv_low, dtype=np.uint8)
        self.blue_hi = np.array(blue_hsv_high, dtype=np.uint8)
        self.yellow_lo = np.array(yellow_hsv_low, dtype=np.uint8)
        self.yellow_hi = np.array(yellow_hsv_high, dtype=np.uint8)

        self.min_area_px = float(min_area_px)
        self.max_area_frac = float(max_area_frac)
        self.min_solidity = float(min_solidity)
        self.max_aspect = float(max_aspect)
        self.yellow_frac_min = float(yellow_frac_min)
        self.yellow_frac_max = float(yellow_frac_max)
        self.max_center_offset = float(max_center_offset)
        self.ring_cov_min = float(ring_cov_min)
        self.mid_occ_min = float(mid_occ_min)
        self.mid_occ_max = float(mid_occ_max)
        self.structure_radius_px = float(structure_radius_px)
        self.min_confidence = float(min_confidence)
        self.max_detections = int(max_detections)

        # Reused structuring elements — allocating these per frame is pure waste.
        self._k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self._k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        # Last frame's masks, kept only so the node can draw them for debugging.
        self.last_blue_mask: np.ndarray | None = None
        self.last_yellow_mask: np.ndarray | None = None

    # ────────────────────────────────────────────────────────────────────────
    # Public API
    # ────────────────────────────────────────────────────────────────────────

    def detect(self, bgr: np.ndarray) -> list[PadDetection2D]:
        """Find every landing pad in a BGR image, best first."""
        if bgr is None or bgr.size == 0:
            return []

        blue, yellow = self.color_masks(bgr)
        self.last_blue_mask, self.last_yellow_mask = blue, yellow

        img_area = float(bgr.shape[0] * bgr.shape[1])
        contours, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        found: list[PadDetection2D] = []
        for contour in contours:
            det = self._evaluate(contour, blue, yellow, img_area)
            if det is not None:
                found.append(det)

        found.sort(key=lambda d: d.confidence, reverse=True)
        return self._suppress_overlaps(found)[: self.max_detections]

    def color_masks(self, bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Clean binary masks of the pad's two colours.

        CLOSE first to knit the ring and cross back together across the JPEG-ish
        colour fringing at their edges, then OPEN to drop speckle. Doing it in
        the other order would erase thin far-away cross arms before they were
        ever joined up.
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        masks = []
        for lo, hi in ((self.blue_lo, self.blue_hi),
                       (self.yellow_lo, self.yellow_hi)):
            mask = cv2.inRange(hsv, lo, hi)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._k_close)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._k_open)
            masks.append(mask)
        return masks[0], masks[1]

    # ────────────────────────────────────────────────────────────────────────
    # Per-candidate evaluation
    # ────────────────────────────────────────────────────────────────────────

    def _evaluate(self, contour, blue, yellow,
                  img_area: float) -> PadDetection2D | None:
        """Run the check cascade on one blue contour. None = not a pad."""
        area = float(cv2.contourArea(contour))
        if area < self.min_area_px or area > self.max_area_frac * img_area:
            return None

        # ── Check 2: shape ──────────────────────────────────────────────────
        # The pad's outer boundary is a convex quad or disc under any viewing
        # angle, so a low solidity means we grabbed something else (a shadowed
        # railing, two blobs bridged by noise).
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        solidity = area / hull_area if hull_area > 0 else 0.0
        if solidity < self.min_solidity:
            return None

        # Perspective squashes the pad but never into a sliver: a long thin blue
        # blob is a line marking or a wall edge, not a pad.
        (_, _), (rw, rh), _ = cv2.minAreaRect(contour)
        if min(rw, rh) <= 1e-6:
            return None
        aspect = max(rw, rh) / min(rw, rh)
        if aspect > self.max_aspect:
            return None

        # Fill the contour: everything below asks questions about the pad's
        # INTERIOR, and the raw blue mask has the ring and cross punched out of
        # it. `roi` is "the pad's footprint", holes included.
        x, y, w, h = cv2.boundingRect(contour)
        roi = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(roi, [contour], -1, 255, cv2.FILLED, offset=(-x, -y))

        yellow_roi = cv2.bitwise_and(yellow[y:y + h, x:x + w], roi)
        yellow_px = int(cv2.countNonZero(yellow_roi))
        if yellow_px == 0:
            return None

        # ── Check 3: colour coexistence ─────────────────────────────────────
        # Too little yellow = a plain blue object. Too much = a yellow object
        # with a blue rim, or the mask leaked.
        yellow_frac = yellow_px / area
        if not (self.yellow_frac_min <= yellow_frac <= self.yellow_frac_max):
            return None

        # ── Check 4: concentricity ──────────────────────────────────────────
        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            return None
        blue_cx = moments["m10"] / moments["m00"]
        blue_cy = moments["m01"] / moments["m00"]

        ys, xs = np.nonzero(yellow_roi)
        yellow_cx = float(xs.mean()) + x
        yellow_cy = float(ys.mean()) + y

        r_eq = math.sqrt(area / math.pi)
        offset = math.hypot(yellow_cx - blue_cx, yellow_cy - blue_cy) / r_eq
        if offset > self.max_center_offset:
            return None

        # ── Checks 5 & 6: ring coverage and cross arms, from one polar sweep ─
        # The sweep is centred on the BLUE centroid: the ring is symmetric about
        # it, while the yellow centroid drifts whenever an arm is clipped.
        radii = np.hypot(xs + x - blue_cx, ys + y - blue_cy)
        r95 = float(np.percentile(radii, 95.0))
        ring_cov, mid_occ, arms = self._polar_checks(
            yellow_roi, blue_cx - x, blue_cy - y, r95)

        # The gates only apply once the pad is big enough on screen that a
        # missing ring or a missing cross would actually be visible. Below that
        # the morphology has already fused ring and arms into one blob and the
        # measurements say nothing — see structure_radius_px.
        resolvable = r95 >= self.structure_radius_px
        if resolvable and not (
                ring_cov >= self.ring_cov_min
                and self.mid_occ_min <= mid_occ <= self.mid_occ_max):
            return None

        # ── Confidence ──────────────────────────────────────────────────────
        # Concentricity carries the most weight: it is the check that survives
        # scale, rotation and perspective while still being hard to trip by
        # accident. The structural checks refine rather than decide.
        center_score = 1.0 - min(offset / self.max_center_offset, 1.0)
        shape_score = min((solidity - self.min_solidity)
                          / max(1.0 - self.min_solidity, 1e-6), 1.0)
        color_score = self._band_score(yellow_frac,
                                       self.yellow_frac_min,
                                       self.yellow_frac_max)
        ring_score = self._ramp(ring_cov, 0.50, 0.95)
        cross_score = self._cross_score(arms) if resolvable else 0.0

        confidence = (0.32 * center_score
                      + 0.20 * ring_score
                      + 0.25 * cross_score
                      + 0.13 * color_score
                      + 0.10 * shape_score)
        if confidence < self.min_confidence:
            return None

        # The pad's centre is the blue blob's centroid. The yellow centroid is a
        # worse estimate: the ring is symmetric but the cross arms rarely are
        # once the mask clips one of them at the image border.
        return PadDetection2D(
            u=blue_cx,
            v=blue_cy,
            radius_px=r_eq,
            area_px=area,
            confidence=float(confidence),
            contour=contour,
            scores={
                "center": round(center_score, 3),
                "ring": round(ring_score, 3),
                "cross": round(cross_score, 3),
                "color": round(color_score, 3),
                "shape": round(shape_score, 3),
                "yellow_frac": round(yellow_frac, 3),
                "solidity": round(solidity, 3),
                "offset": round(offset, 3),
                "ring_cov": round(ring_cov, 3),
                "mid_occ": round(mid_occ, 3),
                "arms": arms,
                "resolvable": resolvable,
            },
        )

    # ────────────────────────────────────────────────────────────────────────
    # Individual structural checks
    # ────────────────────────────────────────────────────────────────────────

    # Polar sweep resolution. 72 rays = one every 5 deg, which resolves four
    # arms with margin; 48 radial samples put ~2 px between steps on a 100 px
    # pad. Both are fixed: the whole sweep is ~3.5k lookups, negligible next to
    # the HSV conversion, and making them parameters would only invite retuning
    # something that does not need it.
    _N_ANG = 72
    _N_RAD = 48

    def _polar_checks(self, yellow_roi: np.ndarray, cx: float, cy: float,
                      r95: float) -> tuple[float, float, int]:
        """One polar sweep of the yellow mask. Returns (ring_cov, mid_occ, arms).

        For each of 72 rays out of the pad centre we find the OUTERMOST yellow
        sample along that ray. That radius is the ring, measured in that
        direction — which is why an ellipse (a pad seen at a slant) is handled
        without ever fitting one.

          ring_cov  fraction of rays that hit yellow at all.
                    A ring   -> ~1.0. A one-sided smear -> low.
          mid_occ   fraction of rays that are ALSO yellow at half that radius.
                    Ring + cross -> the four arms only, ~0.2-0.5.
                    Solid disc   -> ~1.0.   Ring with no cross -> ~0.0.
          arms      how many separate angular lobes those mid hits form,
                    counted around the circle. The pad's cross gives 4.

        Rays that leave the ROI (the pad is clipped by the image border) are
        excluded from all three rather than counted as misses, so a pad hanging
        off the edge of the frame is judged only on the part we can see.
        """
        h, w = yellow_roi.shape
        if r95 < 2.0 or h < 3 or w < 3:
            return 0.0, 0.0, 0

        ang = np.linspace(0.0, 2.0 * math.pi, self._N_ANG, endpoint=False)
        # Reach past the ring so its outer edge is always inside the sweep.
        rad = np.linspace(0.0, 1.25 * r95, self._N_RAD)
        px = cx + np.cos(ang)[:, None] * rad[None, :]
        py = cy + np.sin(ang)[:, None] * rad[None, :]

        xi = np.rint(px).astype(np.int32)
        yi = np.rint(py).astype(np.int32)
        inside = (xi >= 0) & (xi < w) & (yi >= 0) & (yi < h)
        hit = np.zeros(inside.shape, dtype=bool)
        hit[inside] = yellow_roi[yi[inside], xi[inside]] > 0

        # A ray is usable only if most of it stayed in the ROI.
        usable = inside.mean(axis=1) > 0.8
        if not usable.any():
            return 0.0, 0.0, 0

        # Outermost yellow sample per ray (-1 = this ray never saw yellow).
        any_hit = hit.any(axis=1)
        outer = np.where(any_hit,
                         self._N_RAD - 1 - np.argmax(hit[:, ::-1], axis=1),
                         -1)

        ring_cov = float(np.count_nonzero(any_hit & usable)
                         / np.count_nonzero(usable))

        # Halfway out along each ray. Needs outer >= 2 or "half" rounds onto the
        # centre sample, where the cross always crosses itself.
        probe = usable & any_hit & (outer >= 2)
        if not probe.any():
            return ring_cov, 0.0, 0
        mid_idx = np.rint(outer * 0.5).astype(np.int32)
        mid_hit = np.zeros(self._N_ANG, dtype=bool)
        rows = np.nonzero(probe)[0]
        mid_hit[rows] = hit[rows, mid_idx[rows]]

        mid_occ = float(np.count_nonzero(mid_hit[probe])
                        / np.count_nonzero(probe))
        return ring_cov, mid_occ, self._count_lobes(mid_hit)

    @staticmethod
    def _count_lobes(flags: np.ndarray) -> int:
        """Number of circular runs of True — i.e. how many arms the cross has."""
        if not flags.any():
            return 0
        if flags.all():
            return 1
        # Count rising edges around the circle (np.roll closes the wrap-around).
        return int(np.count_nonzero(flags & ~np.roll(flags, 1)))

    @staticmethod
    def _cross_score(arms: int) -> float:
        """Score the arm count. Four is the pad; near-four is a clipped pad."""
        return {0: 0.0, 1: 0.10, 2: 0.35, 3: 0.65,
                4: 1.00, 5: 0.65, 6: 0.35}.get(arms, 0.15)

    # ────────────────────────────────────────────────────────────────────────
    # Small helpers
    # ────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _ramp(value: float, lo: float, hi: float) -> float:
        """Linear 0..1 ramp of `value` across [lo, hi], clamped."""
        if hi <= lo:
            return 0.0
        return float(min(max((value - lo) / (hi - lo), 0.0), 1.0))

    @staticmethod
    def _band_score(value: float, lo: float, hi: float) -> float:
        """1.0 at the centre of [lo, hi], falling to 0 at either edge.

        Used where both extremes are suspicious and the middle is the pad.
        """
        if hi <= lo:
            return 0.0
        mid = 0.5 * (lo + hi)
        half = 0.5 * (hi - lo)
        return float(max(0.0, 1.0 - abs(value - mid) / half))

    @staticmethod
    def _suppress_overlaps(dets: list[PadDetection2D]) -> list[PadDetection2D]:
        """Drop lower-confidence detections whose centres fall inside a better one.

        Nested contours can yield two candidates for one pad (the field and, if
        the ring is broken, a chunk of the interior). Assumes `dets` is already
        sorted best-first.
        """
        kept: list[PadDetection2D] = []
        for det in dets:
            if any(math.hypot(det.u - k.u, det.v - k.v)
                   < max(k.radius_px, det.radius_px) for k in kept):
                continue
            kept.append(det)
        return kept


# ── Debug rendering ──────────────────────────────────────────────────────────

def draw_detections(bgr: np.ndarray, dets: list[PadDetection2D],
                    label: str = "") -> np.ndarray:
    """Annotate a copy of `bgr` with the detections, for /…/debug_image."""
    out = bgr.copy()
    for i, det in enumerate(dets):
        colour = (0, 255, 0) if i == 0 else (0, 200, 255)
        cv2.drawContours(out, [det.contour], -1, colour, 2)
        cv2.circle(out, (int(det.u), int(det.v)), 4, colour, -1)
        cv2.putText(out, f"pad {det.confidence:.2f}",
                    (int(det.u) - 40, int(det.v) - int(det.radius_px) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)
    if label:
        cv2.putText(out, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return out
