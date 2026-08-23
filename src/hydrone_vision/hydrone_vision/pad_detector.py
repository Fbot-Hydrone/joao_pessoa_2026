#!/usr/bin/env python3
r"""
pad_detector — classic (non-learned) detection of the competition landing pad.

TWO PADS, TWO WAYS OF FINDING THEM
----------------------------------
BiguaSim's pad is a strongly saturated BLUE field carrying a YELLOW ring and
cross, sitting on brown-green ground. Nothing else in the map is that blue, so
"find the blue blob, then prove it carries a ring and a cross" works, and
`field_mode="blue"` does exactly that.

The REAL pad does not work that way.

    SIM                          REAL
    +-----------------+          +=================+  <- yellow square border
    |     .-----.     |          I     .-----.     I
    |    /   |   \    |          I    /   |   \    I     field: blue, SAME HUE
    |   |----+----|   |          I   |----+----|   I            as the floor
    |    \   |   /    |          I    \   |   /    I     yellow: border+ring
    |     '-----'     |          I     '-----'     I             +cross
    +-----------------+          +=================+
     blue field                   BLUE FOAM FLOOR all around
     brown ground around

It lies on interlocking blue foam of the same hue as its own field, and it
carries a yellow square border the simulated pad does not have. There is no
"blue blob" to find: the pad and the floor are one blue region.

WHY ABSOLUTE HSV BANDS CANNOT SEPARATE THEM
-------------------------------------------
Measured off the arena photographs and the ZED's own frames, 2026-08-22:

                       H          S          V
    floor, phone      109        142        193
    floor, ZED        103-105    220        190
    pad field, phone  107        104        158
    pad field, ZED    101        172        146
    markings, phone    25        218        254
    markings, ZED      58         44        196

Two things follow, and both are fatal to a fixed band.

The MARKINGS are not yellow to the ZED. Under the arena's lighting its auto
white balance throws a heavy green cast and its exposure washes the paint out:
H 58 (green), S 44. The yellow band this file uses for the simulator is
H 18..38, S >= 110, and against a real ZED frame it admits ZERO pixels. Since
`detect()` reaches the ring, cross and concentricity checks only through the
yellow mask, an empty yellow mask means the cascade never runs at all — the
detector returns nothing and says nothing about why.

The FIELD moves further than any band is wide. Floor saturation is 142 in one
camera and 220 in the other; the pad's own field is 104 and 172. A band cut to
separate them in one frame merges them in the other. An earlier version of this
file capped field saturation at 168 (calibrated on the ZED, where the floor is
220) and consequently swallowed floor and pad together on the phone photograph,
where the floor is 142. There is no constant that survives both.

WHAT THE REAL MODE USES INSTEAD
-------------------------------
Only the MASK stage changes. Every structural check below is untouched: they
ask about yellow relative to a footprint and never about absolute colour.

1. MARKINGS, by CONTRAST rather than by hue. Take the opponent channel

       yb = (R + G)/2 - B

   and subtract its own local mean over a large box. Blue floor and blue field
   both sit far below their neighbourhood; paint sits above it. This is a
   colour DIFFERENCE measured against a LOCAL reference, so a white-balance
   shift, an exposure change, or the slow horizontal banding the ZED shows
   under the arena's lights all move signal and reference together and cancel.

2. FLOOR. Blue-dominant pixels, holes filled, then eroded. Markings are only
   looked for inside the mat, which discards the walls, the ceiling and the
   floor/wall boundary — that boundary is neither blue nor paint, so it fires
   the contrast test along its whole length and, left in, bridges every real
   marking into one useless cluster.

3. FOOTPRINT. Cluster the marking pixels and take each cluster's convex hull.
   The pad's identity comes from its markings, which is where it actually is,
   rather than from a field colour indistinguishable from the floor.

   The link distance that decides what clusters with what is the one thing that
   cannot be derived from the frame: it should scale with the pad, and the pad
   is what we are looking for. So the clustering runs at three link radii and
   the results go through the same overlap suppression as everything else.

Why not just "find the markings"
--------------------------------
Anything pale on a blue mat would pass: a scuff, a cable, a reflection, a
sticker. The structure IS the identity of the pad, so the detector spends most
of its effort proving that a cluster of paint really is a concentric ring and
cross, and reports a confidence built from those checks rather than a bare
yes/no.

The checks, cheapest first
--------------------------
1. COLOUR       field and marking masks — by HSV band in sim, by local contrast
                on the blue mat in real.
2. SHAPE        the footprint is big enough, solid (convex-ish) and not a
                sliver.
3. COEXISTENCE  a plausible fraction of the footprint's interior is marking.
4. CONCENTRIC   the marking centroid sits near the footprint centroid. Rotation-
                and perspective-invariant, and the single strongest cheap cue.
5. RING         cast rays out from the centre: a ring is marked in EVERY
                direction. A smear on one side is not.
6. CROSS        halfway out along each ray, marking appears in exactly four
                angular lobes — the arms. A solid disc is marked at every angle
                there; a bare ring with no cross at none.
7. STRENGTH     (real only) how far the markings cleared the contrast
                threshold. Every check above asks how the paint is ARRANGED;
                this one asks whether it is paint. mark_delta is set low so
                faint far-away markings still enter the mask, and a stain that
                only just clears it can put four lobes on the mid-radius probe
                by accident — one in the arena photographs did, at confidence
                0.85, above the confidence at which phase1_mission commits to a
                landing.

1-4 are hard gates. 5 and 6 gate too, but in SIM only once the blob is big
enough on screen for the ring and the arms to survive thresholding; below that
they merely score, so a pad seen small and far away still registers with lower
confidence and gets confirmed as the drone closes in.

REAL mode requires them outright. Its mask is far more permissive by design —
it accepts any local contrast rather than one hue — so an unresolvable blob
there is indistinguishable from a smudge, and letting it through on
concentricity alone is how a scuff becomes a landing site.

Checks 5 and 6 measure each ray against the ring radius FOUND ALONG THAT RAY,
never against a global circle, which is what makes them hold when perspective
squashes the pad into an ellipse.

THE YELLOW BORDER, AND WHY THE ROI IS PULLED IN
-----------------------------------------------
_polar_checks takes the OUTERMOST marking along each ray to be the ring. The
real pad's square border lies further out than the ring, so any of it inside
the ROI BECOMES the ring, and the mid-radius probe then lands on the actual
circle instead of on the cross arms — measured, mid_occ 0.000 and zero arms on
a pad that is plainly a pad.

Sim ROIs are eroded by `roi_erode_frac` of the blob's equivalent radius, which
is enough there because the simulated markings sit well inside the field.

That does not work in real mode, where the border is inside the hull by
construction. An isotropic erosion deep enough to clear the border along the
square's edges still leaves its CORNERS, which are a factor sqrt(2) further
out — measured, an erosion of 23 px on a 297 px hull left paint at r=169 and
the arms vanished. Real mode therefore SCALES the hull toward its centroid by
`roi_shrink`, which pulls edges and corners in by the same proportion and is
the right transform for a border at a fixed fraction of the pad.

Everything is expressed in fractions of the footprint's own size, so a pad 8 m
ahead of the forward camera and a pad 2 m below the down camera go through
identical code with identical thresholds.

KNOWN CONFUSERS
---------------
Three things in the arena pass every check in this file. All three are recorded
with their measurements in docs/LANDING-SITES.md 3; the one that matters most
is the first.
A solar panel leaning against the arena wall is blue, rectangular, and ruled
with a pale grid. At long range it satisfied every check here and outranked a
real pad. Check 7 happens to drop it in the frames we have, which is luck and
not a discriminator: it cannot be separated from a pad by appearance alone.
What separates them is that the panel is VERTICAL and pads lie on the FLOOR, so
the real fix belongs where the geometry is — a ground-plane gate on the
back-projected point in pad_detector_node — and not in this file. Do not assume
the panel stays rejected from another angle.

The safety netting at the mat's edge is a vertical row of bright dots on blue,
and a hull round six of them scored 0.84. The drone itself, parked on the mat,
scored 0.90. Both are small — hull radius 25-30 px against 72-421 px for every
real pad measured — and raising real_min_radius_px to 40 removes them without
touching a single real detection. It is not the default; the docs say why.

This module is deliberately ROS-free: it takes a BGR ndarray and returns
dataclasses. pad_detector_node.py wraps it; test_pad_detector.py exercises it
directly against synthetic renders and against measurements from the arena.
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

# ── Real-arena mode ──────────────────────────────────────────────────────────
# None of these is a colour. They are contrasts and proportions, which is the
# whole point: see the module docstring for the measurements that rule an
# absolute HSV band out.

# How far above its own neighbourhood (R+G)/2 - B has to rise to count as
# paint. Low enough for the ZED's washed-out rendering of the yellow, high
# enough to ignore the mat's own weave and the JPEG-ish mush around it.
DEFAULT_MARK_DELTA = 8.0
# How far above the SAME local mean the paint of a real pad actually stands,
# as a multiple of mark_delta. mark_delta is set low so faint far-away paint
# still enters the mask; this is the second look that throws out what only just
# cleared it. Measured across the arena images: the markings of a real pad have
# a median contrast of 25-192, every false positive 9-13, so 2.5 x 8 = 20 sits
# in a gap with a factor of two of margin on both sides.
DEFAULT_MARK_CONTRAST_MULT = 2.5
# Radius of the neighbourhood, as a fraction of the frame's longer side. It has
# to be comfortably wider than a marking stroke, or a stroke averages into its
# own reference and stops standing out.
DEFAULT_MARK_WINDOW_FRAC = 0.06
# How far to pull the mat in from its own edge. The floor/wall boundary is
# neither blue nor paint and fires the contrast test along its whole length.
DEFAULT_FLOOR_ERODE_FRAC = 0.012
# Link distances for clustering marking pixels into one pad, as fractions of
# the frame's longer side. Three of them because the right distance scales with
# the pad, and the pad is what we are looking for.
DEFAULT_LINK_FRACS = (0.004, 0.009, 0.020)
# A cluster smaller than this is not a pad we could land on, and at this size
# the ring and arms no longer survive the mask.
DEFAULT_REAL_MIN_RADIUS_PX = 24.0
# A pad's hull is a convex quad or ellipse and fills most of its own min-area
# rect. A sprawl of unrelated marks does not.
DEFAULT_RECT_FILL_MIN = 0.62
# Scale the hull toward its centroid by this before asking about markings, so
# the yellow border is not mistaken for the ring. See the module docstring.
DEFAULT_ROI_SHRINK = 0.80
# Blue-dominance margin for "this is the mat", in 0..255 BGR units.
DEFAULT_BLUE_MARGIN = 8.0

FIELD_BLUE = "blue"
FIELD_DARK = "dark_blue"


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
        field_mode: str = FIELD_BLUE,
        mark_delta: float = DEFAULT_MARK_DELTA,
        mark_window_frac: float = DEFAULT_MARK_WINDOW_FRAC,
        mark_contrast_mult: float = DEFAULT_MARK_CONTRAST_MULT,
        floor_erode_frac: float = DEFAULT_FLOOR_ERODE_FRAC,
        link_fracs=DEFAULT_LINK_FRACS,
        real_min_radius_px: float = DEFAULT_REAL_MIN_RADIUS_PX,
        rect_fill_min: float = DEFAULT_RECT_FILL_MIN,
        roi_shrink: float = DEFAULT_ROI_SHRINK,
        blue_margin: float = DEFAULT_BLUE_MARGIN,
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
        structure_radius_px: float | None = None,
        roi_erode_frac: float = 0.06,
        min_confidence: float = 0.50,
        max_detections: int = 8,
    ):
        if field_mode not in (FIELD_BLUE, FIELD_DARK):
            raise ValueError(
                f"field_mode must be {FIELD_BLUE!r} or {FIELD_DARK!r}, "
                f"got {field_mode!r}")
        self.field_mode = field_mode
        self.field_lo = np.array(blue_hsv_low, dtype=np.uint8)
        self.field_hi = np.array(blue_hsv_high, dtype=np.uint8)
        self.yellow_lo = np.array(yellow_hsv_low, dtype=np.uint8)
        self.yellow_hi = np.array(yellow_hsv_high, dtype=np.uint8)

        self.mark_delta = float(mark_delta)
        self.mark_window_frac = float(mark_window_frac)
        self.mark_contrast_mult = float(mark_contrast_mult)
        self.floor_erode_frac = float(floor_erode_frac)
        self.link_fracs = tuple(float(f) for f in link_fracs)
        self.real_min_radius_px = float(real_min_radius_px)
        self.rect_fill_min = float(rect_fill_min)
        self.roi_shrink = float(roi_shrink)
        self.blue_margin = float(blue_margin)

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
        # Real-mode footprints are convex hulls of paint, which appear at a
        # smaller apparent size than a sim field blob of the same pad; the
        # threshold has to come down with them or the arms are never checked.
        self.structure_radius_px = float(
            structure_radius_px if structure_radius_px is not None
            else (16.0 if field_mode == FIELD_DARK else 22.0))
        self.roi_erode_frac = float(roi_erode_frac)
        self.min_confidence = float(min_confidence)
        self.max_detections = int(max_detections)

        # Reused structuring elements — allocating these per frame is pure waste.
        self._k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self._k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self._k_floor = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

        # Last frame's masks, kept only so the node can draw them for debugging.
        # In real mode the "field" mask is the mat and the "yellow" one is the
        # contrast-derived marking mask.
        self.last_field_mask: np.ndarray | None = None
        self.last_yellow_mask: np.ndarray | None = None

    # ────────────────────────────────────────────────────────────────────────
    # Public API
    # ────────────────────────────────────────────────────────────────────────

    def detect(self, bgr: np.ndarray) -> list[PadDetection2D]:
        """Find every landing pad in a BGR image, best first."""
        if bgr is None or bgr.size == 0:
            return []
        found = (self._detect_real(bgr) if self.field_mode == FIELD_DARK
                 else self._detect_sim(bgr))
        found.sort(key=lambda d: d.confidence, reverse=True)
        return self._suppress_overlaps(found)[: self.max_detections]

    def color_masks(self, bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Clean binary masks of the SIMULATED pad's two colours.

        CLOSE first to knit the ring and cross back together across the JPEG-ish
        colour fringing at their edges, then OPEN to drop speckle. Doing it in
        the other order would erase thin far-away cross arms before they were
        ever joined up.
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        masks = []
        for lo, hi in ((self.field_lo, self.field_hi),
                       (self.yellow_lo, self.yellow_hi)):
            mask = cv2.inRange(hsv, lo, hi)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._k_close)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._k_open)
            masks.append(mask)
        return masks[0], masks[1]

    def real_masks(self, bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(mat, markings) for the REAL pad. Neither is a hue test.

        `mat` is the blue foam floor with its holes filled and its own edge
        pulled in; `markings` is the paint on it. Exposed because it is the
        first thing to look at when the arena's light changes and the detector
        goes quiet.
        """
        mat, mark, _ = self._real_stage(self._opponent(bgr))
        return mat, mark

    def _real_stage(self, opp: np.ndarray):
        """(mat, markings, contrast). The contrast plane is kept because the
        mask is a yes/no and HOW FAR a marking cleared the threshold is the
        cheapest thing that separates paint from a stain — see _detect_real."""
        mat = self._floor_region(opp)
        mark, contrast = self._marking_mask(opp)
        return mat, cv2.bitwise_and(mark, mat), contrast

    # (B, G, R) -> (R + G)/2 - B, the yellow-vs-blue opponent channel.
    _OPPONENT = np.array([[-1.0, 0.5, 0.5]])

    def _opponent(self, bgr: np.ndarray) -> np.ndarray:
        """The opponent channel, in one pass over the frame.

        Both real-mode masks read this same plane, from opposite ends: the mat
        sits far below zero, paint stands above its own local mean. Computing
        it once and sharing it is worth the plumbing — a cv2.split of a float
        copy of the frame cost more than every morphological operation in this
        file put together.
        """
        return cv2.transform(bgr.astype(np.float32), self._OPPONENT)

    # ────────────────────────────────────────────────────────────────────────
    # Simulated pad: find the blue field, then prove it carries the markings
    # ────────────────────────────────────────────────────────────────────────

    def _detect_sim(self, bgr: np.ndarray) -> list[PadDetection2D]:
        field_mask, yellow = self.color_masks(bgr)
        self.last_field_mask, self.last_yellow_mask = field_mask, yellow

        img_area = float(bgr.shape[0] * bgr.shape[1])
        contours, _ = cv2.findContours(field_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        found: list[PadDetection2D] = []
        for contour in contours:
            det = self._evaluate(contour, yellow, img_area)
            if det is not None:
                found.append(det)
        return found

    # ────────────────────────────────────────────────────────────────────────
    # Real pad: find the markings, then prove they are a pad
    # ────────────────────────────────────────────────────────────────────────

    def _detect_real(self, bgr: np.ndarray) -> list[PadDetection2D]:
        mat, mark, contrast = self._real_stage(self._opponent(bgr))
        self.last_field_mask, self.last_yellow_mask = mat, mark

        img_area = float(bgr.shape[0] * bgr.shape[1])
        min_area = max(self.min_area_px,
                       math.pi * self.real_min_radius_px ** 2)

        found: list[PadDetection2D] = []
        for hull in self._marking_clusters(mark, bgr.shape):
            area = float(cv2.contourArea(hull))
            if area < min_area:
                continue
            # A hull's solidity is 1.0 by construction, so the sim's solidity
            # gate says nothing here. This is its replacement: a pad fills most
            # of its own min-area rect, a sprawl of unrelated marks does not.
            (_, _), (rw, rh), _ = cv2.minAreaRect(hull)
            if min(rw, rh) <= 1e-6 or area / (rw * rh) < self.rect_fill_min:
                continue

            det = self._evaluate(hull, mark, img_area,
                                 roi_shrink=self.roi_shrink)
            if det is None:
                continue
            # See the module docstring: real mode's mask is deliberately
            # permissive, so a candidate too small to show a cross is a smudge
            # until proven otherwise. Two to five lobes rather than exactly
            # four, because perspective and clipping merge or split an arm
            # often enough — measured across the arena images, real pads came
            # back with 3, 4 and 5.
            if not det.scores["resolvable"] or not 2 <= det.scores["arms"] <= 5:
                continue

            # Last gate, and the only one that asks how STRONG the markings
            # are rather than how they are arranged. A faint stain the size of
            # a distant pad can put four lobes on the mid-radius probe by
            # accident — one did, at confidence 0.85, above the confidence at
            # which phase1_mission commits to a landing. Real paint clears the
            # threshold by a wide margin and a stain only just clears it.
            strength = self._marking_strength(hull, mark, contrast)
            if strength < self.mark_contrast_mult * self.mark_delta:
                continue
            det.scores["contrast"] = round(strength, 1)
            found.append(det)
        return found

    @staticmethod
    def _marking_strength(hull, mark: np.ndarray,
                          contrast: np.ndarray) -> float:
        """Median contrast of the marking pixels inside `hull`."""
        x, y, w, h = cv2.boundingRect(hull)
        foot = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(foot, [hull], -1, 255, cv2.FILLED, offset=(-x, -y))
        chosen = (mark[y:y + h, x:x + w] > 0) & (foot > 0)
        if not chosen.any():
            return 0.0
        return float(np.median(contrast[y:y + h, x:x + w][chosen]))

    def _marking_mask(self, opp: np.ndarray):
        """Paint, by local contrast in the yellow-vs-blue opponent channel.

        Subtracting the channel's own local mean is what makes this survive the
        ZED's white balance, its exposure and its slow horizontal banding: all
        three move the paint and its surroundings together.
        """
        win = self._odd(self.mark_window_frac * max(opp.shape[:2]), 9)
        local = cv2.boxFilter(opp, -1, (win, win), normalize=True,
                              borderType=cv2.BORDER_REPLICATE)
        contrast = opp - local
        mask = (contrast > self.mark_delta).astype(np.uint8) * 255
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._k_open), contrast

    def _floor_region(self, opp: np.ndarray) -> np.ndarray:
        """The blue mat, holes filled, pulled in from its own edge.

        Holes are filled because the pad's own markings and anything else lying
        on the mat punch through it, and a pad must not fall outside the region
        it sits in. Only components worth at least 1% of the frame count: the
        mat is the largest thing in view whenever a pad is, and small blue
        oddments elsewhere are not floor.
        """
        mat = ((opp < -self.blue_margin).astype(np.uint8) * 255)
        mat = cv2.morphologyEx(mat, cv2.MORPH_CLOSE, self._k_floor)
        mat = cv2.morphologyEx(mat, cv2.MORPH_OPEN, self._k_floor)

        out = np.zeros_like(mat)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mat, 8)
        for i in range(1, count):
            if stats[i, cv2.CC_STAT_AREA] >= 0.01 * mat.size:
                out |= self._fill_holes((labels == i).astype(np.uint8) * 255)

        erode_px = int(round(self.floor_erode_frac * max(opp.shape[:2])))
        if erode_px >= 1:
            out = cv2.erode(out, cv2.getStructuringElement(
                cv2.MORPH_RECT, (2 * erode_px + 1,) * 2))
        return out

    def _marking_clusters(self, mark: np.ndarray, shape) -> list[np.ndarray]:
        """Convex hulls of the marking pixels, grouped at several link scales.

        The right link distance is a fraction of the pad, which is the thing
        being looked for, so all three are tried and the winners are settled by
        the same overlap suppression that settles everything else.

        The kernel is a SQUARE, not a disc, and that is a performance decision:
        OpenCV runs a rectangular dilation separably, and at the largest link
        radius on a 1280x720 frame the disc costs 21 ms against the square's
        0.7 ms. What it buys the disc is a link distance that is Euclidean
        rather than Chebyshev — up to sqrt(2) tighter on the diagonal — which
        is far finer than the spacing between the three radii anyway.
        """
        span = max(shape[:2])
        hulls: list[np.ndarray] = []
        for frac in self.link_fracs:
            radius = max(1, int(round(frac * span)))
            grown = cv2.dilate(mark, cv2.getStructuringElement(
                cv2.MORPH_RECT, (2 * radius + 1,) * 2))
            contours, _ = cv2.findContours(grown, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            hulls.extend(cv2.convexHull(c) for c in contours)
        return hulls

    @staticmethod
    def _fill_holes(mask: np.ndarray) -> np.ndarray:
        """Close every region fully enclosed by `mask`.

        Flood-filled from OUTSIDE a one-pixel border, so it works even when the
        mask already touches (0, 0) — which it does whenever the mat fills the
        frame.
        """
        h, w = mask.shape
        bordered = cv2.copyMakeBorder(mask, 1, 1, 1, 1,
                                      cv2.BORDER_CONSTANT, value=0)
        cv2.floodFill(bordered, np.zeros((h + 4, w + 4), np.uint8), (0, 0), 255)
        return mask | cv2.bitwise_not(bordered)[1:-1, 1:-1]

    @staticmethod
    def _odd(value: float, minimum: int = 3) -> int:
        """Nearest odd integer at or above `minimum` — box filters want odd."""
        n = max(int(round(value)), minimum)
        return n if n % 2 else n + 1

    # ────────────────────────────────────────────────────────────────────────
    # Per-candidate evaluation
    # ────────────────────────────────────────────────────────────────────────

    def _evaluate(self, contour, yellow, img_area: float,
                  roi_shrink: float | None = None) -> PadDetection2D | None:
        """Run the check cascade on one footprint contour. None = not a pad.

        `contour` is the simulated pad's blue field or the real pad's marking
        hull; the checks are the same either way. `roi_shrink` picks how the
        footprint is pulled in before the markings are read out of it — see
        below.
        """
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

        # Perspective squashes the pad but never into a sliver: a long thin
        # footprint is a line marking or a wall edge, not a pad.
        (_, _), (rw, rh), _ = cv2.minAreaRect(contour)
        if min(rw, rh) <= 1e-6:
            return None
        aspect = max(rw, rh) / min(rw, rh)
        if aspect > self.max_aspect:
            return None

        # Fill the contour: everything below asks questions about the pad's
        # INTERIOR, and the field mask has the ring and cross punched out of
        # it. `roi` is "the pad's footprint", holes included.
        x, y, w, h = cv2.boundingRect(contour)
        roi = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(roi, [contour], -1, 255, cv2.FILLED, offset=(-x, -y))

        # Pull the footprint in from its own edge before asking about yellow,
        # so that the real pad's square BORDER is not taken for its ring.
        #
        # Two ways, because the border sits differently in the two modes. In
        # sim the contour is the field and the border, where there is one, only
        # bleeds a thread inside it through antialiasing and colour fringing —
        # an isotropic erosion clears that and costs nothing.
        #
        # In real mode the border is INSIDE the hull by construction, and an
        # erosion deep enough to clear it along the square's edges still leaves
        # its corners, a factor sqrt(2) further out. Scaling the contour toward
        # its centroid pulls edges and corners in by the same proportion, which
        # is what a border at a fixed fraction of the pad needs.
        if roi_shrink is not None:
            roi = self._shrunk_roi(contour, roi_shrink, x, y, w, h)
        else:
            erode_px = int(round(self.roi_erode_frac
                                 * math.sqrt(area / math.pi)))
            if erode_px >= 1:
                k = 2 * erode_px + 1
                roi = cv2.erode(roi, cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (k, k)))

        yellow_roi = cv2.bitwise_and(yellow[y:y + h, x:x + w], roi)
        yellow_px = int(cv2.countNonZero(yellow_roi))
        if yellow_px == 0:
            return None

        # ── Check 3: colour coexistence ─────────────────────────────────────
        # Too little marking = a plain blue object. Too much = a yellow
        # object with a blue rim, or the mask leaked.
        yellow_frac = yellow_px / area
        if not (self.yellow_frac_min <= yellow_frac <= self.yellow_frac_max):
            return None

        # ── Check 4: concentricity ──────────────────────────────────────────
        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            return None
        foot_cx = moments["m10"] / moments["m00"]
        foot_cy = moments["m01"] / moments["m00"]

        ys, xs = np.nonzero(yellow_roi)
        yellow_cx = float(xs.mean()) + x
        yellow_cy = float(ys.mean()) + y

        r_eq = math.sqrt(area / math.pi)
        offset = math.hypot(yellow_cx - foot_cx, yellow_cy - foot_cy) / r_eq
        if offset > self.max_center_offset:
            return None

        # ── Checks 5 & 6: ring coverage and cross arms, from one polar sweep ─
        # The sweep is centred on the FOOTPRINT centroid: the ring is
        # symmetric about it, while the yellow centroid drifts whenever an arm
        # is clipped.
        radii = np.hypot(xs + x - foot_cx, ys + y - foot_cy)
        r95 = float(np.percentile(radii, 95.0))
        ring_cov, mid_occ, arms = self._polar_checks(
            yellow_roi, foot_cx - x, foot_cy - y, r95)

        # The gates only apply once the pad is big enough on screen that a
        # missing ring or a missing cross would actually be visible. Below that
        # the morphology has already fused ring and arms into one blob and the
        # measurements say nothing — see structure_radius_px. Real mode goes
        # further and refuses the candidate outright; _detect_real says why.
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

        # The pad's centre is the footprint's centroid. The yellow centroid is
        # a worse estimate: the ring is symmetric but the cross arms rarely are
        # once the mask clips one of them at the image border.
        return PadDetection2D(
            u=foot_cx,
            v=foot_cy,
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

    @staticmethod
    def _shrunk_roi(contour, scale: float, x: int, y: int,
                    w: int, h: int) -> np.ndarray:
        """`contour` scaled toward its own centroid, rendered in its bbox."""
        moments = cv2.moments(contour)
        roi = np.zeros((h, w), dtype=np.uint8)
        if moments["m00"] <= 0:
            return roi
        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]
        pts = contour.astype(np.float32).reshape(-1, 2)
        pts = (pts - (cx, cy)) * scale + (cx, cy)
        cv2.drawContours(roi, [np.rint(pts).astype(np.int32)], -1, 255,
                         cv2.FILLED, offset=(-x, -y))
        return roi

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
