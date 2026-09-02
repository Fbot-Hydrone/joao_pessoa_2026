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

WHY ABSOLUTE COLOUR CANNOT FIND IT
----------------------------------
Measured off the arena photographs and off the ZED's own frames, 2026-08-22:

                       H          S          V
    floor, phone      109        142        193
    floor, ZED        103-105    220        190
    pad field, phone  107        104        158
    pad field, ZED    101        172        146
    markings, phone    25        218        254
    markings, ZED      58         44        196

The MARKINGS are not yellow to the ZED. Under the arena's lighting its auto
white balance throws a heavy green cast and its exposure washes the paint out:
H 58 (green), S 44. The yellow band this file uses for the simulator is
H 18..38, S >= 110, and against a real ZED frame it admits ZERO pixels. Since
`detect()` reaches the ring, cross and concentricity checks only through the
yellow mask, an empty yellow mask means the cascade never runs at all.

The FIELD moves further than any band is wide. Floor saturation is 142 in one
camera and 220 in the other; the pad's own field is 104 and 172. A band cut to
separate them in one frame merges them in the other.

Nor is the floor separable by colour. Raw ZED capture, 2026-08-23, has a global
BLUE cast, and in it the white wall reads more blue-dominant than the far mat
(opponent -39 against -28). An earlier version of this mode gated on "the pad
must lie on blue-dominant floor"; measured across 44 labelled frames that gate
selected 80-94% of every frame, walls included. It has been removed rather than
retuned: there is no threshold that separates a blue-lit wall from blue foam.

WHAT THE REAL MODE USES INSTEAD
-------------------------------
1. MARKINGS, by CONTRAST rather than by hue. Take the opponent channel

       yb = (R + G)/2 - B

   and subtract its own local mean over a large box. Blue floor and blue field
   both sit far below their neighbourhood; paint sits above it. This is a
   colour DIFFERENCE measured against a LOCAL reference, so a white-balance
   shift, an exposure change, or the slow horizontal banding the ZED shows
   under the arena's lights all move signal and reference together and cancel.

2. HYPOTHESES, as ELLIPSES. Every candidate is an ellipse, because the pad's
   circle under perspective is one and because an ellipse carries the
   foreshortening that a circular model throws away. They come from two places,
   since the pad is two different things at the two ranges that matter:

     RING FIT     the circle resolves as its own marking component. Fitting an
                  ellipse to it puts the centre exactly on the pad. This is the
                  belly camera at landing height.
     CLUSTER FIT  everything -- border, ring, cross -- merges into one small
                  patch, which is the forward ZED across the arena. The ring
                  cannot be isolated, so the ellipse is fitted to the whole
                  cluster instead. Coarser, but it still carries the
                  foreshortening.

   The link distance that decides what clusters with what cannot be derived
   from the frame -- it scales with the pad, and the pad is what we are looking
   for -- so the clustering runs at three radii and overlap suppression settles
   it. A ring fit outranks a cluster fit over the same pad (`ring_bonus`),
   because the winner is the position that gets projected into the world.

3. ONE VERIFIER, in the candidate ellipse's own normalised coordinates. Sweep
   72 rays out to normalised radius 1.25 and ask the same three questions the
   simulator's checks ask, but scaled by the ellipse rather than by a circle:

     ring_cov  fraction of rays that meet any marking. A ring -> ~1.0.
     arms      angular lobes of marking between 0.25 and 0.55 of the OUTERMOST
               marking found along each ray. The cross gives four. A solid
               patch gives one, a bare ring none. Measuring against each ray's
               own outer radius is what lets the same band work whether the
               outermost marking is the square border or the circle -- which is
               why this mode needs no separate handling for the border, and why
               the ROI no longer has to be shrunk to hide it.
     seen      fraction of the sweep that stayed inside the image. The belly
               camera at landing height sees a pad whose centre is off-frame;
               the forward camera never should. It is a per-camera threshold,
               not a gate with one right value.

4. THE FIELD IS DARKER THAN THE MAT. A pad is dark ground carrying bright
   paint, lying on lighter foam, so median(inside) - median(outside) over
   non-marking pixels is negative for a pad under any illumination -- two
   medians from the same frame, so a colour cast cancels. Measured over the
   labelled frames: every pad -25..-2, while the bright window lattice that
   was outranking pads sits at +7..+95.

5. STRENGTH. `mark_delta` is set low so faint far-away paint still enters the
   mask; this asks how far the markings actually cleared it. Real paint clears
   it wide, a stain only just does.

6. AIRFRAME. A fixed-mount camera sees parts of the drone. The belly camera's
   landing legs intrude at two corners and score up to 0.95 -- a dark object
   with a bright edge on blue foam is, to every test above, a small pad.
   Nothing in one frame separates them, so `ignore_regions` blanks them by
   position. It is empty by default and must be measured per airframe.

Why not just "find the markings"
--------------------------------
Anything pale on a blue mat would pass: a scuff, a cable, a reflection. The
structure IS the identity of the pad, so the detector spends most of its effort
proving that a cluster of paint really is a concentric ring and cross, and
reports a confidence built from those checks rather than a bare yes/no.

MEASURED, ON 44 LABELLED FRAMES FROM THE ARENA
----------------------------------------------
Two ZED clips and two belly-camera clips, 2026-08-23, hand-labelled. Scored on
whether the BEST-RANKED detection is the pad, which is what the mission acts on:

                    top-1   mis-ranked   missed   on empty frames
    shipped before   8/42        4         30            0
    this version    32/42        3          7            1

The belly camera is 18/18 with nothing mis-ranked. The forward camera is 14/24;
what it misses is the pad seen almost edge-on at the far wall, 9:1 foreshortened
and a few thousand pixels in area.

Raising the caller's threshold to 0.70 trades recall for accuracy exactly as it
should: 26/42, one mis-ranked, and centre error p90 falls from 72 px to 52 px.
That is deliberate -- see `ecc_penalty`.

KNOWN CONFUSERS
---------------
Three things in the arena pass every check here, and none can be separated from
a pad by appearance in a single frame:

  A SOLAR PANEL leaning on the wall: blue, rectangular, ruled with a pale grid.
  A CABLE lying on the mat in a loop: a pale closed curve on blue.
  THE DRONE'S OWN LEGS in the belly camera (handled by `ignore_regions`).

For the first two the discriminator is geometry, not appearance -- both are off
the floor plane or lack a cross -- and the forward camera already has depth. A
ground-plane gate on the back-projected point in pad_detector_node is the right
place for it. NOT IMPLEMENTED. docs/LANDING-SITES.md 3 has the measurements.

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
# Not one of these is a colour. They are contrasts, proportions and counts,
# which is the whole point: the module docstring has the measurements that rule
# an absolute colour test out, for the markings and for the floor alike.

# How far above its own neighbourhood (R+G)/2 - B has to rise to count as
# paint. Deliberately low, so faint far-away markings still enter the mask;
# DEFAULT_MARK_CONTRAST_MULT is the second look that throws out what only just
# cleared it.
DEFAULT_MARK_DELTA = 8.0
# Radius of that neighbourhood, as a fraction of the frame's longer side. It
# has to be comfortably wider than a marking stroke, or a stroke averages into
# its own reference and stops standing out.
DEFAULT_MARK_WINDOW_FRAC = 0.06
# How far real paint must clear mark_delta, as a multiple of it. Raw ZED
# capture puts a real pad's markings at 19 against a stain's 10-15, so this
# sits between them. An earlier value of 2.5, calibrated on photographs of a
# MONITOR showing the debug stream (which exaggerated the contrast to 25-192),
# was a single unit too high and cost one clip 64 of its 65 frames.
DEFAULT_MARK_CONTRAST_MULT = 1.5
# Smallest semi-axis worth considering, in pixels.
DEFAULT_MIN_AXIS_PX = 18.0
# Largest, as a fraction of the frame's longer side.
DEFAULT_MAX_AXIS_FRAC = 0.60
# Link distances for clustering marking pixels, as fractions of the frame's
# longer side. Three, because the right distance scales with the pad.
DEFAULT_LINK_FRACS = (0.004, 0.009, 0.020)
# How far a contour may stray from the ellipse fitted to it, in units of that
# ellipse's radius, before it stops counting as a RING hypothesis. A ring
# measures 0.03-0.10, a cross 0.37.
DEFAULT_RING_RESID_MAX = 0.16
# Fraction of the sweep's rays whose marking reaches out to the ring. Its own
# constant rather than the simulator's ring_cov_min, because the two modes
# measure it differently: real mode asks how far out the marking REACHES, which
# is a stricter question than whether the ray met marking at all.
DEFAULT_REAL_RING_COV_MIN = 0.70
# Perspective squashes the pad; past this it is a sliver, not a pad.
DEFAULT_MAX_ECC = 6.0
# median(inside) - median(outside) must be at or below this. A pad is dark
# ground under bright paint, so the step is negative for a pad.
DEFAULT_FIELD_STEP_MAX = 0.0
# Added to a RING hypothesis's confidence, so an exact centre outranks an
# approximate one over the same pad.
DEFAULT_RING_BONUS = 0.08
# How much of the confidence a fully foreshortened pad gives up. Its centre is
# poorly determined along the short axis and the map weights by confidence, so
# this turns a slant sighting into a lead rather than a fix.
DEFAULT_ECC_PENALTY = 0.35
# Fraction of the polar sweep that has to stay inside the image. PER CAMERA:
# the belly camera at landing height sees a pad whose centre is off-frame and
# needs this low; the forward camera never should and uses it to throw out
# fragments at the frame edge.
DEFAULT_MIN_SEEN = 0.30

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
        # ── real mode ───────────────────────────────────────────────────────
        mark_delta: float = DEFAULT_MARK_DELTA,
        mark_window_frac: float = DEFAULT_MARK_WINDOW_FRAC,
        mark_contrast_mult: float = DEFAULT_MARK_CONTRAST_MULT,
        min_axis_px: float = DEFAULT_MIN_AXIS_PX,
        max_axis_frac: float = DEFAULT_MAX_AXIS_FRAC,
        link_fracs=DEFAULT_LINK_FRACS,
        ring_resid_max: float = DEFAULT_RING_RESID_MAX,
        real_ring_cov_min: float = DEFAULT_REAL_RING_COV_MIN,
        max_ecc: float = DEFAULT_MAX_ECC,
        field_step_max: float = DEFAULT_FIELD_STEP_MAX,
        ring_bonus: float = DEFAULT_RING_BONUS,
        ecc_penalty: float = DEFAULT_ECC_PENALTY,
        min_seen: float = DEFAULT_MIN_SEEN,
        ignore_regions=(),
        # ── shared ──────────────────────────────────────────────────────────
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
        roi_erode_frac: float = 0.06,
        close_px: int = 5,
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
        self.min_axis_px = float(min_axis_px)
        self.max_axis_frac = float(max_axis_frac)
        self.link_fracs = tuple(float(f) for f in link_fracs)
        self.ring_resid_max = float(ring_resid_max)
        self.real_ring_cov_min = float(real_ring_cov_min)
        self.max_ecc = float(max_ecc)
        self.field_step_max = float(field_step_max)
        self.ring_bonus = float(ring_bonus)
        self.ecc_penalty = float(ecc_penalty)
        self.min_seen = float(min_seen)
        self.ignore_regions = self._as_regions(ignore_regions)

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
        self.roi_erode_frac = float(roi_erode_frac)
        self.min_confidence = float(min_confidence)
        self.max_detections = int(max_detections)

        # Reused structuring elements — allocating these per frame is pure waste.
        # CLOSE knits the field back together across whatever cuts it. At
        # range that is only colour fringing and 5 px is plenty — but the belly
        # camera at confirmation height sees the ring and cross 10-20 px WIDE,
        # and they carve the blue field into quadrants. MEASURED 2026-09-01,
        # hovering dead centre over a real base (off by 0.00 m): every contour
        # came back sol=0.28-0.70 against a 0.80 gate with yfrac=0.000 — pieces
        # of a pad, each with the markings on its border instead of inside it.
        # Two frames in 25 s got through, where six were needed.
        k = max(3, int(close_px) | 1)          # odd, >= 3
        self._k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        self._k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        # Last frame's masks, kept only so a caller can draw them for
        # debugging. In real mode there is no field mask — the mode does not
        # segment the floor, see the module docstring — so it stays None and
        # `last_yellow_mask` holds the contrast-derived marking mask.
        self.last_field_mask: np.ndarray | None = None
        self.last_yellow_mask: np.ndarray | None = None

    @staticmethod
    def _as_regions(spec) -> tuple:
        """Accept [x0,y0,x1,y1, ...] or [(x0,y0,x1,y1), ...], all fractions.

        The flat form is what a ROS parameter can carry; the nested one is what
        is readable in a test.
        """
        if spec is None:
            return ()
        flat = list(spec)
        if flat and not np.isscalar(flat[0]):
            return tuple(tuple(float(v) for v in r) for r in flat)
        if len(flat) % 4:
            raise ValueError("ignore_regions needs groups of four fractions "
                             f"(x0, y0, x1, y1); got {len(flat)} values")
        return tuple(tuple(float(v) for v in flat[i:i + 4])
                     for i in range(0, len(flat), 4))

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
        """(markings, contrast) for the REAL pad. Neither is a hue test.

        `markings` is the binary paint mask; `contrast` is how far each pixel
        rose above its own local mean, which is what separates paint from a
        stain. Exposed because these two are the first thing to look at when
        the arena's light changes and the detector goes quiet.
        """
        opp = self._opponent(bgr)
        win = self._odd(self.mark_window_frac * max(bgr.shape[:2]), 9)
        local = cv2.boxFilter(opp, -1, (win, win), normalize=True,
                              borderType=cv2.BORDER_REPLICATE)
        contrast = opp - local
        mark = (contrast > self.mark_delta).astype(np.uint8) * 255
        mark = cv2.morphologyEx(mark, cv2.MORPH_OPEN, self._k_open)
        self._blank_airframe(mark)
        return mark, contrast

    # (B, G, R) -> (R + G)/2 - B, the yellow-vs-blue opponent channel.
    _OPPONENT = np.array([[-1.0, 0.5, 0.5]])

    def _opponent(self, bgr: np.ndarray) -> np.ndarray:
        """The opponent channel, in one pass over the frame.

        One `cv2.transform` of a float copy, rather than splitting the frame
        into channels: the split alone cost more than every morphological
        operation in this file put together.
        """
        return cv2.transform(bgr.astype(np.float32), self._OPPONENT)

    def _blank_airframe(self, mark: np.ndarray) -> None:
        """Erase the parts of the frame the drone occupies. In place."""
        h, w = mark.shape
        for x0, y0, x1, y1 in self.ignore_regions:
            mark[max(0, int(y0 * h)):max(0, int(y1 * h)),
                 max(0, int(x0 * w)):max(0, int(x1 * w))] = 0

    # ────────────────────────────────────────────────────────────────────────
    # Simulated pad: find the blue field, then prove it carries the markings
    # ────────────────────────────────────────────────────────────────────────

    def _detect_sim(self, bgr: np.ndarray) -> list[PadDetection2D]:
        field_mask, yellow = self.color_masks(bgr)
        self.last_field_mask, self.last_yellow_mask = field_mask, yellow

        img_area = float(bgr.shape[0] * bgr.shape[1])
        contours, _ = cv2.findContours(field_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        # WHERE THE CASCADE LOSES THEM. Every gate below returns the same
        # empty list, and an empty list cannot say which gate fired. Counted
        # here so the node can print it.
        self.reject = dict(contours=len(contours), area=0, solidity=0,
                           aspect=0, no_yellow=0, yellow_frac=0,
                           concentric=0, ring_cross=0, confidence=0)
        # The COUNTS say which gate fired; they do not say by how much. For the
        # three biggest blobs, measure the same quantities the gates test and
        # keep them, so a threshold can be moved by a number instead of by
        # guesswork.
        self.probe = []
        for c in sorted(contours, key=cv2.contourArea, reverse=True)[:3]:
            a = float(cv2.contourArea(c))
            if a < 1.0:
                continue
            hull = float(cv2.contourArea(cv2.convexHull(c)))
            (_, _), (rw, rh), _ = cv2.minAreaRect(c)
            x, y, w, h = cv2.boundingRect(c)
            roi = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(roi, [c], -1, 255, cv2.FILLED, offset=(-x, -y))
            ypx = int(cv2.countNonZero(cv2.bitwise_and(
                yellow[y:y + h, x:x + w], roi)))
            self.probe.append(
                f"area={a / img_area:.3f} sol={a / hull if hull else 0:.2f} "
                f"asp={max(rw, rh) / max(min(rw, rh), 1e-6):.1f} "
                f"yfrac={ypx / a:.3f}")

        found: list[PadDetection2D] = []
        for contour in contours:
            det = self._evaluate(contour, yellow, img_area)
            if det is not None:
                found.append(det)
        return found

    # ────────────────────────────────────────────────────────────────────────
    # Real pad: propose ellipses, then prove each one is a pad
    # ────────────────────────────────────────────────────────────────────────

    def _detect_real(self, bgr: np.ndarray) -> list[PadDetection2D]:
        mark, contrast = self.real_masks(bgr)
        self.last_field_mask, self.last_yellow_mask = None, mark
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        max_axis = self.max_axis_frac * max(bgr.shape[:2])

        found: list[PadDetection2D] = []
        for centre, semi, rot, resid, source in self._hypotheses(mark, max_axis):
            ecc = max(semi) / max(min(semi), 1e-6)
            if ecc > self.max_ecc:
                continue
            checked = self._verify(mark, centre, semi, rot)
            if checked is None:
                continue
            ring_cov, arm_occ, arms, seen = checked
            if ring_cov < self.real_ring_cov_min or seen < self.min_seen:
                continue
            # Two to five lobes rather than exactly four: perspective and
            # clipping merge or split an arm often enough. One lobe is a solid
            # patch and none is a bare ring, and neither is a pad.
            if not 2 <= arms <= 5:
                continue
            strength = self._marking_strength(mark, contrast, centre, semi)
            if strength < self.mark_contrast_mult * self.mark_delta:
                continue
            step = self._field_step(gray, mark, centre, semi, rot)
            if step is None or step > self.field_step_max:
                continue

            fit_score = max(0.0, 1.0 - resid / 0.35)
            ring_score = self._ramp(ring_cov, 0.50, 0.95)
            cross_score = {2: 0.45, 3: 0.75, 4: 1.0, 5: 0.7}.get(arms, 0.2)
            confidence = (0.28 * ring_score + 0.34 * cross_score
                          + 0.23 * fit_score + 0.15 * seen)
            if source == "ring":
                confidence = min(1.0, confidence + self.ring_bonus)
            confidence *= 1.0 - self.ecc_penalty * min(
                max((ecc - 2.0) / 4.0, 0.0), 1.0)
            if confidence < self.min_confidence:
                continue

            found.append(PadDetection2D(
                u=centre[0], v=centre[1],
                radius_px=math.sqrt(semi[0] * semi[1]),
                area_px=math.pi * semi[0] * semi[1],
                confidence=float(confidence),
                contour=self._ellipse_contour(centre, semi, rot),
                scores={
                    "ring": round(ring_score, 3),
                    "cross": round(cross_score, 3),
                    "fit": round(fit_score, 3),
                    "ring_cov": round(ring_cov, 3),
                    "arm_occ": round(arm_occ, 3),
                    "arms": arms,
                    "resid": round(resid, 3),
                    "seen": round(seen, 3),
                    "ecc": round(ecc, 2),
                    "contrast": round(strength, 1),
                    "field_step": round(step, 1),
                    "source": source,
                    "semi_axes": (round(semi[0], 1), round(semi[1], 1)),
                },
            ))
        return found

    # ── Hypotheses ──────────────────────────────────────────────────────────

    def _hypotheses(self, mark: np.ndarray, max_axis: float) -> list:
        """(centre, semi, rot, residual, source) for both families."""
        out = []
        closed = cv2.morphologyEx(mark, cv2.MORPH_CLOSE, self._k_close)
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            got = self._fit(contour, max_axis)
            if got is None:
                continue
            centre, semi, rot, resid = got
            if resid <= self.ring_resid_max:
                out.append((centre, semi, rot, resid, "ring"))

        span = max(mark.shape[:2])
        for frac in self.link_fracs:
            radius = max(1, int(round(frac * span)))
            # A SQUARE kernel, not a disc: OpenCV runs a rectangular dilation
            # separably, and at the largest link radius on a 1280x720 frame the
            # disc costs 21 ms against the square's 0.7. The difference it buys
            # is a Chebyshev rather than Euclidean link distance, far finer
            # than the spacing between the three radii.
            grown = cv2.dilate(mark, cv2.getStructuringElement(
                cv2.MORPH_RECT, (2 * radius + 1,) * 2))
            cs, _ = cv2.findContours(grown, cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_NONE)
            for contour in cs:
                got = self._fit(contour, max_axis)
                if got is not None:
                    out.append(got + ("cluster",))
        return out

    def _fit(self, contour, max_axis: float):
        """Fit an ellipse and say how far the contour strays from it."""
        if len(contour) < 20:
            return None
        (cx, cy), (d1, d2), angle = cv2.fitEllipse(contour)
        semi = (d1 / 2.0, d2 / 2.0)
        if min(semi) < self.min_axis_px or max(semi) > max_axis:
            return None
        phi = math.radians(angle)
        rot = (math.cos(phi), math.sin(phi))
        rn = self._norm_radii(contour.reshape(-1, 2).astype(np.float32),
                              (cx, cy), semi, rot)
        return (cx, cy), semi, rot, float(np.mean(np.abs(rn - 1.0)))

    @staticmethod
    def _norm_radii(pts, centre, semi, rot) -> np.ndarray:
        """Radius of each point in the ellipse's own coordinates. 1.0 = on it."""
        cx, cy = centre
        sa, sb = semi
        c, s = rot
        dx = pts[:, 0] - cx
        dy = pts[:, 1] - cy
        return np.hypot((dx * c + dy * s) / max(sa, 1e-6),
                        (-dx * s + dy * c) / max(sb, 1e-6))

    # ── Verification ────────────────────────────────────────────────────────

    _ARM_BAND = (0.25, 0.55)
    # A ray counts toward the ring only if its outermost marking gets this
    # close to the reference radius.
    _RING_REACH = 0.70
    _R_MAX = 1.25

    def _verify(self, mark: np.ndarray, centre, semi, rot):
        """One polar sweep in the ellipse's normalised coordinates.

        Returns (ring_cov, arm_occ, arms, seen), or None when too little of the
        sweep landed inside the image to say anything.
        """
        h, w = mark.shape
        cx, cy = centre
        sa, sb = semi
        c, s = rot
        # Sample about one step per pixel along the major axis. A fixed count
        # quantises the arm band differently at every apparent size, and on the
        # labelled frames that alone moved the score by +/-3 — noise dressed up
        # as a parameter.
        n_rad = int(min(max(self._R_MAX * max(sa, sb), 32), 96))
        theta = np.linspace(0.0, 2.0 * math.pi, self._N_ANG, endpoint=False)
        rad = np.linspace(0.0, self._R_MAX, n_rad)
        u = np.cos(theta)[:, None] * rad[None, :] * sa
        v = np.sin(theta)[:, None] * rad[None, :] * sb
        xi = np.rint(cx + u * c - v * s).astype(np.int32)
        yi = np.rint(cy + u * s + v * c).astype(np.int32)
        inside = (xi >= 0) & (xi < w) & (yi >= 0) & (yi < h)
        hit = np.zeros(inside.shape, dtype=bool)
        hit[inside] = mark[yi[inside], xi[inside]] > 0

        usable = inside.mean(axis=1) > 0.7
        if np.count_nonzero(usable) < 10:
            return None
        any_hit = hit.any(axis=1)

        # The arms are looked for in a band between 0.25 and 0.55 of the
        # outermost marking, which is what makes the same band work whether the
        # outermost thing is the square border or the circle — no separate
        # handling for the border, and no shrinking of the ROI to hide it.
        #
        # That reference is ONE radius for the whole sweep, the median over
        # rays, not each ray's own. Perspective is already taken out by working
        # in the ellipse's coordinates, so the only thing a per-ray reference
        # still tracks is the SHAPE of the outermost marking — and around a
        # square that swings by sqrt(2) between edge and corner, which pushes
        # the band onto the circle in the corner directions and reports eight
        # arms for a four-armed cross. Measured on a crisp synthetic pad.
        outer = np.where(any_hit,
                         n_rad - 1 - np.argmax(hit[:, ::-1], axis=1), -1)
        probe = usable & (outer >= 4)
        if not probe.any():
            return None
        # Two references, because the two questions want different things
        # from the same numbers.
        #
        # REACH, for ring coverage: how far out the marking gets where it gets
        # furthest, so a high percentile. Ring coverage then asks how many rays
        # get near it — whether the marking REACHES OUT in every direction, not
        # merely whether the ray met marking somewhere. Those are the same
        # question only for a hollow candidate: a cross has paint over the
        # centre, so every ray hits immediately and a plain any-hit test
        # returns 1.00 for two crossing bars.
        #
        # REF, for the arm band: a robust middle, so the median. A high
        # percentile here would track the corners of a square border, whose
        # radius swings by sqrt(2), and push the band out onto the circle.
        reach = float(np.percentile(outer[probe], 80))
        ref = float(np.median(outer[probe]))
        ring_cov = float(np.count_nonzero(
            usable & any_hit & (outer >= self._RING_REACH * reach))
            / np.count_nonzero(usable))

        lo = int(round(ref * self._ARM_BAND[0]))
        hi = int(round(ref * self._ARM_BAND[1]))
        if hi <= lo:
            return None
        arm = np.zeros(self._N_ANG, dtype=bool)
        arm[probe] = hit[probe, lo:hi + 1].any(axis=1)
        arm_occ = float(np.count_nonzero(arm[probe]) / np.count_nonzero(probe))
        return ring_cov, arm_occ, self._count_lobes(arm), float(usable.mean())

    def _marking_strength(self, mark, contrast, centre, semi) -> float:
        """Median contrast of the marking pixels around the candidate."""
        h, w = mark.shape
        cx, cy = centre
        reach = self._R_MAX * max(semi)
        x0, x1 = max(0, int(cx - reach)), min(w, int(cx + reach) + 1)
        y0, y1 = max(0, int(cy - reach)), min(h, int(cy + reach) + 1)
        if x1 - x0 < 3 or y1 - y0 < 3:
            return 0.0
        chosen = mark[y0:y1, x0:x1] > 0
        if not chosen.any():
            return 0.0
        return float(np.median(contrast[y0:y1, x0:x1][chosen]))

    @staticmethod
    def _field_step(gray, mark, centre, semi, rot, outer: float = 1.9):
        """median(inside) - median(outside), over non-marking pixels.

        A pad is dark ground carrying bright paint, lying on lighter foam, so
        this is negative for a pad under any illumination. Both medians come
        from the same frame, so a colour cast cancels out of the difference.
        """
        h, w = mark.shape
        ctr = (int(centre[0]), int(centre[1]))
        axes = (max(int(semi[0]), 1), max(int(semi[1]), 1))
        angle = math.degrees(math.atan2(rot[1], rot[0]))
        inner = np.zeros((h, w), np.uint8)
        cv2.ellipse(inner, ctr, axes, angle, 0, 360, 255, -1)
        ring = np.zeros((h, w), np.uint8)
        cv2.ellipse(ring, ctr, (int(axes[0] * outer), int(axes[1] * outer)),
                    angle, 0, 360, 255, -1)
        ring = cv2.subtract(ring, cv2.dilate(inner, np.ones((9, 9), np.uint8)))
        sel_in = (inner > 0) & (mark == 0)
        sel_out = (ring > 0) & (mark == 0)
        if np.count_nonzero(sel_in) < 50 or np.count_nonzero(sel_out) < 50:
            return None
        return float(np.median(gray[sel_in]) - np.median(gray[sel_out]))

    @staticmethod
    def _ellipse_contour(centre, semi, rot) -> np.ndarray:
        """The candidate as a contour, so callers can draw it like any other."""
        angle = math.degrees(math.atan2(rot[1], rot[0]))
        pts = cv2.ellipse2Poly((int(centre[0]), int(centre[1])),
                               (max(int(semi[0]), 1), max(int(semi[1]), 1)),
                               int(angle), 0, 360, 6)
        return pts.reshape(-1, 1, 2)

    @staticmethod
    def _odd(value: float, minimum: int = 3) -> int:
        """Nearest odd integer at or above `minimum` — box filters want odd."""
        n = max(int(round(value)), minimum)
        return n if n % 2 else n + 1

    # ────────────────────────────────────────────────────────────────────────
    # Per-candidate evaluation
    # ────────────────────────────────────────────────────────────────────────

    def _rej(self, key):
        if getattr(self, "reject", None) is not None:
            self.reject[key] = self.reject.get(key, 0) + 1

    def _evaluate(self, contour, yellow,
                  img_area: float) -> PadDetection2D | None:
        """Run the check cascade on one blue-field contour. None = not a pad.

        SIMULATOR ONLY. The real mode proposes ellipses and verifies them in
        their own normalised coordinates instead — see _detect_real.
        """
        area = float(cv2.contourArea(contour))
        if area < self.min_area_px or area > self.max_area_frac * img_area:
            self._rej("area")
            return None

        # ── Check 2: shape ──────────────────────────────────────────────────
        # The pad's outer boundary is a convex quad or disc under any viewing
        # angle, so a low solidity means we grabbed something else (a shadowed
        # railing, two blobs bridged by noise).
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        solidity = area / hull_area if hull_area > 0 else 0.0
        if solidity < self.min_solidity:
            self._rej("solidity")
            return None

        # Perspective squashes the pad but never into a sliver: a long thin
        # footprint is a line marking or a wall edge, not a pad.
        (_, _), (rw, rh), _ = cv2.minAreaRect(contour)
        if min(rw, rh) <= 1e-6:
            return None
        aspect = max(rw, rh) / min(rw, rh)
        if aspect > self.max_aspect:
            self._rej("aspect")
            return None

        # Fill the contour: everything below asks questions about the pad's
        # INTERIOR, and the field mask has the ring and cross punched out of
        # it. `roi` is "the pad's footprint", holes included.
        x, y, w, h = cv2.boundingRect(contour)
        roi = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(roi, [contour], -1, 255, cv2.FILLED, offset=(-x, -y))

        # Pull the footprint in from its own edge before asking about yellow.
        # The simulated markings sit well inside the field, so this costs
        # nothing there and clears any colour fringing at the boundary.
        erode_px = int(round(self.roi_erode_frac * math.sqrt(area / math.pi)))
        if erode_px >= 1:
            k = 2 * erode_px + 1
            roi = cv2.erode(roi, cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (k, k)))

        yellow_roi = cv2.bitwise_and(yellow[y:y + h, x:x + w], roi)
        yellow_px = int(cv2.countNonZero(yellow_roi))
        if yellow_px == 0:
            self._rej("no_yellow")
            return None

        # ── Check 3: colour coexistence ─────────────────────────────────────
        # Too little marking = a plain blue object. Too much = a yellow
        # object with a blue rim, or the mask leaked.
        yellow_frac = yellow_px / area
        if not (self.yellow_frac_min <= yellow_frac <= self.yellow_frac_max):
            self._rej("yellow_frac")
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
            self._rej("concentric")
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
        # measurements say nothing — see structure_radius_px.
        resolvable = r95 >= self.structure_radius_px
        if resolvable and not (
                ring_cov >= self.ring_cov_min
                and self.mid_occ_min <= mid_occ <= self.mid_occ_max):
            self._rej("ring_cross")
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
            # The last gate, and the one a blob reaches only after passing
            # every geometric test — so its SCORE, not just the count, is what
            # says whether the threshold or the scene is wrong.
            self._rej("confidence")
            self.last_conf = (f"conf={confidence:.2f} centre={center_score:.2f} "
                              f"ring={ring_score:.2f} cross={cross_score:.2f} "
                              f"cov={ring_cov:.2f} arms={arms} "
                              f"resolvable={resolvable} r95={r95:.0f}px")
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
