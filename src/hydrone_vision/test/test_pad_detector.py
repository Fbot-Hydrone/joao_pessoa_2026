#!/usr/bin/env python3
"""
Synthetic-image tests for hydrone_vision.pad_detector.

The point is to pin the detector's behaviour without needing UE5: the pad is a
few coloured primitives, so we can render it ourselves at any scale, rotation
and viewing angle, drop it on a noisy ground, and assert what comes back.

Run inside the stack container (it has opencv + numpy):

    docker run --rm -v $PWD:/repo -w /repo joao_pessoa_2026-hydrone:latest \
        python3 -m pytest src/hydrone_vision/test/test_pad_detector.py -q

The renders here are idealised — flat lighting, no motion blur, no JPEG. They
prove the geometry and the check cascade are right; the HSV bands still have to
be confirmed against the real arena (see docs/LANDING-SITES.md).
"""

import math
import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hydrone_vision.pad_detector import PadDetector, draw_detections  # noqa: E402


# ── Synthetic pad rendering ──────────────────────────────────────────────────

BLUE_BGR = (200, 60, 10)      # saturated blue  (HSV ~ 106, 245, 200)
YELLOW_BGR = (30, 220, 235)   # saturated yellow (HSV ~ 26, 220, 235)


def render_pad(size: int = 256) -> np.ndarray:
    """A face-on pad: blue field, yellow ring, yellow cross through the centre.

    Proportions follow the competition pad: the ring sits at ~60% of the half
    width, the stroke is ~8% of the pad, and the cross spans the ring.
    """
    img = np.zeros((size, size, 3), dtype=np.uint8)
    img[:] = BLUE_BGR

    c = size // 2
    r_ring = int(size * 0.30)
    stroke = max(int(size * 0.08), 2)

    cv2.circle(img, (c, c), r_ring, YELLOW_BGR, stroke)
    cv2.line(img, (c - r_ring, c), (c + r_ring, c), YELLOW_BGR, stroke)
    cv2.line(img, (c, c - r_ring), (c, c + r_ring), YELLOW_BGR, stroke)
    return img


def ground(width: int = 640, height: int = 480, seed: int = 0) -> np.ndarray:
    """Arena floor: desaturated speckled brown/green, nothing near blue/yellow."""
    rng = np.random.default_rng(seed)
    base = rng.integers(55, 95, size=(height, width, 3), dtype=np.uint8)
    base[:, :, 0] = (base[:, :, 0] * 0.7).astype(np.uint8)   # kill the blue ch.
    return cv2.GaussianBlur(base, (5, 5), 0)


def paste_pad(scene: np.ndarray, pad: np.ndarray, quad: np.ndarray) -> np.ndarray:
    """Warp `pad` onto the four image points `quad` (TL, TR, BR, BL) of `scene`."""
    h, w = pad.shape[:2]
    src = np.float32([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]])
    M = cv2.getPerspectiveTransform(src, np.float32(quad))

    warped = cv2.warpPerspective(pad, M, (scene.shape[1], scene.shape[0]))
    mask = cv2.warpPerspective(np.full((h, w), 255, np.uint8), M,
                               (scene.shape[1], scene.shape[0]))
    out = scene.copy()
    out[mask > 0] = warped[mask > 0]
    return out


def square_quad(cx: float, cy: float, half: float, angle_deg: float = 0.0):
    """A square of half-width `half` centred at (cx, cy), rotated in-plane."""
    a = math.radians(angle_deg)
    corners = [(-half, -half), (half, -half), (half, half), (-half, half)]
    return [(cx + x * math.cos(a) - y * math.sin(a),
             cy + x * math.sin(a) + y * math.cos(a)) for x, y in corners]


def scene_with_pad(cx=320, cy=240, half=90, angle=0.0, seed=0):
    return paste_pad(ground(seed=seed), render_pad(), square_quad(cx, cy, half, angle))


# ── Positives ────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("half", [130, 90, 60, 40, 30, 25, 20, 16, 13, 11, 9])
def test_detects_pad_across_apparent_sizes(half):
    """One pad, found, centred — from filling the frame down to ~18 px across.

    The size sweep is the whole point: the drone must pick the pad up far ahead
    on the forward camera (small) and stay locked on it while descending (huge).
    At 640x480 and 90 deg horizontal FOV, an 18 px pad is a 1 m pad about 25 m
    away — well past anything the mission needs.
    """
    det = PadDetector().detect(scene_with_pad(half=half))
    assert len(det) == 1, f"half={half}: expected 1 detection, got {len(det)}"
    assert abs(det[0].u - 320) < max(0.15 * half, 4)
    assert abs(det[0].v - 240) < max(0.15 * half, 4)


@pytest.mark.parametrize("angle", [0, 17, 45, 73, 90, 138])
def test_rotation_invariant(angle):
    """Yaw must not matter: the drone approaches from an arbitrary heading."""
    det = PadDetector().detect(scene_with_pad(half=90, angle=angle))
    assert len(det) == 1, f"angle={angle}: got {len(det)}"
    assert det[0].confidence > 0.5


def test_oblique_view_from_forward_camera():
    """A pad on the ground seen by the forward camera is a squashed trapezoid."""
    quad = [(250, 300), (400, 300), (450, 380), (200, 380)]
    scene = paste_pad(ground(), render_pad(), quad)
    det = PadDetector().detect(scene)
    assert len(det) == 1
    assert 250 < det[0].u < 400
    assert 290 < det[0].v < 390


def test_two_pads_both_found():
    """Two bases in view: both reported, so the map can hold both."""
    scene = paste_pad(ground(), render_pad(), square_quad(160, 240, 70))
    scene = paste_pad(scene, render_pad(), square_quad(480, 200, 55))
    det = PadDetector().detect(scene)
    assert len(det) == 2
    assert {round(d.u / 100) for d in det} == {2, 5}


def test_pad_clipped_by_border_still_found():
    """The blue field runs off the left edge but the ring is whole: still a pad.

    Rays that leave the ROI are excluded from the polar checks, so the visible
    part carries the decision.
    """
    scene = paste_pad(ground(), render_pad(), square_quad(75, 240, 90))
    assert len(PadDetector().detect(scene)) == 1


def test_pad_clipped_through_the_ring_is_a_known_miss():
    """DOCUMENTED LIMITATION, pinned deliberately.

    When the border cuts the ring itself, the ring stops enclosing anything: the
    blue outside it merges with the blue inside, the four sectors break out as
    their own contours, and no candidate carries a concentric ring any more.

    This is safe — a missed frame, never a false landing site — and it clears
    itself as soon as the pad moves inward, which is exactly what happens while
    the drone flies toward it. Nothing downstream depends on catching this frame.
    """
    scene = paste_pad(ground(), render_pad(), square_quad(60, 240, 90))
    assert PadDetector().detect(scene) == []


def test_confidence_rises_as_the_pad_gets_closer():
    """Confidence must climb with apparent size, so the mission gets MORE certain
    as it descends, not less."""
    far = PadDetector().detect(scene_with_pad(half=22))
    near = PadDetector().detect(scene_with_pad(half=110))
    assert far and near
    assert near[0].confidence > far[0].confidence


def test_high_confidence_implies_the_structure_was_verified():
    """The contract the mission relies on before committing to a landing.

    The cross check contributes 0.25 and is forced to zero whenever the pad is
    too small on screen to resolve the arms, so the remaining weights cap an
    unverified detection at 0.75. Anything above that has been structurally
    confirmed — which is why pad_mission_node's commit_confidence sits at 0.80.
    """
    for half in (9, 11, 13, 16, 20, 25, 30, 40, 60, 90, 130):
        for det in PadDetector().detect(scene_with_pad(half=half)):
            if det.confidence > 0.75:
                assert det.scores["resolvable"], f"half={half} scored high unverified"


# ── Negatives — the reason the structural checks exist ───────────────────────

def test_empty_ground_gives_nothing():
    assert PadDetector().detect(ground()) == []


def test_plain_blue_rectangle_rejected():
    """A blue tarp: right colour, no yellow. Must not be a landing site."""
    scene = ground()
    cv2.rectangle(scene, (240, 160), (400, 320), BLUE_BGR, -1)
    assert PadDetector().detect(scene) == []


def test_blue_with_offcentre_yellow_rejected():
    """Blue with a yellow blob near one corner — fails concentricity."""
    scene = ground()
    cv2.rectangle(scene, (240, 160), (400, 320), BLUE_BGR, -1)
    cv2.circle(scene, (270, 190), 22, YELLOW_BGR, -1)
    assert PadDetector().detect(scene) == []


def test_blue_with_solid_yellow_disc_rejected():
    """Concentric but SOLID yellow — no ring. This is the check that separates
    the pad from a generic bullseye or a yellow cone on a blue mat."""
    scene = ground()
    cv2.rectangle(scene, (240, 160), (400, 320), BLUE_BGR, -1)
    cv2.circle(scene, (320, 240), 55, YELLOW_BGR, -1)
    assert PadDetector().detect(scene) == []


def test_blue_with_ring_but_no_cross_rejected():
    """Concentric AND a real ring — but nothing crosses the middle.

    Together with the solid-disc case this brackets the cross check from both
    sides: too little yellow halfway out is as wrong as too much.
    """
    scene = ground()
    cv2.rectangle(scene, (240, 160), (400, 320), BLUE_BGR, -1)
    cv2.circle(scene, (320, 240), 55, YELLOW_BGR, 10)
    assert PadDetector().detect(scene) == []


def test_yellow_only_rejected():
    scene = ground()
    cv2.circle(scene, (320, 240), 70, YELLOW_BGR, 12)
    assert PadDetector().detect(scene) == []


def test_thin_blue_line_rejected():
    """A painted arena line: right colour, wrong aspect ratio."""
    scene = ground()
    cv2.rectangle(scene, (40, 230), (600, 250), BLUE_BGR, -1)
    cv2.circle(scene, (320, 240), 8, YELLOW_BGR, 3)
    assert PadDetector().detect(scene) == []


def test_desaturated_pad_rejected():
    """A washed-out blue/yellow print has low saturation. The rules promise a
    saturated pad, so the saturation floor is what keeps pale look-alikes out."""
    pad = render_pad()
    hsv = cv2.cvtColor(pad, cv2.COLOR_BGR2HSV)
    hsv[:, :, 1] = (hsv[:, :, 1] * 0.25).astype(np.uint8)
    pale = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    scene = paste_pad(ground(), pale, square_quad(320, 240, 90))
    assert PadDetector().detect(scene) == []


# ── Robustness ───────────────────────────────────────────────────────────────

@pytest.mark.parametrize("sigma", [4, 10, 20])
def test_survives_sensor_noise(sigma):
    """Additive noise stands in for a real camera's grain and compression."""
    rng = np.random.default_rng(7)
    scene = scene_with_pad(half=80).astype(np.int16)
    scene += rng.normal(0, sigma, scene.shape).astype(np.int16)
    scene = np.clip(scene, 0, 255).astype(np.uint8)
    det = PadDetector().detect(scene)
    assert len(det) == 1, f"sigma={sigma}: got {len(det)}"


@pytest.mark.parametrize("gain", [0.55, 0.8, 1.25])
def test_survives_exposure_changes(gain):
    """Sunlight vs shade: brightness moves, hue and saturation do not."""
    scene = np.clip(scene_with_pad(half=80).astype(np.float32) * gain,
                    0, 255).astype(np.uint8)
    assert len(PadDetector().detect(scene)) == 1


def test_motion_blur_tolerated():
    scene = cv2.blur(scene_with_pad(half=85), (9, 3))
    assert len(PadDetector().detect(scene)) == 1


def test_scores_are_populated_for_debugging():
    det = PadDetector().detect(scene_with_pad(half=100))[0]
    assert {"center", "ring", "cross", "color", "shape"} <= set(det.scores)
    assert det.scores["arms"] == 4, "the cross has four arms"
    assert det.scores["ring_cov"] > 0.95, "the ring encircles the centre"


def test_draw_detections_does_not_mutate_input():
    scene = scene_with_pad()
    before = scene.copy()
    draw_detections(scene, PadDetector().detect(scene), "forward")
    assert np.array_equal(scene, before)


def test_handles_degenerate_input():
    d = PadDetector()
    assert d.detect(None) == []
    assert d.detect(np.zeros((0, 0, 3), np.uint8)) == []
    assert d.detect(np.zeros((8, 8, 3), np.uint8)) == []


# ── The REAL pad ────────────────────────────────────────────────────────────
# Photographed 2026-08-22 in the arena, and grabbed off the ZED the same
# evening. It is NOT the simulated pad recoloured:
#
#   its field is blue of the SAME HUE as the foam floor it lies on
#   it carries a yellow square border the simulated one does not have
#   the ZED renders its paint GREEN and washed out, not yellow
#
# The last of those is what silently killed the first version. The yellow HSV
# band admitted ZERO pixels of a real ZED frame, and since every structural
# check reaches the image only through the yellow mask, the detector returned
# nothing and reported nothing. `field_mode="dark_blue"` therefore stopped
# using hue at all — see the pad_detector module docstring.
#
# Two renderings below, both from measurements rather than invention:
# render_real_pad() is the pad as the phone camera saw it, and
# render_real_pad_as_zed() is the same pad in the colours the ZED actually
# produced. A detector that only handles the first one is the detector that
# already failed in the arena.

REAL_FLOOR_BGR = (190, 110, 70)     # H~105 S~161 V~190
REAL_FIELD_BGR = (90, 70, 55)       # H~105 S~ 99 V~ 90
REAL_MARK_BGR = (45, 200, 240)      # H~ 25 S~218 V~254, saturated yellow

# The same three surfaces as the ZED rendered them, converted straight from the
# HSV medians measured on its frames: floor 103/220/190, field 101/172/146,
# paint 58/44/196. Note the paint: hue 58 is GREEN, saturation 44 is almost
# none. It is 30 hue degrees away from anything a yellow band would accept.
ZED_FLOOR_BGR = (190, 119, 26)
ZED_FIELD_BGR = (146, 110, 48)
ZED_MARK_BGR = (162, 196, 164)


def render_real_pad(side=260, w=640, h=480, cx=320, cy=240,
                    floor=REAL_FLOOR_BGR, field=REAL_FIELD_BGR,
                    mark=REAL_MARK_BGR, seam=(205, 125, 85)):
    """The real pad on its real floor. Proportions taken off the photographs."""
    img = np.full((h, w, 3), floor, np.uint8)
    for x in range(0, w, 150):
        cv2.line(img, (x, 0), (x, h), seam, 2)
    for y in range(0, h, 150):
        cv2.line(img, (0, y), (w, y), seam, 2)
    half = side // 2
    tl, br = (cx - half, cy - half), (cx + half, cy + half)
    cv2.rectangle(img, tl, br, field, -1)
    bt = max(2, side // 26)
    cv2.rectangle(img, tl, br, mark, bt)
    cv2.circle(img, (cx, cy), int(side * 0.375), mark, bt)
    arm = int(side * 0.275)
    cv2.line(img, (cx - arm, cy), (cx + arm, cy), mark, bt)
    cv2.line(img, (cx, cy - arm), (cx, cy + arm), mark, bt)
    return cv2.GaussianBlur(img, (3, 3), 0)


def render_real_pad_as_zed(**kw):
    """The same pad in the colours the ZED actually delivered in the arena."""
    kw.setdefault("floor", ZED_FLOOR_BGR)
    kw.setdefault("field", ZED_FIELD_BGR)
    kw.setdefault("mark", ZED_MARK_BGR)
    kw.setdefault("seam", (200, 130, 40))
    return render_real_pad(**kw)


def real(**kw):
    return PadDetector(field_mode="dark_blue", **kw)


# ── The real pad is found, in both cameras' colours ──────────────────────────

@pytest.mark.parametrize("render", [render_real_pad, render_real_pad_as_zed],
                         ids=["phone_colours", "zed_colours"])
def test_real_pad_is_found_in_dark_blue_mode(render):
    dets = real().detect(render())
    assert dets, "the real pad was not detected at all"
    best = dets[0]
    assert best.confidence >= 0.60, f"confidence {best.confidence:.3f} too low"
    assert abs(best.u - 320) < 20 and abs(best.v - 240) < 20


@pytest.mark.parametrize("render", [render_real_pad, render_real_pad_as_zed],
                         ids=["phone_colours", "zed_colours"])
def test_real_pad_cross_is_actually_resolved(render):
    """Four arms — not eight, and not zero.

    Eight is what a PER-RAY arm reference gives on a square pad: the outermost
    marking is the border, whose radius swings by sqrt(2) between edge and
    corner, so in the corner directions the band lands on the circle and each
    arm is counted twice. Zero is what an ROI eroded too little gives, when the
    border is taken for the ring and the probe lands outside the arms.
    """
    dets = real().detect(render())
    assert dets
    scores = dets[0].scores
    assert scores["arms"] == 4, f"arms={scores['arms']}, expected the cross's 4"
    assert 0.05 < scores["arm_occ"] < 0.85, (
        f"arm_occ={scores['arm_occ']:.3f}: at the arms' radius the pad is "
        "marked in four narrow sectors, neither nowhere nor everywhere")


def test_the_zed_colours_really_are_outside_the_yellow_band():
    """Guards the premise of every test above it.

    If someone widens the yellow band far enough to swallow hue 58, this stops
    failing and the ZED renderings quietly stop testing anything.
    """
    hsv = cv2.cvtColor(np.uint8([[ZED_MARK_BGR]]), cv2.COLOR_BGR2HSV)[0, 0]
    lo, hi = np.array([18, 110, 90]), np.array([38, 255, 255])
    assert not (np.all(hsv >= lo) and np.all(hsv <= hi)), (
        f"ZED paint {tuple(int(v) for v in hsv)} is inside the yellow band; "
        "these renderings no longer reproduce the arena failure")


# ── What the mode is for, pinned from both sides ─────────────────────────────

def test_sim_mode_does_not_find_the_real_pad():
    """Pins the bug this whole mode exists for, so it cannot regress quietly."""
    assert not PadDetector().detect(render_real_pad())
    assert not PadDetector().detect(render_real_pad_as_zed())


def test_sim_mode_finds_no_yellow_at_all_in_zed_colours():
    """The exact failure measured in the arena: an EMPTY yellow mask.

    Worth pinning separately from the detection result, because it is the
    reason nothing was detected AND the reason nothing was reported — with no
    yellow there is no candidate for any check to reject or explain.
    """
    _, yellow = PadDetector().color_masks(render_real_pad_as_zed())
    assert cv2.countNonZero(yellow) == 0


def test_real_mode_rejects_pad_markings_on_a_pale_field():
    """The pad is DARK ground under bright paint. Invert that and it is not one.

    This replaces an earlier "the markings must lie on blue floor" gate. That
    gate was measured on raw ZED capture and found to select 80-94% of every
    frame -- under the camera's blue cast a white wall is more blue-dominant
    than distant blue foam -- so the floor is not separable by colour at all.
    What does separate a pad from the bright window lattice that outranked one
    is which side of its surroundings its field sits on.
    """
    scene = render_real_pad(floor=(120, 120, 120), field=(215, 215, 215),
                            mark=(150, 235, 245), seam=(125, 125, 125))
    assert real().detect(scene) == []
    # The same geometry with the field darker than the floor IS a pad, so the
    # test is about the step and not about the drawing.
    assert real().detect(render_real_pad(floor=(215, 215, 215),
                                         field=(120, 120, 120),
                                         mark=(150, 235, 245),
                                         seam=(210, 210, 210)))


def test_real_mode_rejects_a_scuff_on_the_mat():
    """A pale mark of pad size, with no ring and no cross."""
    scene = np.full((480, 640, 3), REAL_FLOOR_BGR, np.uint8)
    cv2.ellipse(scene, (320, 240), (90, 70), 20, 0, 360, REAL_MARK_BGR, -1)
    assert real().detect(scene) == []


def test_real_mode_rejects_a_faint_stain_that_looks_structured():
    """The mask is a yes/no; how far a marking cleared it is a second signal.

    A low-contrast blotch the apparent size of a distant pad can put four lobes
    on the mid-radius probe by accident. One in the arena photographs did, at
    confidence 0.85 — above phase1_mission's commit threshold. Real paint
    clears mark_delta by a wide margin; a stain only just clears it.
    """
    scene = np.full((480, 640, 3), REAL_FLOOR_BGR, np.uint8)
    # Contrast 11.5 against the mat, squarely inside the 9-13 band every false
    # positive in the arena photographs fell into.
    faint = (REAL_FLOOR_BGR[0], REAL_FLOOR_BGR[1] + 10, REAL_FLOOR_BGR[2] + 20)
    cv2.rectangle(scene, (240, 160), (400, 320), faint, 6)
    cv2.circle(scene, (320, 240), 58, faint, 6)
    cv2.line(scene, (278, 240), (362, 240), faint, 6)
    cv2.line(scene, (320, 198), (320, 282), faint, 6)
    scene = cv2.GaussianBlur(scene, (5, 5), 0)

    assert real().detect(scene) == [], "a faint look-alike was reported as a pad"
    # Without the gate this very shape scores 0.90, so the test is about
    # CONTRAST and not about the geometry it happens to draw.
    ungated = real(mark_contrast_mult=0.0).detect(scene)
    assert ungated and ungated[0].confidence > 0.80


def test_real_pad_clears_the_contrast_gate_with_margin():
    """Both cameras' colours, so the margin is not an artefact of one of them."""
    for render in (render_real_pad, render_real_pad_as_zed):
        det = real().detect(render())[0]
        assert det.scores["contrast"] >= 2 * 2.5 * 8.0, (
            f"contrast {det.scores['contrast']} leaves no headroom over the "
            "gate; a slightly dimmer arena would lose the pad")


def test_real_mode_rejects_a_ring_with_no_cross():
    scene = np.full((480, 640, 3), REAL_FLOOR_BGR, np.uint8)
    cv2.rectangle(scene, (190, 110), (450, 370), REAL_FIELD_BGR, -1)
    cv2.rectangle(scene, (190, 110), (450, 370), REAL_MARK_BGR, 10)
    cv2.circle(scene, (320, 240), 97, REAL_MARK_BGR, 10)
    assert real().detect(scene) == []


def test_real_mode_on_bare_mat_is_quiet():
    assert real().detect(np.full((480, 640, 3), REAL_FLOOR_BGR, np.uint8)) == []


# ── Why it survives the ZED, and the sim mode did not ────────────────────────

@pytest.mark.parametrize("gains", [(1.0, 1.15, 0.85), (0.85, 1.0, 1.2),
                                   (1.2, 1.2, 0.75)])
def test_real_mode_survives_white_balance_shifts(gains):
    """Per-channel gain is what auto white balance does, and it is what moved
    the arena's paint from hue 25 to hue 58. The opponent channel is read
    against its own local mean, so a gain moves signal and reference together.
    """
    scene = render_real_pad_as_zed().astype(np.float32) * np.array(gains)
    scene = np.clip(scene, 0, 255).astype(np.uint8)
    dets = real().detect(scene)
    assert dets, f"gains={gains}: lost the pad"
    assert abs(dets[0].u - 320) < 25 and abs(dets[0].v - 240) < 25


@pytest.mark.parametrize("amplitude", [12, 25, 40])
def test_real_mode_survives_rolling_shutter_banding(amplitude):
    """The ZED shows slow horizontal bands under the arena's lights — the
    'waving' of an old CRT. They scale all three channels together, so a colour
    DIFFERENCE barely moves and its local mean absorbs the rest.
    """
    scene = render_real_pad_as_zed().astype(np.float32)
    rows = np.arange(scene.shape[0], dtype=np.float32)
    band = amplitude * np.sin(2 * math.pi * rows / 90.0)
    scene *= (1.0 + band / 255.0)[:, None, None]
    scene = np.clip(scene, 0, 255).astype(np.uint8)
    assert real().detect(scene), f"amplitude={amplitude}: lost the pad"


def test_real_mode_survives_a_vignette():
    """A slow brightness ramp across the frame — the other thing a fixed value
    threshold cannot take."""
    scene = render_real_pad_as_zed().astype(np.float32)
    cols = np.linspace(0.65, 1.25, scene.shape[1], dtype=np.float32)
    scene *= cols[None, :, None]
    scene = np.clip(scene, 0, 255).astype(np.uint8)
    assert real().detect(scene)


@pytest.mark.parametrize("side", [340, 260, 180, 120, 90])
def test_real_pad_across_apparent_sizes(side):
    dets = real().detect(render_real_pad_as_zed(side=side))
    assert dets, f"side={side}: lost the pad"
    assert abs(dets[0].u - 320) < 0.12 * side


def test_real_mode_reports_the_markings_and_their_contrast_for_debugging():
    """real_masks is the first thing to look at when the arena light changes,
    so it stays part of the public surface — the binary mask AND how far each
    pixel cleared the threshold, which is what mark_delta is tuned against."""
    det = real()
    scene = render_real_pad_as_zed()
    mark, contrast = det.real_masks(scene)
    assert cv2.countNonZero(mark) > 0
    assert contrast.shape == scene.shape[:2]
    assert contrast[mark > 0].min() > det.mark_delta
    det.detect(scene)
    # Real mode does not segment the floor, so there is no field mask to show.
    assert det.last_field_mask is None
    assert det.last_yellow_mask is not None


# ── What the two cameras each need ───────────────────────────────────────────

def test_centre_comes_from_the_ring_when_the_ring_resolves():
    """The reported pixel is projected into the world, so it has to be the
    pad's centre and not the middle of whatever paint was grouped together."""
    det = real().detect(render_real_pad(side=300))[0]
    assert det.scores["source"] == "ring"
    assert math.hypot(det.u - 320, det.v - 240) < 3.0, (
        f"centre ({det.u:.0f},{det.v:.0f}) is off the pad's true centre")


def test_a_ring_fit_outranks_a_cluster_fit_on_the_same_pad():
    """Both families propose the same pad; the exact one must win, because the
    winner is what gets projected."""
    dets = real(min_confidence=0.0).detect(render_real_pad())
    assert dets and dets[0].scores["source"] == "ring"


@pytest.mark.parametrize("cy,tag", [(150, "bottom half"), (60, "a sliver"),
                                    (20, "barely an arc")])
def test_partly_visible_pad_is_found_from_the_belly_camera(cy, tag):
    """At landing height the pad no longer fits in the belly camera's view.

    An arc of the circle plus the cross is enough, and the fitted ellipse puts
    the centre where the pad's centre actually is — even when that is outside
    the image, which is the case the mission descends into.
    """
    scene = render_real_pad_as_zed(side=300, cy=cy)
    dets = real(min_seen=0.30).detect(scene)
    assert dets, f"{tag}: lost the pad"
    assert dets[0].scores["source"] == "ring", f"{tag}: not from the arc"
    assert abs(dets[0].u - 320) < 15 and abs(dets[0].v - cy) < 15, (
        f"{tag}: centre ({dets[0].u:.0f},{dets[0].v:.0f}) should be near "
        f"(320,{cy})")


def test_cross_alone_is_not_enough_for_the_belly_camera():
    """A deliberate limit, not an oversight.

    Below the height where any of the circle is in shot, all that is left is
    two crossing bars, and a mat seam crossing another seam forges that. The
    belly camera's answer gates a landing, so it says no.
    """
    # A pad rendered far larger than the frame: the circle and the border are
    # entirely outside it and only the cross is in shot, which is what the
    # belly camera sees in the last stretch of a descent.
    for side in (900, 1200, 1600):
        assert real(min_seen=0.0).detect(
            render_real_pad_as_zed(side=side)) == [], f"side={side}"


def test_min_seen_is_what_separates_the_two_cameras():
    """Which candidate WINS, not whether anything is found.

    On a pad hanging off the top of the frame there are two readings: the arc,
    whose sweep runs off the image but whose centre is right, and a compact
    cluster wholly inside the frame whose centre is 60-80 px low. `min_seen`
    chooses. The belly camera wants the arc; the forward camera, which should
    never be looking at a clipped pad and whose answer becomes a world
    position, would rather have neither.
    """
    scene = render_real_pad_as_zed(side=300, cy=40)
    belly = real(min_seen=0.30).detect(scene)
    forward = real(min_seen=0.85).detect(scene)
    assert belly and belly[0].scores["source"] == "ring"
    assert abs(belly[0].v - 40) < 15, "belly reading should be on the centre"
    assert forward and forward[0].scores["source"] == "cluster"
    assert forward[0].v - 40 > 40, (
        "the fully-visible cluster reading is the biased one; if it stops "
        "being biased this test is measuring nothing")


def test_ignore_regions_blanks_the_airframe():
    """The belly camera sees the drone's own legs, and a dark object with a
    bright edge on blue foam passes every test in the detector — one scored
    0.95. Nothing in a single frame separates them, so they go by position."""
    scene = render_real_pad_as_zed(side=200, cx=460, cy=300)
    assert real().detect(scene), "the pad itself must survive"
    covering_the_pad = [0.5, 0.4, 1.0, 1.0]
    assert real(ignore_regions=covering_the_pad).detect(scene) == []
    # Same rectangles, expressed the way a ROS parameter carries them.
    assert real(ignore_regions=[(0.5, 0.4, 1.0, 1.0)]).detect(scene) == []


def test_ignore_regions_rejects_a_malformed_parameter():
    with pytest.raises(ValueError):
        PadDetector(field_mode="dark_blue", ignore_regions=[0.1, 0.2, 0.3])


def test_foreshortening_is_paid_for_in_confidence():
    """A pad squashed by perspective is still a pad, but its centre is loose
    along the short axis. pad_map weights by confidence, so a slant sighting
    has to arrive as a lead rather than as a fix."""
    square = real().detect(render_real_pad_as_zed(side=280))
    squashed = cv2.resize(render_real_pad_as_zed(side=280), (640, 160))
    slant = real(min_confidence=0.0).detect(squashed)
    assert square and slant
    assert slant[0].scores["ecc"] > 2.0
    assert slant[0].confidence < square[0].confidence - 0.10, (
        f"slant {slant[0].confidence:.2f} vs square {square[0].confidence:.2f}: "
        "foreshortening cost nothing")


def test_field_mode_rejects_nonsense():
    with pytest.raises(ValueError):
        PadDetector(field_mode="purple")
