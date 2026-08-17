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
