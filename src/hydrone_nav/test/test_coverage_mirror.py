#!/usr/bin/env python3
"""sweep_side — which hand the mapping sets off on, relative to the nose.

    python3 -m pytest src/hydrone_nav/test/test_coverage_mirror.py
"""

import math

import pytest

from hydrone_nav import coverage

BOUNDS = ((-5.0, -5.0, 0.3), (5.0, 5.0, 2.5))
INSET = 1.2
HEADINGS_DEG = [90.0, 91.0, 0.0, 180.0, -90.0, 120.0]   # 91: the drone


def _nearest_corner(bounds, here):
    """phase1_mission_node._nearest_corner, on already-framed inputs."""
    (min_x, min_y, _), (max_x, max_y, _) = bounds
    i = INSET
    corners = [(min_x + i, min_y + i), (max_x - i, min_y + i),
               (max_x - i, max_y - i), (min_x + i, max_y - i)]
    return coverage.nearest_corner_index(corners, here)


def _sweep(builder, yaw, side, home=(0.03, -0.05), **kw):
    """What the mission flies: framed bounds, framed start corner, back out."""
    b = coverage.sweep_frame_bounds(BOUNDS, yaw, side)
    if builder is not coverage.camera_lawnmower:
        kw["start_corner"] = _nearest_corner(
            b, coverage.sweep_frame_xy(home, yaw, side))
        kw["inset_m"] = INSET
    return coverage.sweep_frame_path(builder(b, z=2.5, **kw), yaw, side)


def _body(p, yaw, home=(0.0, 0.0)):
    """A map point as (ahead, left) of the nose at takeoff, about the arena
    centre (the takeoff point, give or take pose noise)."""
    dx, dy = p[0] - home[0], p[1] - home[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * dx + s * dy, -s * dx + c * dy)


BUILDERS = [
    (coverage.perimeter_sweep, {}),
    (coverage.u_sweep, {}),
    (coverage.camera_lawnmower, dict(swath_m=1.9, margin_m=INSET)),
]


@pytest.mark.parametrize("deg", HEADINGS_DEG)
@pytest.mark.parametrize("builder,kw", BUILDERS)
def test_right_is_the_same_relative_to_the_nose_at_any_heading(
        deg, builder, kw):
    """The map can be turned any way under the vehicle; the sweep is not."""
    ref = _sweep(builder, coverage.DESIGN_HEADING, "right", **kw)
    got = _sweep(builder, math.radians(deg), "right", **kw)
    snapped = coverage.quarter_turns(math.radians(deg)) * math.pi / 2.0
    for a, b in zip(ref, got):
        ba = _body(a, coverage.DESIGN_HEADING)
        bb = _body(b, snapped)
        assert bb[0] == pytest.approx(ba[0], abs=0.1)
        assert bb[1] == pytest.approx(ba[1], abs=0.1)


@pytest.mark.parametrize("deg", HEADINGS_DEG)
@pytest.mark.parametrize("builder,kw", BUILDERS)
def test_left_is_the_mirror_of_right_across_the_nose(deg, builder, kw):
    yaw = math.radians(deg)
    snapped = coverage.quarter_turns(yaw) * math.pi / 2.0
    right = _sweep(builder, yaw, "right", **kw)
    left = _sweep(builder, yaw, "left", **kw)
    assert len(left) == len(right) > 0
    for r, l in zip(right, left):
        br, bl = _body(r, snapped), _body(l, snapped)
        assert bl[0] == pytest.approx(br[0], abs=0.1)
        assert bl[1] == pytest.approx(-br[1], abs=0.1)
        assert l[2] == r[2]
        # heading mirrored too: relative yaw negated
        rr, rl = r[3] - snapped, l[3] - snapped
        assert math.sin(rl) == pytest.approx(-math.sin(rr), abs=1e-9)
        assert math.cos(rl) == pytest.approx(math.cos(rr), abs=1e-9)


@pytest.mark.parametrize("deg", HEADINGS_DEG)
@pytest.mark.parametrize("builder,kw", BUILDERS)
def test_first_leg_runs_off_on_the_requested_hand(deg, builder, kw):
    """The first leg of the sweep proper (corner to corner, lane end to lane
    end) runs to the right for 'right' and to the left for 'left'."""
    yaw = math.radians(deg)
    snapped = coverage.quarter_turns(yaw) * math.pi / 2.0
    for side, sign in (("right", -1.0), ("left", +1.0)):
        path = _sweep(builder, yaw, side, **kw)
        lateral = (_body(path[1], snapped)[1] - _body(path[0], snapped)[1])
        assert sign * lateral > 0.5, (side, path[:2])


def test_right_is_the_sweep_as_it_always_flew():
    """At the design heading 'right' changes nothing: the original call."""
    for builder, kw in BUILDERS:
        kw = dict(kw)
        b = coverage.sweep_frame_bounds(BOUNDS, coverage.DESIGN_HEADING,
                                        "right")
        assert b == BOUNDS
        if builder is not coverage.camera_lawnmower:
            kw.update(start_corner=0, inset_m=INSET)
        orig = builder(BOUNDS, z=2.5, **kw)
        got = coverage.sweep_frame_path(orig, coverage.DESIGN_HEADING,
                                        "right")
        assert got == orig


@pytest.mark.parametrize("deg", HEADINGS_DEG)
@pytest.mark.parametrize("side", ["right", "left"])
@pytest.mark.parametrize("builder,kw", BUILDERS)
def test_stays_in_the_box(deg, side, builder, kw):
    (min_x, min_y, _), (max_x, max_y, _) = BOUNDS
    for x, y, _, _ in _sweep(builder, math.radians(deg), side, **kw):
        assert min_x <= x <= max_x and min_y <= y <= max_y


def test_a_tie_goes_to_corner_zero_not_to_pose_noise():
    corners = [(-3.8, -3.8), (3.8, -3.8), (3.8, 3.8), (-3.8, 3.8)]
    for here in [(0.0, 0.0), (0.03, -0.05), (-0.2, 0.3), (0.1, 0.1)]:
        assert coverage.nearest_corner_index(corners, here) == 0
    assert coverage.nearest_corner_index(corners, (3.0, 3.5)) == 2
