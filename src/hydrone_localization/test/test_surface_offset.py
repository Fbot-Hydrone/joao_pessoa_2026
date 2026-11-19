"""surface_offset: the steps a downward rangefinder takes over bases.

    python3 -m pytest src/hydrone_localization/test/test_surface_offset.py -q
"""

import math

from hydrone_localization.surface_offset import Params, SurfaceOffset

DT = 0.05          # 20 Hz, what the FCU logs for the DroneCAN rangefinder


def fly(so, heights, surfaces, tilt_deg=0.0, vz=0.0, evidence=True, t0=0.0):
    """Feed samples of a vehicle at `heights` (above the floor) over
    `surfaces` (height of what is under the sensor). Returns outputs."""
    out = []
    a = math.radians(tilt_deg)
    for i, (h, s) in enumerate(zip(heights, surfaces)):
        r = (h - s) / math.cos(a)                 # slant range along the beam
        out.append(so.update(t0 + i * DT, r, a, 0.0, vz, evidence=evidence))
    return out


def projected(out, tilt_deg):
    """What the EKF ends up with: r_out * cos(tilt)."""
    return [ro * math.cos(math.radians(tilt_deg)) for ro, _ in out]


def test_level_flight_over_a_base_holds_the_floor_height():
    so = SurfaceOffset()
    n = 40
    surf = [0.0] * 10 + [0.5] * 20 + [0.0] * 10
    out = fly(so, [1.5] * n, surf)
    hs = [h for _, h in out]
    # Away from the two edges the floor height never moves.
    assert all(abs(h - 1.5) < 1e-6 for h in hs[:10] + hs[12:30] + hs[32:])
    assert [round(st.size, 3) for st in so.steps] == [-0.5, 0.5]
    assert so.s == 0.0


def test_the_edge_sample_halfway_is_one_step_not_two():
    """Log 46, 651.49: 1.615, 1.549, 1.121 — the beam half on the edge."""
    so = SurfaceOffset()
    rs = [1.615] * 5 + [1.549, 1.121] + [1.110] * 10
    for i, r in enumerate(rs):
        so.update(i * DT, r, 0.0, 0.0, 0.0, evidence=True)
    assert len(so.steps) == 1
    assert abs(so.s - (1.615 - 1.110)) < 0.02


def test_step_found_while_leaning_is_stored_vertical():
    """The worry: detect 0.40 while tilted, then level off and keep sending
    0.40 slant. s must be vertical; what the EKF reconstructs must be the
    floor height at ANY tilt afterwards."""
    so = SurfaceOffset()
    tilt = 15.0
    out = fly(so, [1.5] * 30, [0.0] * 10 + [0.4] * 20, tilt_deg=tilt)
    assert abs(so.s - 0.4) < 1e-6           # vertical, not 0.4/cos(15)
    # now level, still over the base
    for i, tilt2 in enumerate((0.0, 5.0, 10.0, 20.0)):
        a = math.radians(tilt2)
        r = (1.5 - 0.4) / math.cos(a)
        ro, h = so.update(2.0 + i * DT, r, a, 0.0, 0.0, evidence=True)
        assert abs(ro * math.cos(a) - 1.5) < 1e-6
        assert abs(h - 1.5) < 1e-6


def test_climbing_is_not_a_step():
    so = SurfaceOffset()
    hs = [0.5 + 0.5 * i * DT for i in range(40)]      # 0.5 m/s up
    fly(so, hs, [0.0] * 40, vz=0.5)
    assert so.steps == [] and so.s == 0.0


def test_climbing_while_crossing_an_edge():
    so = SurfaceOffset()
    hs = [1.0 + 0.5 * i * DT for i in range(40)]
    out = fly(so, hs, [0.0] * 20 + [0.5] * 20, vz=0.5)
    assert len(so.steps) == 1 and abs(so.s - 0.5) < 1e-6
    assert abs(out[-1][1] - hs[-1]) < 1e-6


def test_raised_surface_without_evidence_is_released_upwards():
    """No zone, no camera: after the timeout the offset goes, the EKF sees
    the true (shorter) range and the vehicle climbs — the safe direction."""
    p = Params(confirm_timeout_s=1.0)
    so = SurfaceOffset(p)
    fly(so, [1.5] * 60, [0.0] * 10 + [0.5] * 50, evidence=False)
    assert so.s == 0.0
    assert so.steps[0].kept is False


def test_evidence_late_but_inside_the_window_keeps_it():
    so = SurfaceOffset(Params(confirm_timeout_s=1.0))
    fly(so, [1.5] * 15, [0.0] * 10 + [0.5] * 5, evidence=False)
    fly(so, [1.5] * 30, [0.5] * 30, evidence=True, t0=15 * DT)
    assert abs(so.s - 0.5) < 1e-6 and so.steps[0].kept


def test_a_spike_that_comes_back_cancels():
    so = SurfaceOffset()
    rs = [1.5] * 5 + [3.8, 3.8] + [1.5] * 5
    for i, r in enumerate(rs):
        so.update(i * DT, r, 0.0, 0.0, 0.0, evidence=False)
    assert so.s == 0.0


def test_too_much_tilt_is_ignored():
    so = SurfaceOffset()
    a = math.radians(40)
    assert so.update(0.0, 2.0, a, 0.0, 0.0) == (None, None)


def test_starting_on_the_takeoff_base():
    so = SurfaceOffset()
    so.reset(0.5)
    # take off straight up from the 0.5 m base, then fly off its edge
    hs = [0.6 + 0.5 * i * DT for i in range(30)] + [0.6 + 0.5 * 29 * DT] * 20
    surf = [0.5] * 40 + [0.0] * 10
    for i, (h, s) in enumerate(zip(hs, surf)):
        v = 0.5 if i < 30 else 0.0
        so.update(i * DT, h - s, 0.0, 0.0, v, evidence=True)
    assert so.s == 0.0
    assert len(so.steps) == 1 and abs(so.steps[0].size - 0.5) < 1e-6


def test_an_impossible_jump_with_no_motion_is_held_as_a_glitch():
    """0.7 m in one sample while the IMU says level flight: below the floor,
    so a glitch — the EKF does not see it, and it is not a base."""
    so = SurfaceOffset(Params(confirm_timeout_s=10.0))
    rs = [1.3] * 5 + [2.0] * 5
    for i, r in enumerate(rs):
        _, h = so.update(i * DT, r, 0.0, 0.0, 0.0)
    assert abs(h - 1.3) < 1e-6
    assert so.steps[-1].reason.startswith("below floor")
