"""Known map, beams, guard, localizer, pose mux: unit tests on small synthetic cases."""
import numpy as np
import pytest

from hydrone_confined.core.beam_localizer import BeamLocalizer, LocalizerParams, distance_field
from hydrone_confined.core.beams import BeamParams, beams, level, sector_min, vertical_clearance, wall_yaw
from hydrone_confined.core.guard import guard
from hydrone_confined.core.known_map import KnownMap
from hydrone_confined.core.pose_mux import PoseMux, compose, inverse

# a 4 x 2 m box with a wall across at x = 2, a doorway in it (y 0.6..1.4) and a
# window high in the right end wall
BOX = {
    'structure': {'x': [0, 4], 'y': [0, 2], 'height': 1.5},
    'walls': [
        {'name': 'left', 'a': [0, 0], 'b': [0, 2], 't': 0.0},
        {'name': 'right', 'a': [4, 0], 'b': [4, 2], 't': 0.0,
         'openings': [{'name': 'win', 'u': [0.6, 1.4], 'z': [0.7, 1.5], 'role': 'exit'}]},
        {'name': 'bottom', 'a': [0, 0], 'b': [4, 0], 't': 0.0},
        {'name': 'top', 'a': [0, 2], 'b': [4, 2], 't': 0.0},
        {'name': 'mid', 'a': [2, 0], 'b': [2, 2], 't': 0.06,
         'openings': [{'name': 'door', 'u': [0.6, 1.4], 'z': [0.0, 1.5]}]},
    ],
    'rooms': [{'name': 'a', 'x': [0, 2], 'y': [0, 2]}, {'name': 'b', 'x': [2, 4], 'y': [0, 2]}],
}


def box_points(km, xy, yaw, z, n=4000, seed=0, noise=0.0):
    """Body-frame points a 2D-ish lidar at (xy, z) would return, from the map's own ray cast."""
    rng = np.random.default_rng(seed)
    az = rng.uniform(-np.pi, np.pi, n)
    dz = rng.uniform(-0.1, 0.13, n)
    r = km.raycast(np.array([xy]), (yaw + az)[None, :], (z - 0.12, z + 0.15))[0]
    ok = np.isfinite(r)
    r = r[ok] + rng.normal(0, noise, ok.sum())
    return np.column_stack([r * np.cos(az[ok]), r * np.sin(az[ok]), dz[ok]])


def test_openings_depend_on_height():
    km = KnownMap.from_dict(BOX)
    right = km.walls[1]
    assert right.solid_spans(0.2, 0.4) == [(0.0, 2.0)]              # below the sill
    assert right.solid_spans(0.9, 1.2) == [(0.0, 0.6), (1.4, 2.0)]   # through the window
    # at 1.0 m the ray through the window leaves the box, at 0.3 m it hits the sill
    r_hi = km.raycast(np.array([[3.0, 1.0]]), np.array([[0.0]]), (0.95, 1.05))[0, 0]
    r_lo = km.raycast(np.array([[3.0, 1.0]]), np.array([[0.0]]), (0.25, 0.35))[0, 0]
    assert np.isinf(r_hi) and r_lo == pytest.approx(1.0)


def test_raycast_thickness_and_doorway():
    km = KnownMap.from_dict(BOX)
    band = (0.4, 0.6)
    # from (1, 0.3) straight at the mid wall: hits its near face at x = 1.97
    assert km.raycast(np.array([[1.0, 0.3]]), np.array([[0.0]]), band)[0, 0] == pytest.approx(0.97)
    # through the doorway, to the right end wall
    assert km.raycast(np.array([[1.0, 1.0]]), np.array([[0.0]]), band)[0, 0] == pytest.approx(3.0)


def test_rasterize_and_distance_field():
    km = KnownMap.from_dict(BOX)
    occ, origin, res = km.rasterize((0.4, 0.6), res=0.05, margin=0.2)
    ij = np.floor((np.array([2.0, 0.3]) - origin) / res).astype(int)
    assert occ[ij[0], ij[1]]
    ij = np.floor((np.array([2.0, 1.0]) - origin) / res).astype(int)
    assert not occ[ij[0], ij[1]]                                     # the doorway
    d = distance_field(np.eye(5, dtype=bool))
    assert d[0, 0] == 0 and d[0, 2] == pytest.approx(np.sqrt(2))


def test_beams_ranges_and_margin():
    km = KnownMap.from_dict(BOX)
    pts = box_points(km, (1.0, 0.5), 0.0, 0.5)
    b = beams(pts, BeamParams(n=4))
    # forward: mid wall face at x = 1.97, left: top wall 1.5 away, back: 1.0, right: 0.5
    assert b.ranges[0] == pytest.approx(0.97, abs=0.03)
    assert b.ranges[1] == pytest.approx(1.5, abs=0.03)
    assert b.ranges[2] == pytest.approx(1.0, abs=0.03)
    assert b.ranges[3] == pytest.approx(0.5, abs=0.03)
    # out of the band -> nothing
    pts2 = pts.copy()
    pts2[:, 2] = 1.0
    assert not np.isfinite(beams(pts2, BeamParams(n=4)).ranges).any()


def test_levelling_undoes_tilt():
    km = KnownMap.from_dict(BOX)
    pts = box_points(km, (1.0, 1.0), 0.0, 0.5)
    pts[:, 2] = 0.0
    roll, pitch = 0.1, -0.15
    from hydrone_confined.core.beams import rot_rp
    tilted = pts @ rot_rp(roll, pitch)      # what a tilted sensor would report
    assert np.allclose(level(tilted, roll, pitch), pts, atol=1e-9)


def test_vertical_clearance():
    rng = np.random.default_rng(1)
    # a ceiling 0.9 m up, seen where a Mid-360 looks (up to 52 deg)
    h = rng.uniform(0.7, 1.4, 600)
    a = rng.uniform(-np.pi, np.pi, 600)
    ceil = np.column_stack([h * np.cos(a), h * np.sin(a), np.full(600, 0.9)])
    # walls 0.5 m away, below it
    wz = rng.uniform(0.0, 0.5 * np.tan(np.deg2rad(52)), 600)
    walls = np.column_stack([0.5 * np.cos(a), 0.5 * np.sin(a), wz])
    u, d = vertical_clearance(np.vstack([ceil, walls]))
    assert u == pytest.approx(0.9, abs=0.01) and d is None
    # low in a small room: the top rays end on the walls, there's no ceiling in view
    u, _ = vertical_clearance(walls)
    assert u is None


def test_wall_yaw_finds_the_axis():
    km = KnownMap.from_dict(BOX)
    for yaw in (0.0, 0.3, -0.6, 1.2):
        pts = box_points(km, (1.0, 0.8), yaw, 0.5, n=3000)
        th, score = wall_yaw(pts, band=(-0.2, 0.2))
        # body-frame wall axis = map axis (0) - yaw, mod 90
        err = (th - (-yaw) + np.pi / 4) % (np.pi / 2) - np.pi / 4
        assert abs(err) < np.deg2rad(1.0), (yaw, th)
        assert score > 2.0


def test_guard_caps_approach_and_keeps_sliding():
    km = KnownMap.from_dict(BOX)
    pts = box_points(km, (1.0, 0.35), 0.0, 0.5)          # 0.35 m above the bottom wall
    sec = sector_min(pts, n=16)
    v, closest, _ = guard(np.array([0.3, -0.3]), 0.0, sec)
    assert closest == pytest.approx(0.35, abs=0.05)
    assert v[1] > -0.1          # into the wall: mostly removed
    assert v[0] > 0.25          # along it: kept


def test_localizer_converges_and_tracks():
    km = KnownMap.from_dict(BOX)
    loc = BeamLocalizer(km, LocalizerParams(seed=3))
    true = np.array([1.2, 0.7])
    yaw = 0.4
    loc.init_gaussian(true + [0.25, -0.2], yaw - 0.2, sigma_xy=0.25, sigma_yaw=0.2)
    bp = BeamParams(n=24)
    for k in range(25):
        pts = box_points(km, true, yaw, 0.5, seed=k, noise=0.01)
        loc.predict(np.zeros(2))
        loc.update(beams(pts, bp), (0.38, 0.65), yaw_meas=wall_yaw(pts, band=(-0.2, 0.2)))
    xy, y, cov = loc.estimate()
    assert np.linalg.norm(xy - true) < 0.05, xy
    assert abs(y - yaw) < np.deg2rad(2)
    assert not loc.lost
    # walk through the doorway into room b, odometry 20% short
    for k in range(40):
        step = np.array([0.05, 0.0])
        true = true + np.array([np.cos(yaw), np.sin(yaw)]) * 0.05 if k < 20 else true + [0.05, 0.0]
        d_body = step if k < 20 else np.array([np.cos(-yaw), np.sin(-yaw)]) * 0.05
        loc.predict(d_body * 0.8)
        pts = box_points(km, true, yaw, 0.5, seed=100 + k, noise=0.01)
        loc.update(beams(pts, bp), (0.38, 0.65), yaw_meas=wall_yaw(pts, band=(-0.2, 0.2)))
    xy, y, _ = loc.estimate()
    assert np.linalg.norm(xy - true) < 0.08, (xy, true)


def _run_mux(mux, t0, t1, odom_fn, map_fn, healthy=True, dt=0.1, delay=0.0):
    """Feed odometry and map poses at 10 Hz; returns the output poses."""
    out = []
    t = t0
    while t < t1 - 1e-9:
        mux.on_odom(t, odom_fn(t))
        if map_fn is not None:
            mux.on_map(t, map_fn(t - delay), healthy, stamp=t - delay)
        out.append(mux.pose().copy())
        t += dt
    return np.array(out)


def test_mux_output_never_steps_and_converges_to_the_map():
    mux = PoseMux()
    mux.set_anchor([1.0, 0.5, 0.0, np.pi / 2], [0.0, 0.0, 0.0, 0.0])
    still = lambda t: np.zeros(4)  # noqa: E731
    out = _run_mux(mux, 0.0, 1.0, still, None)
    assert np.allclose(out[-1], [1.0, 0.5, 0.0, np.pi / 2])
    # the map says 0.2 m further along x; asked for the map, the output walks there
    mux.request('map', 1.0)
    out = _run_mux(mux, 1.0, 5.0, still, lambda t: np.array([1.2, 0.5, 0.0, np.pi / 2]))
    steps = np.linalg.norm(np.diff(out[:, :2], axis=0), axis=1)
    assert steps.max() <= 0.1 + 1e-9                  # 1 m/s cap, never a jump
    assert abs(out[-1, 0] - 1.2) < 0.005
    assert mux.mode == 'map'
    # the map goes quiet: the anchor holds, nothing moves
    out = _run_mux(mux, 5.0, 7.0, still, None)
    assert np.allclose(out[-1], out[0]) and abs(out[-1, 0] - 1.2) < 0.005
    assert mux.mode == 'odom' and any('stale' in m for _, m in mux.events)


def test_mux_holds_a_sliding_odometry_to_the_walls():
    mux = PoseMux()
    mux.request('map')
    truth = np.array([3.0, 1.0, 0.7, 0.0])
    slide = lambda t: np.array([0.15 * t, 0.0, 0.0, 0.0])  # noqa: E731  LIO drifting, drone still
    out = _run_mux(mux, 0.0, 20.0, slide, lambda t: truth)
    err = np.linalg.norm(out[80:, :2] - truth[:2], axis=1)     # after the first pull-in (3.2 m at 1 m/s)
    assert err.max() < 0.08, err.max()                # ~ slide rate x tau


def test_mux_matches_map_poses_at_their_scan_stamp():
    moving = lambda t: np.array([1.0 * t, 0.0, 0.0, 0.0])  # noqa: E731  1 m/s, odometry exact
    truth = moving
    # map poses arrive 0.2 s late but carry their stamp: no lag error
    mux = PoseMux()
    mux.request('map')
    out = _run_mux(mux, 0.0, 6.0, moving, truth, delay=0.2)
    assert abs(out[-1, 0] - 5.9) < 0.01
    # ...and the anchor never leaves where it started (odometry was right)
    assert np.allclose(mux.T, 0.0, atol=1e-6)


def test_mux_flies_on_the_map_when_odometry_dies_and_absorbs_its_jump():
    mux = PoseMux()
    mux.request('map')
    truth = lambda t: np.array([2.0 + 0.1 * t, 1.0, 0.7, 0.0])  # noqa: E731  moving 0.1 m/s
    out = _run_mux(mux, 0.0, 5.0, truth, truth)
    # LIO goes silent for 4 s (map poses keep coming): the output follows the map
    t, prev = 5.0, mux.pose().copy()
    steps = []
    while t < 9.0:
        alone = mux.on_map(t, truth(t), True, stamp=t)
        assert alone == (t - 4.9 > mux.p.odom_timeout + 1e-9)
        steps.append(np.linalg.norm(mux.pose()[:2] - prev[:2]))
        prev = mux.pose().copy()
        t += 0.1
    assert mux.mode == 'map-only'
    assert abs(mux.pose()[0] - truth(t - 0.1)[0]) < 0.03
    assert max(steps) < 0.05
    # it comes back 2.8 m off (FAST-LIO after "No Effective Points"): no jump
    before = mux.pose().copy()
    mux.on_odom(t, truth(t) + [2.8, 0.0, 0.0, 0.0])
    assert np.linalg.norm(mux.pose()[:2] - before[:2]) < 0.05
    assert any('re-anchored' in m for _, m in mux.events)
    # and a jump in mid-flight is absorbed too
    t += 0.1
    mux.on_odom(t, truth(t) + [2.8, 0.0, 0.0, 0.0])
    before = mux.pose().copy()
    t += 0.1
    mux.on_odom(t, truth(t) + [4.3, 0.0, 0.0, 0.0])
    assert np.linalg.norm(mux.pose()[:2] - before[:2]) < 0.05
    assert any('jumped' in m for _, m in mux.events)


def test_mux_anchor_tracks_map_slowly_on_odometry():
    mux = PoseMux()
    mux.track = True
    mux._ok_since = -1.0
    still = lambda t: np.zeros(4)  # noqa: E731
    out = _run_mux(mux, 0.0, 12.0, still, lambda t: np.array([1.0, 0.0, 0.0, 0.0]))
    steps = np.abs(np.diff(out[:, 0]))
    assert steps.max() <= 0.025 + 1e-9                # 0.25 m/s cap
    assert abs(out[-1, 0] - 1.0) < 0.05
    assert mux.mode == 'odom'


def test_pose_algebra():
    a = np.array([1.0, 2.0, 0.5, 0.7])
    b = np.array([-0.3, 0.4, 0.1, -1.2])
    assert np.allclose(compose(inverse(a), compose(a, b)), b)
