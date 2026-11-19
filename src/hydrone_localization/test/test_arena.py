"""arena: zones in the arena frame, tied to the FCU frame at arming.

    python3 -m pytest src/hydrone_localization/test/test_arena.py -q
"""

import math

import pytest

from hydrone_localization.arena import (Arena, camera_vouches,
                                        rangefinder_in_image,
                                        sensor_ground_point)


def cbr():
    # CBR 2026 Fig. 3: 2 m takeoff strip + 6 m, 1.5 m... use an 8 x 4 arena
    a = Arena(size_xy=(8.0, 4.0), takeoff_base=(1.0, 2.0, 0.0),
              zone_edges_x=[0.0, 2.0, 8.0], zone_raised=[True, False])
    a.validate()
    return a


def test_validation():
    with pytest.raises(ValueError):
        Arena(size_xy=(8, 4), zone_edges_x=[0, 2, 8],
              zone_raised=[True]).validate()
    with pytest.raises(ValueError):
        Arena(size_xy=(8, 4), zone_edges_x=[0.5, 2, 8],
              zone_raised=[True, True]).validate()      # gap at the wall
    with pytest.raises(ValueError):
        Arena(size_xy=(8, 4), zone_edges_x=[0, 3, 2, 8],
              zone_raised=[True, True, True]).validate()


def test_latch_round_trip_with_any_fcu_yaw():
    a = cbr()
    a.latch(5.0, -3.0, math.radians(91.0))   # FCU heading 91 deg (2026-11-17)
    # The vehicle is on the base centre
    ax, ay = a.to_arena(5.0, -3.0)
    assert abs(ax - 1.0) < 1e-9 and abs(ay - 2.0) < 1e-9
    # One metre FORWARD of the vehicle is +x in the arena (heading 0)
    fx = 5.0 + math.cos(math.radians(91.0))
    fy = -3.0 + math.sin(math.radians(91.0))
    ax, ay = a.to_arena(fx, fy)
    assert abs(ax - 2.0) < 1e-9 and abs(ay - 2.0) < 1e-9
    for p in ((0.0, 0.0), (8.0, 4.0), (3.3, 1.1)):
        q = a.to_arena(*a.to_fcu(*p))
        assert abs(q[0] - p[0]) < 1e-9 and abs(q[1] - p[1]) < 1e-9


def test_base_placed_facing_another_way():
    a = Arena(size_xy=(8, 4), takeoff_base=(1.0, 2.0, 90.0),
              zone_edges_x=[0, 2, 8], zone_raised=[True, False])
    a.latch(0.0, 0.0, 0.0)
    # vehicle faces arena +y; 1 m forward in FCU (+x) is arena (1, 3)
    ax, ay = a.to_arena(1.0, 0.0)
    assert abs(ax - 1.0) < 1e-9 and abs(ay - 3.0) < 1e-9


def test_zones_and_margins():
    a = cbr()
    a.latch(0.0, 0.0, 0.0)
    assert a.raised_at(1.0, 2.0)            # takeoff strip
    assert not a.raised_at(5.0, 2.0)        # landing strip, camera needed
    assert a.raised_at(2.1, 2.0)            # inside the boundary margin
    assert not a.raised_at(2.3, 2.0)
    assert a.zones_at(5.0, 9.0) == []       # far outside the arena


def test_sensor_ground_point_follows_yaw():
    x, y = sensor_ground_point(0.0, 0.0, math.radians(90), (0.14, 0.10))
    assert abs(x + 0.10) < 1e-9 and abs(y - 0.14) < 1e-9


def test_rangefinder_spot_in_the_belly_image():
    # big drone: rangefinder (0.14, 0.10), camera (0.065, 0.071)
    u, v = rangefinder_in_image((0.14, 0.10), (0.065, 0.071), 1.0,
                                fx=814.6, cx=320, cy=240)
    # spot is 7.5 cm AHEAD (up in the image) and 2.9 cm LEFT (left)
    assert abs(v - (240 - 0.075 * 814.6)) < 1e-6
    assert abs(u - (320 - 0.029 * 814.6)) < 1e-6
    assert camera_vouches([(u + 30, v, 40.0)], (u, v))
    assert not camera_vouches([(u + 200, v, 40.0)], (u, v))


def test_no_zones_means_everything_may_be_raised():
    a = Arena()
    a.validate()
    assert a.raised_at(123.0, -4.0)
