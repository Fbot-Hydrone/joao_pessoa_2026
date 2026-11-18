"""The belly camera's offset is written twice and the two must agree.

sources_real.launch.py   down_cam_mount_xyz  ROS FLU   [x fwd, y LEFT,  z]
phase1_real / phase1_dry belly_offset_xy     landing   [x fwd, y RIGHT]

The first places the camera in TF (pad positions in the map); the second moves
the pixel the visual descent centres the pad on, so the VEHICLE — not the
camera — ends up over the base. Measured 2026-11-18 on the big drone: 6.5 cm
forward, 7.1 cm left of the motor centre.

    python3 -m pytest src/hydrone_bringup/test/test_belly_offset.py -q
"""

import ast
import os
import re

from hydrone_nav import precision_landing as pl

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.join(HERE, "..", "launch")


def _read(name):
    with open(os.path.join(LAUNCH, name)) as f:
        return f.read()


def _mount_xyz():
    src = _read("sources_real.launch.py")
    m = re.search(r'"down_cam_mount_xyz",\s*default_value="(\[[^"]+\])"', src)
    assert m, "down_cam_mount_xyz default not found in sources_real"
    return ast.literal_eval(m.group(1))


def _belly(name):
    src = _read(name)
    m = (re.search(r'^BELLY_OFFSET_XY = "(\[[^"]+\])"', src, re.M)
         or re.search(r'"belly_offset_xy":\s*"(\[[^"]+\])"', src))
    assert m, f"belly_offset_xy not forwarded by {name}"
    return ast.literal_eval(m.group(1))


def test_real_wrappers_mirror_the_mount():
    mx, my, _ = _mount_xyz()
    for name in ("phase1_real.launch.py", "phase1_dry.launch.py"):
        bx, by = _belly(name)
        assert abs(bx - mx) < 1e-9, name
        assert abs(by + my) < 1e-9, f"{name}: y must be the mount's y negated"


def test_the_measured_mount():
    mx, my, _ = _mount_xyz()
    assert (mx, my) == (0.065, 0.071)


def test_offset_moves_the_target_the_right_way():
    """Camera forward-left of centre: with the CENTRE over the pad, the pad
    is behind the camera (+v) and to its right (+u)."""
    bx, by = _belly("phase1_real.launch.py")
    u, v = pl.target_uv((320.0, 240.0), (bx, by), fx=814.6, height=1.0)
    assert u > 320.0 and v > 240.0
    # 7.1 cm and 6.5 cm at 1 m with fx 814.6
    assert abs((u - 320.0) - 0.071 * 814.6) < 1e-6
    assert abs((v - 240.0) - 0.065 * 814.6) < 1e-6
