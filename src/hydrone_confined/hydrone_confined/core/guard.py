"""Last line before the flight controller: don't fly into what the beams see.

Takes a desired horizontal velocity and the sector ranges around the drone.
Toward any sector closer than `slow`, the velocity component into it is capped
to what can still be stopped in the space left (k * (r - stop)); closer than
`stop`, it is pushed back out. Motion along a wall is left alone, so the
drone slides through doorways instead of stalling in front of them.

Works on beams alone, no map, no pose: it's the reactive layer under
whatever planner sits on top.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class GuardParams:
    stop: float = 0.30          # m sensor-to-wall where approach speed hits zero
    slow: float = 0.70          # m where capping starts
    k: float = 0.8              # 1/s: allowed approach speed per metre left
    push: float = 0.12          # m/s back out at contact


def guard(v_xy, yaw, sectors, p=None):
    """(filtered world velocity (2,), closest range, its world direction).

    v_xy: world-frame velocity; yaw: body heading in the same frame;
    sectors: beams.Beams tiling the circle (body frame).
    """
    p = p or GuardParams()
    v = np.array(v_xy, dtype=float)[:2]
    ok = np.isfinite(sectors.ranges)
    if not ok.any():
        return v, np.inf, None
    ang = yaw + sectors.angles[ok]
    rng = sectors.ranges[ok]
    near = rng < p.slow
    for a, r in zip(ang[near], rng[near]):
        u = np.array([np.cos(a), np.sin(a)])
        into = float(v @ u)
        allowed = max(0.0, p.k * (r - p.stop))
        if into > allowed:
            v -= u * (into - allowed)
        if r < p.stop:
            v -= u * p.push * (p.stop - r) / p.stop
    k = int(np.argmin(rng))
    return v, float(rng[k]), np.array([np.cos(ang[k]), np.sin(ang[k])])
