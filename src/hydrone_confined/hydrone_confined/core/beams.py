"""Virtual range beams from a 3D lidar scan.

The 2D robot with N rangefinders around it, out of a 360 deg lidar: level the
scan with the flight controller's roll and pitch, keep the returns in a thin
height band around the sensor, and for each beam angle take the nearest
return within a fixed angular margin of it. A non-repetitive scan (Mid-360)
never fires exactly along a given angle, so the margin is what makes a
"beam" out of it; it also bounds the angle error of every range.

Also here, because they come from the same levelled scan:
  - vertical clearances (distance to the ceiling above / floor below)
  - the dominant wall direction of a rectilinear place (yaw modulo 90 deg)

Everything is in the body frame with yaw left in: x forward, y left, z up.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class BeamParams:
    n: int = 24                     # beams, evenly spaced CCW from body +x
    half_width: float = np.deg2rad(3.0)   # returns within this of a beam's angle
    band: tuple = (-0.12, 0.15)     # m, relative to the sensor, after levelling
    min_range: float = 0.12         # closer is the airframe / prop guards
    max_range: float = 12.0
    kth: int = 2                    # k-th nearest return, so one stray point isn't a wall


@dataclass
class Beams:
    angles: np.ndarray              # (n,) body frame, rad
    ranges: np.ndarray              # (n,) m, inf = nothing in the band
    counts: np.ndarray              # (n,) returns that fell in each beam

    def valid(self):
        return np.isfinite(self.ranges)


def rot_rp(roll, pitch):
    """Rotation that levels a body-frame vector (yaw kept)."""
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return ry @ rx


def level(points, roll=0.0, pitch=0.0):
    """Body-frame points (N,3), sensor at the origin -> gravity-levelled."""
    p = np.asarray(points, dtype=float)
    if roll == 0.0 and pitch == 0.0:
        return p
    return p @ rot_rp(roll, pitch).T


def beams(points, p=None, angles=None):
    """Beams from levelled points (sensor at the origin)."""
    p = p or BeamParams()
    if angles is None:
        angles = np.arange(p.n) * (2 * np.pi / p.n)
    angles = np.asarray(angles, dtype=float)
    pts = np.asarray(points, dtype=float)
    ranges = np.full(len(angles), np.inf)
    counts = np.zeros(len(angles), dtype=int)
    if not len(pts):
        return Beams(angles, ranges, counts)
    m = (pts[:, 2] >= p.band[0]) & (pts[:, 2] <= p.band[1])
    xy = pts[m, :2]
    r = np.hypot(xy[:, 0], xy[:, 1])
    ok = (r >= p.min_range) & (r <= p.max_range)
    xy, r = xy[ok], r[ok]
    if not len(r):
        return Beams(angles, ranges, counts)
    az = np.arctan2(xy[:, 1], xy[:, 0])
    # angular distance to every beam, wrapped
    d = np.abs((az[:, None] - angles[None, :] + np.pi) % (2 * np.pi) - np.pi)
    inside = d <= p.half_width
    counts = inside.sum(axis=0)
    rr = np.where(inside, r[:, None], np.inf)
    k = min(p.kth, len(r)) - 1
    part = np.partition(rr, k, axis=0)[k]
    # a lone return is still better than nothing when that's all there is
    lone = (counts > 0) & (counts <= k)
    part[lone] = rr[:, lone].min(axis=0)
    return Beams(angles, part, counts)


def sector_min(points, n=16, band=(-0.12, 0.15), min_range=0.12, max_range=12.0):
    """Nearest return in each of n sectors that tile the full circle (for guarding)."""
    return beams(points, BeamParams(n=n, half_width=np.pi / n, band=band, min_range=min_range,
                                    max_range=max_range, kth=2))


def vertical_clearance(points, cone=np.deg2rad(20.0), radius=1.5, fov_top=np.deg2rad(52.0)):
    """(up, down): distance to the ceiling above / floor below the sensor, or None.

    Indoors the steep upward rays hit the walls of a small room before the
    ceiling, so a median would sit on the walls. The ceiling is the top
    surface, nothing is above it: take the returns within 6 cm of the top
    (98th percentile) and average those. Same, mirrored, for the floor (a
    Mid-360 sees little below -7 deg, so that's mostly None indoors).
    """
    pts = np.asarray(points, dtype=float)
    if not len(pts):
        return None, None
    h = np.hypot(pts[:, 0], pts[:, 1])
    el = np.arctan2(pts[:, 2], h)
    up, down = None, None
    m = (el > cone) & (h < radius)
    zu, eu = pts[m, 2], el[m]
    if len(zu) >= 10:
        top = np.percentile(zu, 98)
        near = zu > top - 0.06
        if (eu[near] < fov_top - np.deg2rad(6.0)).mean() >= 0.3:
            up = float(zu[near].mean())
    zd = pts[(pts[:, 2] < -0.05) & (h < radius), 2]
    if len(zd) >= 10:
        bot = np.percentile(zd, 2)
        down = float(-zd[zd < bot + 0.06].mean())
    return up, down


def wall_yaw(points, band=(-0.3, 0.3), max_range=4.0, bin_=0.03, coarse_deg=1.0):
    """Dominant wall direction of a rectilinear place, modulo 90 deg.

    Returns (theta in [0, pi/2), score): theta is the angle of a wall axis in
    the body frame. Found by rotating the band's returns and keeping the angle
    whose projections on both axes pile up in the fewest bins.
    """
    pts = np.asarray(points, dtype=float)
    if not len(pts):
        return None, 0.0
    m = (pts[:, 2] >= band[0]) & (pts[:, 2] <= band[1])
    xy = pts[m, :2]
    xy = xy[np.hypot(xy[:, 0], xy[:, 1]) <= max_range]
    if len(xy) < 30:
        return None, 0.0

    def sharp(th):
        c, s = np.cos(th), np.sin(th)
        u = np.floor((xy[:, 0] * c + xy[:, 1] * s) / bin_).astype(np.int64)
        v = np.floor((-xy[:, 0] * s + xy[:, 1] * c) / bin_).astype(np.int64)
        return float((np.bincount(u - u.min()) ** 2).sum() + (np.bincount(v - v.min()) ** 2).sum())

    ths = np.deg2rad(np.arange(0.0, 90.0, coarse_deg))
    sc = np.array([sharp(t) for t in ths])
    best = ths[int(np.argmax(sc))]
    fine = best + np.deg2rad(np.arange(-coarse_deg, coarse_deg + 1e-9, 0.1))
    sf = np.array([sharp(t) for t in fine])
    th = float(fine[int(np.argmax(sf))]) % (np.pi / 2)
    # how much the best beats a typical angle: 1 = no structure at all
    score = float(sf.max() / max(np.median(sc), 1.0))
    return th, score
