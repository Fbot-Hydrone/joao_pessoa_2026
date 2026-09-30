"""Where am I in a known map, from a handful of range beams.

A small particle filter (x, y, yaw):
  predict  displacement in the body frame + noise (from whatever odometry is
           around: LIO deltas, commanded velocity; it only has to be roughly right)
  update   each beam's endpoint, seen from each particle, should land on a wall:
           a likelihood field (distance to the nearest wall face at the beam's
           height band) scores it. Beams that see nothing are skipped.
           In a rectilinear place the wall direction (yaw mod 90 deg) is scored too.
  z        from the ceiling (indoors) or the floor, against the map's surfaces.

No LIO in here: beams + attitude + the map are enough, which is the point.
"""
from dataclasses import dataclass

import numpy as np

from hydrone_confined.core.beams import Beams


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def wrap90(a):
    return (a + np.pi / 4) % (np.pi / 2) - np.pi / 4


def distance_field(occ):
    """Exact distance (cells) from every cell to the nearest True cell.

    Separable: nearest occupied along each column first, then a min over rows.
    O(nx^2 ny) but vectorised, a few ms for a 200 x 200 grid.
    """
    nx, ny = occ.shape
    big = float(nx + ny)
    if not occ.any():
        return np.full(occ.shape, big)
    j = np.arange(ny, dtype=float)
    last = np.where(occ, j[None, :], -np.inf)
    last = np.maximum.accumulate(last, axis=1)
    nxt = np.where(occ, j[None, :], np.inf)
    nxt = np.minimum.accumulate(nxt[:, ::-1], axis=1)[:, ::-1]
    g = np.minimum(j[None, :] - last, nxt - j[None, :])       # (nx, ny) along y
    g = np.minimum(g, big)
    i = np.arange(nx, dtype=float)
    di2 = (i[:, None] - i[None, :]) ** 2                        # (nx, nx)
    d2 = np.empty(occ.shape)
    for c in range(ny):
        d2[:, c] = (di2 + (g[:, c] ** 2)[None, :]).min(axis=1)
    return np.sqrt(d2)


@dataclass
class LocalizerParams:
    n_particles: int = 250
    sigma_hit: float = 0.07         # m, endpoint to wall
    p_rand: float = 0.1             # beams that hit something the map doesn't have
    max_beam: float = 6.0           # longer beams are too far to trust the angle
    motion_xy: float = 0.02         # m of noise per step, plus...
    motion_frac: float = 0.25       # ...this fraction of the step
    motion_yaw: float = np.deg2rad(1.0)
    yaw_sigma: float = np.deg2rad(2.0)   # wall-direction measurement
    yaw_min_score: float = 1.6
    resample_ess: float = 0.5
    rough_xy: float = 0.01
    rough_yaw: float = np.deg2rad(0.3)
    res: float = 0.05               # likelihood-field cell
    band_step: float = 0.05         # bands are cached on this grid
    lost_consistency: float = 0.6   # fewer of the judged beams than this agreeing = lost
    min_judged: int = 5             # beams the map expects a wall for, to judge at all
    seed: int = 0


class BeamLocalizer:

    def __init__(self, known_map, params=None):
        self.map = known_map
        self.p = params or LocalizerParams()
        self.rng = np.random.default_rng(self.p.seed)
        self.xy = np.zeros((0, 2))
        self.yaw = np.zeros(0)
        self.w = np.zeros(0)
        self.z = None
        self._fields = {}
        # the map's own wall direction, mod 90: 0 for an axis-aligned maze
        ang = [np.arctan2(w.tangent[1], w.tangent[0]) for w in known_map.walls]
        self.map_axis = float(np.angle(np.mean(np.exp(4j * np.array(ang)))) / 4) if ang else 0.0
        self.last_consistency = 1.0
        self.last_residual = 0.0
        self.last_judged = 0

    # ── setup ───────────────────────────────────────────────────────────────
    @property
    def ready(self):
        return len(self.w) > 0

    def init_gaussian(self, xy, yaw, sigma_xy=0.2, sigma_yaw=np.deg2rad(15.0), z=None):
        n = self.p.n_particles
        self.xy = np.asarray(xy, dtype=float)[None, :] + self.rng.normal(0, sigma_xy, (n, 2))
        self.yaw = wrap(yaw + self.rng.normal(0, sigma_yaw, n))
        self.w = np.full(n, 1.0 / n)
        self.z = z

    def init_rooms(self, rooms=None, yaw=None, sigma_yaw=np.deg2rad(20.0)):
        """Spread over rooms (all of them by default): relocalising from scratch."""
        rooms = rooms or self.map.rooms
        n = self.p.n_particles
        area = np.array([(r.x[1] - r.x[0]) * (r.y[1] - r.y[0]) for r in rooms])
        k = self.rng.choice(len(rooms), n, p=area / area.sum())
        lo = np.array([[rooms[i].x[0], rooms[i].y[0]] for i in k])
        hi = np.array([[rooms[i].x[1], rooms[i].y[1]] for i in k])
        self.xy = lo + self.rng.random((n, 2)) * (hi - lo)
        self.yaw = (self.rng.uniform(-np.pi, np.pi, n) if yaw is None
                    else wrap(yaw + self.rng.normal(0, sigma_yaw, n)))
        self.w = np.full(n, 1.0 / n)

    # ── map lookups ─────────────────────────────────────────────────────────
    def _field(self, band):
        q = self.p.band_step
        key = (round(band[0] / q) * q, round(band[1] / q) * q)
        f = self._fields.get(key)
        if f is None:
            occ, origin, res = self.map.rasterize(key, res=self.p.res, margin=1.0)
            f = (distance_field(occ) * res, occ, origin, res)
            if len(self._fields) > 32:
                self._fields.clear()
            self._fields[key] = f
        return f

    @staticmethod
    def _lookup(grid, origin, res, xy, fill):
        ij = np.floor((xy - origin) / res).astype(int)
        ok = (ij[..., 0] >= 0) & (ij[..., 0] < grid.shape[0]) & (ij[..., 1] >= 0) & (ij[..., 1] < grid.shape[1])
        out = np.full(xy.shape[:-1], fill, dtype=grid.dtype)
        out[ok] = grid[ij[ok, 0], ij[ok, 1]]
        return out

    # ── filter ──────────────────────────────────────────────────────────────
    def predict(self, d_body, d_yaw=0.0, scale=1.0):
        """Move every particle by a body-frame displacement (2,) and a yaw change."""
        if not self.ready:
            return
        d = np.asarray(d_body, dtype=float)[:2]
        n = len(self.w)
        step = float(np.hypot(*d))
        s = scale * (self.p.motion_xy + self.p.motion_frac * step)
        c, si = np.cos(self.yaw), np.sin(self.yaw)
        self.xy[:, 0] += c * d[0] - si * d[1]
        self.xy[:, 1] += si * d[0] + c * d[1]
        self.xy += self.rng.normal(0, s, (n, 2))
        self.yaw = wrap(self.yaw + d_yaw + self.rng.normal(0, scale * self.p.motion_yaw, n))

    def update(self, bm: Beams, band, yaw_meas=None, mount=(0.0, 0.0)):
        """Weigh particles by the beams. band = absolute [z0, z1] of the beams.

        mount: sensor xy offset from base_link, in the body frame.
        yaw_meas: (theta, score) from beams.wall_yaw, body-frame wall axis mod 90.
        """
        if not self.ready:
            return
        dist, occ, origin, res = self._field(band)
        ok = np.isfinite(bm.ranges) & (bm.ranges <= self.p.max_beam)
        logw = np.log(np.maximum(self.w, 1e-300))
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        sensor = self.xy + np.stack([c * mount[0] - s * mount[1], s * mount[0] + c * mount[1]], 1)
        if ok.any():
            a = self.yaw[:, None] + bm.angles[None, ok]
            r = bm.ranges[None, ok]
            end = sensor[:, None, :] + np.stack([np.cos(a) * r, np.sin(a) * r], axis=-1)
            d = self._lookup(dist, origin, res, end, fill=1.0)
            p = (1 - self.p.p_rand) * np.exp(-0.5 * (d / self.p.sigma_hit) ** 2) + self.p.p_rand
            logw += np.log(p).sum(axis=1)
        if yaw_meas is not None and yaw_meas[0] is not None and yaw_meas[1] >= self.p.yaw_min_score:
            e = wrap90(yaw_meas[0] - (self.map_axis - self.yaw))
            logw += -0.5 * (e / self.p.yaw_sigma) ** 2
        # a particle inside a wall is impossible
        inwall = self._lookup(occ, origin, res, self.xy, fill=False)
        logw[inwall] = -np.inf
        if not np.isfinite(logw).any():
            self.w = np.full(len(self.w), 1.0 / len(self.w))
            return
        logw -= logw[np.isfinite(logw)].max()
        w = np.exp(logw)
        self.w = w / w.sum()
        if self.ess() < self.p.resample_ess * len(self.w):
            self._resample()
        self._health(bm, band, mount)

    def _resample(self):
        n = len(self.w)
        u = (self.rng.random() + np.arange(n)) / n
        idx = np.minimum(np.searchsorted(np.cumsum(self.w), u), n - 1)
        self.xy = self.xy[idx] + self.rng.normal(0, self.p.rough_xy, (n, 2))
        self.yaw = wrap(self.yaw[idx] + self.rng.normal(0, self.p.rough_yaw, n))
        self.w = np.full(n, 1.0 / n)

    def ess(self):
        return float(1.0 / np.sum(self.w ** 2)) if self.ready else 0.0

    # ── output ──────────────────────────────────────────────────────────────
    def estimate(self):
        """(xy (2,), yaw, cov (3,3)) weighted mean, or None before init."""
        if not self.ready:
            return None
        xy = self.w @ self.xy
        yaw = float(np.angle(np.sum(self.w * np.exp(1j * self.yaw))))
        dx = self.xy - xy
        dy = wrap(self.yaw - yaw)
        v = np.column_stack([dx, dy])
        cov = (v * self.w[:, None]).T @ v
        return xy, yaw, cov

    def _health(self, bm, band, mount):
        """Beams from the mean pose vs a ray cast into the map.

        Only beams the map says should hit a wall are judged: a return from
        something the map doesn't have (event walls, people) says nothing
        about the pose, and neither does a dropout.
        """
        est = self.estimate()
        xy, yaw = est[0], est[1]
        c, s = np.cos(yaw), np.sin(yaw)
        sensor = xy + np.array([c * mount[0] - s * mount[1], s * mount[0] + c * mount[1]])
        exp = self.map.raycast(sensor[None, :], (yaw + bm.angles)[None, :], band,
                               max_range=self.p.max_beam)[0]
        judged = np.isfinite(exp) & np.isfinite(bm.ranges)
        self.last_judged = int(judged.sum())
        if self.last_judged < self.p.min_judged:
            self.last_consistency = 0.0
            self.last_residual = np.inf
            return
        res = np.abs(bm.ranges[judged] - exp[judged])
        self.last_consistency = float((res < 0.15).mean())
        self.last_residual = float(np.median(res))

    @property
    def lost(self):
        return self.last_consistency < self.p.lost_consistency

    def z_from(self, xy, up=None, down=None, sensor_up=0.1):
        """base_link z from the ceiling distance (indoors) or floor distance."""
        ceil = self.map.ceiling_at(xy)
        if ceil is not None and up is not None:
            return ceil - up - sensor_up
        if down is not None:
            return self.map.support_at(xy) + down - sensor_up
        return None
