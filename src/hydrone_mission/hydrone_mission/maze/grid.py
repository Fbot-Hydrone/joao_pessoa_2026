"""2D log-odds occupancy at the flight band, plus per-cell roof and low-band layers.

One `update(points, origin)` per registered scan:
  - occupied: hits with z inside flight_z +- band
  - free: only the part of each ray that lies inside that band (unknown != free)
  - roof: hits in [flight_z + roof_lo, flight_z + roof_hi], low enough to miss
    arena nets; wall tops and the roof of a confined space land here (openings.py
    drops the wall tops, which are occupied cells themselves)
  - low: hits in [floor + low_margin, flight_z - low_top] on vertical faces
    (a cell whose hits in one scan span some height): sills, arena walls, but
    not the top of a pad that happens to sit next to a wall
  - low_pass: rays crossing the low band without ending there (no sill under a
    door shows up as passes without hits)

The floor is the dominant low horizontal surface, from a running z histogram.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class GridParams:
    res: float = 0.1            # cell size, m
    half_size: float = 20.0     # grid spans +-half_size around the odom origin
    flight_z: float = 0.5       # centre of the occupancy band (odom z)
    band: float = 0.2           # +- around flight_z
    roof_lo: float = 0.3        # roof layer, relative to flight_z
    roof_hi: float = 1.0
    low_margin: float = 0.1     # low layer starts this far above the floor
    low_top: float = 0.45       # and ends this far under flight_z (below a sill top)
    low_vertical: float = 0.06  # low hits count only where one scan spans this much z
    floor_flat: float = 0.08    # a floor vote needs one scan's hits in a cell this level
    floor_z: float = None       # fixed floor; None = estimate it
    floor_guess: float = None   # used until the estimate settles; None = flight_z - 1.1
    voxel: float = 0.05         # endpoint downsampling before ray casting
    max_range: float = 6.0      # free space is carved only this far
    hit_range: float = 12.0     # hits further away only carve free space
    step: float = 0.05          # ray sampling step
    l_hit: float = 0.85
    l_miss: float = -0.4
    l_min: float = -2.0
    l_max: float = 3.5
    occ_thresh: float = 0.5
    free_thresh: float = -0.3


class OccupancyGrid:

    def __init__(self, params=None, **kw):
        self.p = params or GridParams(**kw)
        p = self.p
        self.n = int(round(2 * p.half_size / p.res))
        self.origin = -p.half_size           # x/y of cell (0, 0)'s corner
        shape = (self.n, self.n)
        self.logodds = np.zeros(shape, dtype=np.float32)
        self.roof = np.zeros(shape, dtype=np.uint16)
        self.low = np.zeros(shape, dtype=np.uint16)
        self.low_pass = np.zeros(shape, dtype=np.uint16)
        self.observed = np.zeros(shape, dtype=bool)
        # touched bounding box, for cropping
        self.bbox = None
        self._zhist = np.zeros(400, dtype=np.int64)   # 5 cm bins, -10..10 m
        self.floor_z = p.floor_z
        self.scans = 0

    # ── cells ───────────────────────────────────────────────────────────────
    def to_cell(self, xy):
        xy = np.asarray(xy, dtype=float)
        return np.floor((xy - self.origin) / self.p.res).astype(int)

    def cell_center(self, ij):
        return self.origin + (np.asarray(ij, dtype=float) + 0.5) * self.p.res

    def inside(self, ij):
        ij = np.asarray(ij)
        return (ij >= 0).all(axis=-1) & (ij < self.n).all(axis=-1)

    def occupied(self):
        return self.logodds > self.p.occ_thresh

    def free(self):
        return self.logodds < self.p.free_thresh

    def unknown(self):
        return ~self.occupied() & ~self.free()

    def floor(self):
        if self.floor_z is not None:
            return self.floor_z
        if self.p.floor_guess is not None:
            return self.p.floor_guess
        return self.p.flight_z - 1.1

    def crop(self, margin=1.0):
        """(i0, i1, j0, j1) slice bounds of the touched area plus a margin."""
        if self.bbox is None:
            c = self.n // 2
            return c - 1, c + 1, c - 1, c + 1
        m = int(np.ceil(margin / self.p.res))
        i0, i1, j0, j1 = self.bbox
        return max(0, i0 - m), min(self.n, i1 + m + 1), max(0, j0 - m), min(self.n, j1 + m + 1)

    # ── update ──────────────────────────────────────────────────────────────
    def update(self, points, sensor_origin):
        """Fuse one registered scan: (N,3) odom points, (3,) sensor origin."""
        p = self.p
        pts = np.asarray(points, dtype=np.float32)
        o = np.asarray(sensor_origin, dtype=np.float32)
        if len(pts) == 0:
            return
        d = pts - o
        rng = np.linalg.norm(d, axis=1)
        # far returns (floor seen through a window) still carve free space up to
        # max_range; squash them onto a sphere so they downsample together
        far = rng > p.hit_range
        if far.any():
            pts = pts.copy()
            pts[far] = o + d[far] * (p.hit_range / rng[far])[:, None]
            d = pts - o
            rng = np.minimum(rng, p.hit_range)
        # voxel downsample the endpoints (duplicate rays add nothing)
        keys = np.floor(pts / p.voxel).astype(np.int64)
        _, first = np.unique(keys[:, 0] * 73856093 ^ keys[:, 1] * 19349663 ^ keys[:, 2] * 83492791,
                             return_index=True)
        pts, d, rng, far = pts[first], d[first], rng[first], far[first]
        self.scans += 1
        self._update_floor(pts[~far])

        z = pts[:, 2]
        fz, floor = p.flight_z, self.floor()
        in_band = np.abs(z - fz) <= p.band
        in_roof = (z >= fz + p.roof_lo) & (z <= fz + p.roof_hi)
        lo_a, lo_b = floor + p.low_margin, fz - p.low_top
        low_ready = self.floor_z is not None or p.floor_guess is not None
        in_low = (z >= lo_a) & (z <= lo_b) & low_ready

        ij = self.to_cell(pts[:, :2])
        ok = self.inside(ij) & ~far
        flat = ij[:, 0] * self.n + ij[:, 1]

        hit_cells = np.unique(flat[in_band & ok])
        self._add_counts(self.roof, np.unique(flat[in_roof & ok]))
        if low_ready:
            self._add_counts(self.low, self._span_cells(flat[in_low & ok], z[in_low & ok],
                                                        lo=p.low_vertical))

        # free space: sample each ray only where it is inside the band
        free_cells = self._ray_cells(o, d, rng, fz - p.band, fz + p.band)
        free_cells = np.setdiff1d(free_cells, hit_cells, assume_unique=True)
        if low_ready:
            passes = self._ray_cells(o, d, rng, lo_a, lo_b)
            low_hit_cells = np.unique(flat[in_low & ok])
            self._add_counts(self.low_pass, np.setdiff1d(passes, low_hit_cells, assume_unique=True))

        lo = self.logodds.reshape(-1)
        lo[hit_cells] = np.minimum(lo[hit_cells] + p.l_hit, p.l_max)
        lo[free_cells] = np.maximum(lo[free_cells] + p.l_miss, p.l_min)
        touched = np.concatenate([hit_cells, free_cells])
        self.observed.reshape(-1)[touched] = True
        if len(touched):
            ii, jj = touched // self.n, touched % self.n
            box = (ii.min(), ii.max(), jj.min(), jj.max())
            if self.bbox is None:
                self.bbox = box
            else:
                b = self.bbox
                self.bbox = (min(b[0], box[0]), max(b[1], box[1]), min(b[2], box[2]), max(b[3], box[3]))

    def _span_cells(self, cells, z, lo=None, hi=None):
        """Cells whose hits in this scan span at least `lo` / at most `hi` in z."""
        if len(cells) == 0:
            return cells
        order = np.argsort(cells, kind='stable')
        cells, z = cells[order], z[order]
        uniq, start = np.unique(cells, return_index=True)
        span = np.maximum.reduceat(z, start) - np.minimum.reduceat(z, start)
        keep = np.ones(len(uniq), dtype=bool)
        if lo is not None:
            keep &= span >= lo
        if hi is not None:
            keep &= span <= hi
        return uniq[keep]

    def _add_counts(self, layer, cells):
        v = layer.reshape(-1)
        v[cells] = np.minimum(v[cells].astype(np.int32) + 1, 65535).astype(np.uint16)

    def _ray_cells(self, o, d, rng, za, zb):
        """Unique flat cells the rays cross while za <= z <= zb, endpoints excluded."""
        p = self.p
        dirs = d / np.maximum(rng, 1e-6)[:, None]
        # stop half a cell short of the hit, and at max_range
        t_end = np.minimum(rng - 0.5 * p.res, p.max_range)
        dz = dirs[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            ta = (za - o[2]) / dz
            tb = (zb - o[2]) / dz
        flat_ray = np.abs(dz) < 1e-6
        inside_now = (o[2] >= za) & (o[2] <= zb)
        t0 = np.where(flat_ray, 0.0, np.minimum(ta, tb))
        t1 = np.where(flat_ray, np.inf if inside_now else -np.inf, np.maximum(ta, tb))
        t0 = np.maximum(t0, 0.0)
        t1 = np.minimum(t1, t_end)
        keep = t1 > t0
        if not keep.any():
            return np.zeros(0, dtype=np.int64)
        t0, t1, dirs = t0[keep], t1[keep], dirs[keep]
        counts = np.ceil((t1 - t0) / p.step).astype(np.int64) + 1
        total = int(counts.sum())
        ray = np.repeat(np.arange(len(t0)), counts)
        start = np.repeat(np.cumsum(counts) - counts, counts)
        t = t0[ray] + (np.arange(total) - start) * p.step
        t = np.minimum(t, t1[ray])
        xy = o[:2] + dirs[ray, :2] * t[:, None]
        ij = np.floor((xy - self.origin) / p.res).astype(np.int64)
        ok = (ij >= 0).all(axis=1) & (ij < self.n).all(axis=1)
        return np.unique(ij[ok, 0] * self.n + ij[ok, 1])

    def _update_floor(self, pts):
        if self.p.floor_z is not None:
            return
        # one vote per cell that this scan saw as a horizontal surface. Voting
        # per point let a wall, which is a dense sheet of heights, outvote the
        # floor; inside a structure the floor is barely visible at all
        ij = self.to_cell(pts[:, :2])
        ok = self.inside(ij)
        flat, z = ij[ok, 0] * self.n + ij[ok, 1], pts[ok, 2]
        level = self._span_cells(flat, z, hi=self.p.floor_flat)
        if not len(level):
            return
        order = np.argsort(flat, kind='stable')
        uniq, start = np.unique(flat[order], return_index=True)
        low = np.minimum.reduceat(z[order], start)[np.isin(uniq, level)]
        b = np.floor((low + 10.0) / 0.05).astype(int)
        b = b[(b >= 0) & (b < len(self._zhist))]
        self._zhist += np.bincount(b, minlength=len(self._zhist))
        # only below the flight band; the floor is the lowest strong peak
        top = int((self.p.flight_z - self.p.low_top - 0.2 + 10.0) / 0.05)
        h = self._zhist[:max(top, 1)]
        if h.sum() < 60:
            return
        # a horizontal surface piles up in one or two bins; walls spread out
        h2 = h[:-1] + h[1:]
        peak = h2.max()
        strong = np.nonzero(h2 >= 0.3 * peak)[0]
        k = int(strong[0])
        # refine inside the pair of bins
        k = k if h[k] >= h[k + 1] else k + 1
        self.floor_z = None if peak < 25 else (k + 0.5) * 0.05 - 10.0
        self._floor_est = self.floor_z

    def estimated_floor(self):
        return getattr(self, '_floor_est', None)
