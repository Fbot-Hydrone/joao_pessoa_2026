"""Arena rectangle from low-band wall hits; its centre is the landing point.

The arena side walls are short (~0.5 m) and everything else (structure, pads)
is inside them, so the minimum-area rectangle around the low-band hits is the
arena. Its orientation comes from a trimmed minimum-area rectangle (rotating
calipers); each side then sits on the outermost line with enough hits on it,
pushed out over anything known to be inside (the structure's footprint).
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class Rect:
    center: np.ndarray      # (2,)
    size: np.ndarray        # (2,) along axis u, along axis v
    yaw: float              # angle of axis u
    area: float

    @property
    def axes(self):
        u = np.array([np.cos(self.yaw), np.sin(self.yaw)])
        return u, np.array([-u[1], u[0]])

    def corners(self):
        u, v = self.axes
        hu, hv = self.size / 2
        return np.array([self.center + su * hu * u + sv * hv * v
                         for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1))])

    def contains(self, xy, margin=0.0):
        u, v = self.axes
        d = np.atleast_2d(xy) - self.center
        return (np.abs(d @ u) <= self.size[0] / 2 + margin) & (np.abs(d @ v) <= self.size[1] / 2 + margin)


def convex_hull(pts):
    """Monotone chain; returns hull vertices counter-clockwise."""
    pts = np.unique(np.asarray(pts, dtype=float), axis=0)
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def min_area_rect(pts):
    """Rotating calipers over the hull edges. None for fewer than 3 points."""
    hull = convex_hull(pts)
    if len(hull) < 3:
        return None
    edges = np.roll(hull, -1, axis=0) - hull
    angles = np.unique(np.mod(np.arctan2(edges[:, 1], edges[:, 0]), np.pi / 2))
    best = None
    for a in angles:
        u = np.array([np.cos(a), np.sin(a)])
        v = np.array([-u[1], u[0]])
        pu, pv = hull @ u, hull @ v
        area = (pu.max() - pu.min()) * (pv.max() - pv.min())
        if best is None or area < best.area:
            cu, cv = (pu.max() + pu.min()) / 2, (pv.max() + pv.min()) / 2
            best = Rect(center=cu * u + cv * v, size=np.array([pu.max() - pu.min(), pv.max() - pv.min()]),
                        yaw=float(a), area=float(area))
    return best


def robust_rect(pts, trim=1.0, iters=2):
    """min_area_rect with percentile trimming along the fitted axes."""
    pts = np.asarray(pts, dtype=float)
    rect = min_area_rect(pts)
    for _ in range(iters):
        if rect is None:
            return None
        u, v = rect.axes
        pu, pv = pts @ u, pts @ v
        keep = ((pu >= np.percentile(pu, trim)) & (pu <= np.percentile(pu, 100 - trim))
                & (pv >= np.percentile(pv, trim)) & (pv <= np.percentile(pv, 100 - trim)))
        rect = min_area_rect(pts[keep])
    return rect


def supported_rect(pts, inside=None, min_support=8, slab=0.2, trim=1.0):
    """Rectangle whose sides sit on the outermost lines with real support.

    pts      wall hits (xy)
    inside   points known to be inside the arena (the structure footprint, the
             takeoff point); trusted, never trimmed

    Percentile trimming alone eats a wall that was only glimpsed, and the wall
    behind the structure isn't seen at all. So each side is the outermost slab
    holding min_support hits, pushed out to cover `inside`; with no such slab
    on a side, the trimmed extreme of the hits.
    """
    pts = np.asarray(pts, dtype=float)
    inside = np.zeros((0, 2)) if inside is None else np.asarray(inside, dtype=float).reshape(-1, 2)
    rect = robust_rect(np.vstack([pts, inside]), trim=trim, iters=1)
    if rect is None:
        return None
    u, v = rect.axes
    lo_hi = []
    for a in (u, v):
        proj = np.sort(pts @ a)
        ahead = np.searchsorted(proj, proj + slab, side='right') - np.arange(len(proj))
        behind = np.arange(len(proj)) + 1 - np.searchsorted(proj, proj - slab, side='left')
        ok_lo = np.nonzero(ahead >= min_support)[0]
        ok_hi = np.nonzero(behind >= min_support)[0]
        lo = proj[ok_lo[0]] if len(ok_lo) else np.percentile(proj, trim)
        hi = proj[ok_hi[-1]] if len(ok_hi) else np.percentile(proj, 100 - trim)
        if len(inside):
            pi = inside @ a
            lo, hi = min(lo, pi.min()), max(hi, pi.max())
        lo_hi.append((lo, hi))
    (u0, u1), (v0, v1) = lo_hi
    if u1 <= u0 or v1 <= v0:
        return None
    center = (u0 + u1) / 2 * u + (v0 + v1) / 2 * v
    return Rect(center=center, size=np.array([u1 - u0, v1 - v0]), yaw=rect.yaw,
                area=float((u1 - u0) * (v1 - v0)))


def arena_from_grid(grid, inside=None, min_hits=2, min_cells=40, min_side=3.0, min_support=8):
    """Arena Rect from the grid's low layer (+ points known inside), or None."""
    ij = np.argwhere(grid.low >= min_hits)
    if len(ij) < min_cells:
        return None
    rect = supported_rect(grid.cell_center(ij), inside, min_support=min_support)
    if rect is None or rect.size.min() < min_side:
        return None
    return rect
