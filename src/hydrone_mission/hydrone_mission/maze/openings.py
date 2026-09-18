"""Covered region and openings in its boundary.

covered   cells with roof-band evidence, closed and hole-filled: the footprint of
          a confined space (walls tops + roof)
opening   observed-free cells on the boundary between covered free space and
          uncovered free space, with a passable width
kind      'window' when the low band under the gap has hits (a sill),
          'door' when rays pass through the low band there without hitting,
          'unknown' when it hasn't been seen low enough yet
"""
from dataclasses import dataclass, field
import math

import numpy as np

from hydrone_mission.maze.morph import close, dilate, dilate4, fill_holes, label


@dataclass
class OpeningParams:
    min_roof_hits: int = 2      # roof-layer hits for a cell to count
    close_radius: float = 0.5   # bridges gaps up to ~2x this in the roof outline
    min_width: float = 0.45     # drone diameter + margin
    max_width: float = 1.5
    flank_search: float = 1.0   # how far along the gap to look for its walls
    wall_near: float = 0.6      # a candidate cell must have wall within this
    probe: float = 0.45         # the gap must be clear this far in and out
    outside_depth: float = 1.2  # and lead to uncovered open space at least this deep
    sill_min_hits: int = 2
    door_min_passes: int = 12   # rays through the low band just past the gap


@dataclass
class Opening:
    center: np.ndarray          # (2,) odom xy
    width: float
    normal: np.ndarray          # (2,) unit, pointing into the covered region
    confidence: float
    kind: str                   # window / door / unknown
    sill_hits: int = 0
    low_passes: int = 0
    flanks: int = 0             # 0, 1 or 2 walls found beside the gap
    confirmed: bool = True      # open space seen beyond it; False = not seen yet
    cells: np.ndarray = field(default=None, repr=False)   # (k,2) full-grid cells

    @property
    def tangent(self):
        return np.array([-self.normal[1], self.normal[0]])

    def outside(self, dist):
        return self.center - self.normal * dist

    def inside(self, dist):
        return self.center + self.normal * dist


def covered_region(grid, crop, params=None):
    """Bool crop of the roofed footprint."""
    p = params or OpeningParams()
    i0, i1, j0, j1 = crop
    # a wall's top is not a roof: the arena walls reach into the roof band all
    # round, and filling that ring's holes would roof the whole arena. So only
    # roof hits away from walls are filled in — the wall tops are then added
    # back, so that a covered region still ends on its own wall
    hits = grid.roof[i0:i1, j0:j1] >= p.min_roof_hits
    occ = grid.occupied()[i0:i1, j0:j1]
    if not hits.any():
        return hits
    closed = close(hits & ~occ, p.close_radius / grid.p.res)
    return fill_holes(closed) | (hits & occ)


def _sample(mask, grid, crop, xy):
    """mask value at world points (False outside the crop)."""
    i0, i1, j0, j1 = crop
    ij = grid.to_cell(np.atleast_2d(xy)) - np.array([i0, j0])
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < i1 - i0) & (ij[:, 1] >= 0) & (ij[:, 1] < j1 - j0)
    out = np.zeros(len(ij), dtype=bool)
    out[ok] = mask[ij[ok, 0], ij[ok, 1]]
    return out


def _count(layer, grid, crop, xy):
    i0, i1, j0, j1 = crop
    ij = np.unique(grid.to_cell(np.atleast_2d(xy)), axis=0) - np.array([i0, j0])
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < i1 - i0) & (ij[:, 1] >= 0) & (ij[:, 1] < j1 - j0)
    return int(layer[ij[ok, 0], ij[ok, 1]].astype(np.int64).sum())


def detect_openings(grid, params=None, crop=None, covered=None, viewer=None):
    """All passable openings in the covered region's boundary.

    viewer: the drone's xy, used to read door evidence on the far side of a gap.

    Returns (openings, covered, crop). Openings whose far side hasn't been seen
    yet come back with confirmed=False.
    """
    p = params or OpeningParams()
    crop = crop or grid.crop(margin=1.5)
    i0, i1, j0, j1 = crop
    res = grid.p.res
    if covered is None:
        covered = covered_region(grid, crop, p)
    free = grid.free()[i0:i1, j0:j1]
    occ = grid.occupied()[i0:i1, j0:j1]
    if not covered.any():
        return [], covered, crop
    out_free = free & ~covered
    # next to uncovered space that isn't wall: free, or not seen yet (an exit
    # seen from deep inside); the checks below sort them out
    cand = free & covered & dilate4(~covered & ~occ)
    # an opening is a gap in a wall, so there must be wall near it. Without
    # this the ragged edge of the roofed region (which just follows what the
    # lidar has seen) joins onto the real gap and swamps its shape
    cand &= dilate(occ, p.wall_near / res)
    # a gap is 1-2 cells deep; take its neighbours too so the pieces join up
    cand = dilate(cand, 1.5) & free & covered
    lab, n = label(cand, conn=8)
    roof = grid.roof[i0:i1, j0:j1]
    low = grid.low[i0:i1, j0:j1]
    low_pass = grid.low_pass[i0:i1, j0:j1]
    found = []
    for k in range(1, n + 1):
        cells = np.argwhere(lab == k)
        if len(cells) < 2:
            continue
        xy = grid.origin + (cells + np.array([i0, j0]) + 0.5) * res
        c = xy.mean(axis=0)
        normal = _normal(cells, xy, out_free, covered, roof, grid, crop)
        if normal is None:
            continue
        normal = _snap_to_wall(occ, grid, crop, c, normal)
        t = np.array([-normal[1], normal[0]])
        center, width, flanks = _gap_width(free, occ, grid, crop, c, t, p)
        if width is None or not (p.min_width <= width <= p.max_width):
            continue
        center = center + normal * _wall_offset(occ, low, grid, crop, center, normal, t, width)
        if not _straight_gap(occ, grid, crop, center, normal, t, width):
            continue
        if not _clear_through(occ, grid, crop, center, normal, t, width, p):
            continue
        outside = _leads_outside(free, occ, covered, grid, crop, center, normal, t, width, p)
        if outside == 'no':
            continue
        hits, passes = _sill(low, low_pass, grid, crop, center, normal, t, width, viewer)
        # a sill is wall material under the gap, and that is what a door lacks.
        # Passes alone can't rule a window out: on a low pad the lidar sits at
        # sill height and shoots straight through the window into the low band
        if hits >= p.sill_min_hits:
            kind = 'window'
        elif passes >= p.door_min_passes:
            kind = 'door'
        else:
            kind = 'unknown'
        conf = (0.3, 0.6, 1.0)[flanks] * min(1.0, len(cells) / 6.0)
        found.append(Opening(center=center, width=width, normal=normal, confidence=conf,
                             kind=kind, sill_hits=hits, low_passes=passes, flanks=flanks,
                             confirmed=outside == 'yes', cells=cells + np.array([i0, j0])))
    return _dedupe(found), covered, crop


def _normal(cells, xy, out_free, covered, roof, grid, crop):
    """Inward unit normal: away from the uncovered free side, or toward more roof."""
    h, w = out_free.shape
    o = np.zeros(2)
    for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        a, b = cells[:, 0] + di, cells[:, 1] + dj
        ok = (a >= 0) & (a < h) & (b >= 0) & (b < w)
        o += np.array([di, dj]) * out_free[a[ok], b[ok]].sum()
    if len(cells) >= 3:
        cov = np.cov((xy - xy.mean(axis=0)).T)
        vals, vecs = np.linalg.eigh(cov)
        t = vecs[:, 1]
        n_pca = np.array([-t[1], t[0]])
        elongated = vals[1] > 2.5 * max(vals[0], 1e-6)
    else:
        n_pca, elongated = None, False
    if elongated:
        n = n_pca
    elif np.linalg.norm(o) > 0:
        n = -o / np.linalg.norm(o)
    else:
        return None
    # sign: the inside has more covered cells and roof hits
    c = xy.mean(axis=0)
    score = []
    for s in (1.0, -1.0):
        ring = c + s * n * 0.7 + _disc_points(0.5, grid.p.res)
        score.append(_sample(covered, grid, crop, ring).sum() + 0.2 * _count(roof, grid, crop, ring))
    if score[0] == score[1]:
        if np.linalg.norm(o) == 0:
            return None
        return n if np.dot(n, -o) >= 0 else -n
    return n if score[0] > score[1] else -n


def _snap_to_wall(occ, grid, crop, c, n0, span=40.0, step=5.0):
    """Turn the normal until the flank walls line up with it.

    The gap cells alone give a normal that can be tens of degrees off when the
    far side is barely seen, and everything after this (width, sill strip,
    approach) is measured along it.
    """
    along = np.concatenate([np.arange(-1.2, -0.44, 0.05), np.arange(0.45, 1.21, 0.05)])
    offs = np.arange(-0.25, 0.251, 0.05)
    def score_of(n):
        t = np.array([-n[1], n[0]])
        return max(int(_sample(occ, grid, crop, c + b * n + along[:, None] * t).sum())
                   for b in offs)

    base = score_of(n0)
    best, best_n = base, n0
    degs = np.arange(-span, span + 1e-9, step)
    for deg in degs[np.argsort(np.abs(degs), kind='stable')]:   # ties: the smallest turn
        a = math.radians(deg)
        n = np.array([n0[0] * math.cos(a) - n0[1] * math.sin(a),
                      n0[0] * math.sin(a) + n0[1] * math.cos(a)])
        score = score_of(n)
        if score > best:
            best, best_n = score, n
    # only a clearly better wall line is worth turning for; a gap whose walls
    # are barely seen keeps the normal the cells gave it
    return best_n if best >= 1.25 * max(base, 1) else n0


def _disc_points(r, step):
    g = np.arange(-r, r + 1e-9, step)
    xx, yy = np.meshgrid(g, g)
    pts = np.stack([xx.ravel(), yy.ravel()], axis=1)
    return pts[np.linalg.norm(pts, axis=1) <= r]


def _gap_width(free, occ, grid, crop, c, t, p):
    """Walk along the gap from c until it stops being free, both ways.

    Returns (centre, width, flanks); the candidate cells sit a little inside the
    wall line, so walk several parallel lines and keep the one with walls beside
    it.
    """
    res = grid.p.res
    n = np.array([t[1], -t[0]])
    best = (None, None, 0)
    steps = np.arange(0.0, p.flank_search + 1e-9, res * 0.5)
    for off in (0.0, -0.1, -0.2, -0.3, 0.1):
        base = c + n * off
        if not _sample(free, grid, crop, base)[0]:
            continue
        ends, flanks = [], 0
        for sgn in (1.0, -1.0):
            pts = base + sgn * steps[:, None] * t
            fr = _sample(free, grid, crop, pts)
            stop = np.nonzero(~fr)[0]
            if len(stop) == 0:
                ends.append(None)
                continue
            k = stop[0]
            ends.append(sgn * steps[k])
            flanks += int(_sample(occ, grid, crop, pts[k:k + 3]).any())
        if None in ends:
            continue
        lo, hi = min(ends), max(ends)
        width = hi - lo
        centre = base + t * (lo + hi) / 2
        if best[1] is None or flanks > best[2] or (flanks == best[2] and width > best[1]):
            best = (centre, width, flanks)
    return best


def _clear_through(occ, grid, crop, center, normal, t, width, p):
    """No wall right in front of or behind the gap (rules out strips hugging a wall)."""
    d = np.arange(0.15, p.probe + 1e-9, grid.p.res * 0.5)
    blocked_lines = 0
    for off in (-0.25, 0.0, 0.25):
        o = center + t * off * min(width, 0.8)
        for sgn in (1.0, -1.0):
            if _sample(occ, grid, crop, o + sgn * d[:, None] * normal).any():
                blocked_lines += 1
                break
    return blocked_lines <= 1


def _wall_offset(occ, low, grid, crop, center, normal, t, width):
    """Where the wall line is along the normal, relative to center.

    The gap cells sit a cell or so off it; the flank walls and the sill say
    where it really is.
    """
    best, best_b = 0, 0.0
    half = width / 2
    along = np.concatenate([np.arange(-half - 0.3, -half, 0.05), np.arange(half + 0.05, half + 0.35, 0.05)])
    gap = np.arange(-half + 0.1, half - 0.1 + 1e-9, 0.05)
    for b in np.arange(-0.3, 0.301, 0.05):
        flank = center + b * normal + along[:, None] * t
        score = 2 * int(_sample(occ, grid, crop, flank).sum())
        score += min(_count(low, grid, crop, center + b * normal + gap[:, None] * t), 20)
        if score > best or (score == best and abs(b) < abs(best_b)):
            best, best_b = score, b
    return best_b


def _straight_gap(occ, grid, crop, center, normal, t, width):
    """No wall inside the gap itself (a wall corner seen at a slant isn't a gap)."""
    half = max(width / 2 - 0.15, 0.05)
    a = np.arange(-half, half + 1e-9, grid.p.res * 0.5)
    hits = sum(int(_sample(occ, grid, crop, center + b * normal + a[:, None] * t).sum())
               for b in (-0.05, 0.05))
    return hits <= 1


def _leads_outside(free, occ, covered, grid, crop, center, normal, t, width, p):
    """'yes' if there's open, roofless space beyond the gap, 'no' if something
    is in the way, 'maybe' if it just hasn't been seen.

    A passage into a room whose roof isn't mapped yet has a wall ~1 m behind
    it; the outside doesn't.
    """
    d = np.arange(0.2, p.outside_depth + 1e-9, grid.p.res * 0.5)
    good = clear = 0
    for off in (-0.2, 0.0, 0.2):
        line = center + t * off * min(width, 0.8) / 0.8 - d[:, None] * normal
        blocked = _sample(occ, grid, crop, line) | _sample(covered, grid, crop, line)
        if not blocked.any():
            clear += 1
            good += _sample(free, grid, crop, line).mean() >= 0.6
    if good >= 2:
        return 'yes'
    return 'maybe' if clear >= 2 else 'no'


def _sill(low, low_pass, grid, crop, center, normal, t, width, viewer=None):
    """(sill hits at the wall line, low-band passes just beyond it).

    Passes count only on the far side from the viewer: rays landing on a pad
    top in front of a window pass through the low band on the near side.
    """
    half = max(width / 2 - 0.1, 0.1)
    step = grid.p.res * 0.5
    a = np.arange(-half, half + 1e-9, step)

    def strip(b0, b1):
        aa, bb = np.meshgrid(a, np.arange(b0, b1 + 1e-9, step))
        return center + aa.ravel()[:, None] * t + bb.ravel()[:, None] * normal

    side = 1.0
    if viewer is not None and np.dot(np.asarray(viewer) - center, normal) > 0:
        side = -1.0
    # just past the wall: a ray that cleared a sill can't drop into the low band
    # that fast with the Mid-360's -7 deg floor
    far = strip(0.1, 0.2) if side > 0 else strip(-0.2, -0.1)
    return _count(low, grid, crop, strip(-0.15, 0.15)), _count(low_pass, grid, crop, far)


def _dedupe(found, dist=0.5):
    """Merge detections of the same gap, keeping the most confident."""
    found = sorted(found, key=lambda o: -o.confidence)
    out = []
    for o in found:
        if all(np.linalg.norm(o.center - q.center) > dist for q in out):
            out.append(o)
    return out


def window_extent(points, opening, flight_z, slab=0.15):
    """(bottom, top) z of the gap from raw 3D points in the wall plane, or None each.

    bottom = top of the sill, top = lintel or roof. Their middle is the height to
    fly through at.
    """
    if points is None or len(points) == 0:
        return None, None
    d = points[:, :2] - opening.center
    along_n = d @ opening.normal
    along_t = d @ opening.tangent
    near = (np.abs(along_n) < slab) & (np.abs(along_t) < max(opening.width / 2 - 0.1, 0.1))
    z = points[near, 2]
    below, above = z[z < flight_z], z[z >= flight_z]
    bot = float(np.percentile(below, 95)) if len(below) >= 5 else None
    top = float(np.percentile(above, 5)) if len(above) >= 5 else None
    return bot, top
