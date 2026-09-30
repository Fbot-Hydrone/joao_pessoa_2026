"""A known 2.5D map of a confined space: walls with openings, in its own frame.

Walls are straight, have a thickness and a height, and carry openings (a gap
along the wall between two heights: a window, a door, a low hole). Rooms are
rectangles, used for routes and coverage, not for ray casting. Floor is z = 0.

What a lidar sees at some height depends on that height: a window is a gap at
1.1 m and a wall at 0.4 m. So everything here is asked for a height *band*
[z0, z1]: a wall is solid in the band unless an opening spans all of it. That
matches a beam taken as the nearest return inside the band.

Loaded from YAML (see hydrone_mission/config/phase4/maps/rulebook.yaml).
"""
from dataclasses import dataclass, field

import numpy as np
import yaml


@dataclass
class Opening:
    u: tuple                    # (u0, u1) along the wall from its `a` end, m
    z: tuple                    # (z0, z1) above the floor
    kind: str = 'window'        # window / door / hole
    name: str = ''
    role: str = 'interior'      # interior / entry / exit / forbidden
    soft: bool = False          # guessed, not measured: the live data has the last word

    @property
    def pass_z(self):
        return 0.5 * (self.z[0] + self.z[1])

    @property
    def width(self):
        return self.u[1] - self.u[0]


@dataclass
class Wall:
    a: np.ndarray
    b: np.ndarray
    t: float = 0.06             # thickness, centred on a-b
    z: tuple = (0.0, 1.5)
    name: str = ''
    kind: str = 'inner'         # inner / outer
    openings: list = field(default_factory=list)

    @property
    def length(self):
        return float(np.linalg.norm(self.b - self.a))

    @property
    def tangent(self):
        return (self.b - self.a) / max(self.length, 1e-9)

    @property
    def normal(self):
        t = self.tangent
        return np.array([-t[1], t[0]])

    def at(self, u):
        return self.a + self.tangent * u

    def solid_spans(self, z0, z1):
        """(u0, u1) pieces of the wall that block something between z0 and z1."""
        if z1 <= self.z[0] or z0 >= self.z[1]:
            return []
        gaps = sorted((o.u for o in self.openings if o.z[0] <= z0 and o.z[1] >= z1))
        spans, u = [], 0.0
        for g0, g1 in gaps:
            if g0 > u:
                spans.append((u, g0))
            u = max(u, g1)
        if u < self.length:
            spans.append((u, self.length))
        return [(a, b) for a, b in spans if b - a > 1e-3]


@dataclass
class Room:
    name: str
    x: tuple
    y: tuple

    @property
    def center(self):
        return np.array([0.5 * (self.x[0] + self.x[1]), 0.5 * (self.y[0] + self.y[1])])

    def contains(self, xy, margin=0.0):
        return (self.x[0] - margin <= xy[0] <= self.x[1] + margin
                and self.y[0] - margin <= xy[1] <= self.y[1] + margin)


@dataclass
class PassageRef:
    """An opening, located: which wall it is on and where its centre is."""
    wall: Wall
    opening: Opening

    @property
    def name(self):
        return self.opening.name

    @property
    def center(self):
        return self.wall.at(0.5 * (self.opening.u[0] + self.opening.u[1]))

    @property
    def normal(self):
        return self.wall.normal

    @property
    def tangent(self):
        return self.wall.tangent


class KnownMap:

    def __init__(self, walls, rooms=(), structure=None, arena=None, takeoff=None, landing=None,
                 name='', roof_z=None):
        self.walls = list(walls)
        self.rooms = list(rooms)
        self.structure = structure      # (x0, x1, y0, y1) roofed footprint, or None
        self.arena = arena              # (x0, x1, y0, y1) or None
        self.takeoff = takeoff or {}    # center, size, top, yaw (rad)
        self.landing = landing          # (2,) or None
        self.name = name
        self.roof_z = roof_z
        self._slices = {}

    # ── loading ─────────────────────────────────────────────────────────────
    @classmethod
    def from_yaml(cls, path):
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f))

    @classmethod
    def from_dict(cls, d):
        walls = []
        for w in d.get('walls', []):
            ops = [Opening(u=tuple(o['u']), z=tuple(o['z']), kind=o.get('kind', 'window'),
                           name=o.get('name', ''), role=o.get('role', 'interior'),
                           soft=bool(o.get('soft', False)))
                   for o in w.get('openings', [])]
            walls.append(Wall(a=np.array(w['a'], dtype=float), b=np.array(w['b'], dtype=float),
                              t=float(w.get('t', 0.06)), z=tuple(w.get('z', (0.0, 1.5))),
                              name=w.get('name', ''), kind=w.get('kind', 'inner'), openings=ops))
        rooms = [Room(r['name'], tuple(r['x']), tuple(r['y'])) for r in d.get('rooms', [])]
        s = d.get('structure')
        structure = tuple(s['x']) + tuple(s['y']) if s else None
        a = d.get('arena')
        arena = tuple(a['x']) + tuple(a['y']) if a else None
        to = dict(d.get('takeoff', {}))
        if 'center' in to:
            to['center'] = np.array(to['center'], dtype=float)
        to['yaw'] = np.deg2rad(float(to.get('yaw_deg', 0.0)))
        lp = d.get('landing', {}).get('center')
        return cls(walls, rooms, structure, arena, to,
                   None if lp is None else np.array(lp, dtype=float),
                   d.get('name', ''), s.get('height') if s else None)

    # ── queries ─────────────────────────────────────────────────────────────
    def passages(self, role=None):
        out = [PassageRef(w, o) for w in self.walls for o in w.openings]
        return [p for p in out if role is None or p.opening.role == role]

    def passage(self, name):
        for p in self.passages():
            if p.name == name:
                return p
        raise KeyError(name)

    def room_at(self, xy):
        for r in self.rooms:
            if r.contains(xy):
                return r
        return None

    def inside_structure(self, xy, margin=0.0):
        if self.structure is None:
            return False
        x0, x1, y0, y1 = self.structure
        return x0 - margin <= xy[0] <= x1 + margin and y0 - margin <= xy[1] <= y1 + margin

    def ceiling_at(self, xy):
        """z of the surface above xy (the roof over the structure), or None."""
        return self.roof_z if self.inside_structure(xy) else None

    def support_at(self, xy):
        """z of what's under xy: the takeoff pad top, or the floor."""
        to = self.takeoff
        if 'center' in to and 'size' in to:
            c, s = to['center'], to['size']
            if abs(xy[0] - c[0]) <= s[0] / 2 and abs(xy[1] - c[1]) <= s[1] / 2:
                return float(to.get('top', 0.0))
        return 0.0

    # ── geometry for ray casting ────────────────────────────────────────────
    def segments(self, z0, z1):
        """(M,2,2) wall faces solid somewhere in [z0, z1]: each solid piece is a
        thin rectangle, all four sides (the jambs matter in doorways)."""
        key = (round(z0, 2), round(z1, 2))
        if key in self._slices:
            return self._slices[key]
        segs = []
        for w in self.walls:
            t, n = w.tangent, w.normal * (w.t / 2)
            for u0, u1 in w.solid_spans(z0, z1):
                p0, p1 = w.at(u0), w.at(u1)
                c = [p0 + n, p1 + n, p1 - n, p0 - n]
                segs += [(c[0], c[1]), (c[1], c[2]), (c[2], c[3]), (c[3], c[0])]
        out = np.array(segs, dtype=float).reshape(-1, 2, 2)
        if len(self._slices) > 64:
            self._slices.clear()
        self._slices[key] = out
        return out

    def raycast(self, origins, angles, band, max_range=12.0):
        """Ranges from each origin along each angle to the nearest wall face.

        origins (P,2), angles (P,K) world angles -> (P,K); inf past max_range.
        """
        segs = self.segments(*band)
        o = np.asarray(origins, dtype=float)[:, None, None, :]         # P,1,1,2
        ang = np.asarray(angles, dtype=float)
        d = np.stack([np.cos(ang), np.sin(ang)], axis=-1)[:, :, None, :]  # P,K,1,2
        if not len(segs):
            return np.full(ang.shape, np.inf)
        a = segs[None, None, :, 0, :]                                     # 1,1,M,2
        e = segs[None, None, :, 1, :] - a
        w = a - o
        den = d[..., 0] * e[..., 1] - d[..., 1] * e[..., 0]               # P,K,M
        with np.errstate(divide='ignore', invalid='ignore'):
            r = (w[..., 0] * e[..., 1] - w[..., 1] * e[..., 0]) / den
            s = (w[..., 0] * d[..., 1] - w[..., 1] * d[..., 0]) / den
        hit = (np.abs(den) > 1e-12) & (r > 0) & (s >= 0) & (s <= 1)
        r = np.where(hit, r, np.inf).min(axis=-1)
        r[r > max_range] = np.inf
        return r

    def rasterize(self, band, res=0.05, margin=0.5, bounds=None):
        """Occupancy of the band as a bool grid. Returns (occ, origin (2,), res).

        Indexing is [i, j] with i along x, like the online OccupancyGrid.
        """
        if bounds is None:
            bounds = self.arena or self.structure
        x0, x1, y0, y1 = bounds
        origin = np.array([x0 - margin, y0 - margin])
        nx = int(np.ceil((x1 - x0 + 2 * margin) / res))
        ny = int(np.ceil((y1 - y0 + 2 * margin) / res))
        xs = origin[0] + (np.arange(nx) + 0.5) * res
        ys = origin[1] + (np.arange(ny) + 0.5) * res
        X, Y = np.meshgrid(xs, ys, indexing='ij')
        occ = np.zeros((nx, ny), dtype=bool)
        for w in self.walls:
            t, n = w.tangent, w.normal
            for u0, u1 in w.solid_spans(*band):
                ru = (X - w.a[0]) * t[0] + (Y - w.a[1]) * t[1]
                rn = (X - w.a[0]) * n[0] + (Y - w.a[1]) * n[1]
                occ |= (ru >= u0 - res / 2) & (ru <= u1 + res / 2) & (np.abs(rn) <= w.t / 2 + res / 2)
        return occ, origin, res
