"""Box worlds for the kinematic sim: axis-aligned boxes and a floor at z = 0.

Two ways in:
  build(layout)          hand-written 2 x 6 m box mazes (the explorer's test layouts)
  from_known_map(km)     any KnownMap with axis-aligned walls, optionally perturbed
                         (passages moved or closed) so the real thing differs
                         from what the drone was told, plus QR codes on walls

The hand-written layouts:

Canonical layout (world frame, floor z = 0, metres):
  arena      [0, 8] x [0, 8], walls 0.5 m tall
  takeoff    pad [0, 2] x [0, 2], 0.54 m tall, drone base_link 0.08 above it
  structure  [2, 8] x [0, 2], 1.5 m tall, roofed, along the y = 0 arena wall
  entrance   window in the x = 2 end wall, facing the pad
  door       in the y = 2 long face near that end
  exit       window in the y = 2 long face at the far end
  landing    pad in the middle, (4, 4)

The layout can be mirrored and turned by 90 deg steps (boxes stay axis-aligned);
the odom yaw at takeoff is free, so odom isn't aligned with the walls.
"""
from dataclasses import dataclass, field

import numpy as np

WALL_H = 1.5
OUTER_T = 0.08
INNER_T = 0.06
SILL = 0.7


@dataclass
class World:
    lo: np.ndarray                  # (M,3)
    hi: np.ndarray
    tags: list
    structure: tuple                # (xmin, xmax, ymin, ymax)
    openings: dict                  # name -> (a (2,), b (2,), z0, z1): gap segment on the wall line
    arena_center: np.ndarray
    takeoff: np.ndarray             # (3,) base_link at takeoff
    yaw0: float = 0.0               # odom yaw in the world
    layout: str = 'nominal'
    meta: dict = field(default_factory=dict)

    # odom <-> world
    def R(self):
        c, s = np.cos(self.yaw0), np.sin(self.yaw0)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def to_odom(self, p):
        return (np.asarray(p) - self.takeoff) @ self.R()

    def to_world(self, p):
        return np.asarray(p) @ self.R().T + self.takeoff

    def vec_to_world(self, v):
        return np.asarray(v) @ self.R().T

    def support_z(self, xy):
        """Top of whatever is under xy (floor = 0)."""
        z = 0.0
        for lo, hi in zip(self.lo, self.hi):
            if lo[0] <= xy[0] <= hi[0] and lo[1] <= xy[1] <= hi[1] and hi[2] < 1.0:
                z = max(z, hi[2])
        return z


def _wall(axis, at, a0, a1, thick, gaps=(), h=WALL_H, inward=1.0, tag='wall'):
    """A wall on the plane x=at (axis 'x') or y=at, spanning a0..a1 on the other axis.

    thick grows toward `inward` (+1/-1). gaps = [(g0, g1, z0, z1)].
    """
    boxes = []
    t0, t1 = sorted((at, at + inward * thick))
    cuts = sorted(gaps)
    pos = a0
    pieces = []
    for g0, g1, z0, z1 in cuts:
        if g0 > pos:
            pieces.append((pos, g0, 0.0, h, tag))
        if z0 > 0:
            pieces.append((g0, g1, 0.0, z0, 'sill'))
        if z1 < h:
            pieces.append((g0, g1, z1, h, 'lintel'))
        pos = g1
    if pos < a1:
        pieces.append((pos, a1, 0.0, h, tag))
    for s0, s1, z0, z1, tg in pieces:
        if axis == 'x':
            boxes.append(((t0, s0, z0), (t1, s1, z1), tg))
        else:
            boxes.append(((s0, t0, z0), (s1, t1, z1), tg))
    return boxes


def canonical(layout='nominal'):
    boxes = []
    # arena walls, outside [0, 8]
    for lo, hi in (((-0.1, -0.1, 0), (8.1, 0, 0.5)), ((-0.1, 8, 0), (8.1, 8.1, 0.5)),
                   ((-0.1, 0, 0), (0, 8, 0.5)), ((8, 0, 0), (8.1, 8, 0.5))):
        boxes.append((lo, hi, 'arena'))
    boxes.append(((0, 0, 0), (2, 2, 0.54), 'takeoff_pad'))
    boxes.append(((3.5, 3.5, 0), (4.5, 4.5, 0.1), 'landing_pad'))
    ent = (1.05, 1.85, SILL, WALL_H)
    door = (3.3, 4.1, 0.0, WALL_H)
    ext = (7.0, 7.8, SILL, WALL_H)
    openings = {'entrance': ((2.0, 1.05), (2.0, 1.85), SILL, WALL_H),
                'door': ((3.3, 2.0), (4.1, 2.0), 0.0, WALL_H),
                'exit': ((7.0, 2.0), (7.8, 2.0), SILL, WALL_H)}
    # outer walls, thickness inside the footprint
    boxes += _wall('x', 2.0, 0.0, 2.0, OUTER_T, [ent], inward=1)
    boxes += _wall('x', 8.0, 0.0, 2.0, OUTER_T, inward=-1)
    boxes += _wall('y', 0.0, 2.0, 8.0, OUTER_T, inward=1)
    boxes += _wall('y', 2.0, 2.0, 8.0, OUTER_T, [door, ext], inward=-1)
    boxes.append(((2.0, 0.0, WALL_H), (8.0, 2.0, WALL_H + 0.06), 'roof'))
    T = INNER_T / 2
    full = lambda g0, g1: (g0, g1, 0.0, WALL_H)  # noqa: E731
    if layout == 'nominal':
        inner = [
            ('y', 1.0, 2.0, 3.0, []),                       # entrance room | small cell A
            ('x', 3.0, 0.0, 2.0, [full(0.08, 0.93), full(1.07, 1.92)]),   # A, E -> big room
            ('x', 5.0, 0.0, 2.0, [full(0.08, 0.93)]),       # big room -> C2
            ('y', 1.0, 5.0, 6.0, [full(5.07, 5.92)]),       # C2 -> C1
            ('x', 6.0, 0.0, 2.0, [full(1.07, 1.92)]),       # C1 -> L1
            ('y', 1.0, 6.0, 8.0, [full(6.08, 6.93)]),       # L1 -> L2 (dead end)
        ]
    elif layout == 'variant':
        inner = [
            ('y', 1.0, 2.0, 3.0, [full(2.1, 2.92)]),        # E -> A (dead end)
            ('x', 3.0, 0.0, 2.0, [full(1.07, 1.92)]),       # E -> big room
            ('x', 4.0, 0.0, 1.0, []),                       # stub splitting the big room
            ('x', 5.0, 0.0, 2.0, [full(1.07, 1.92)]),       # big room -> C1
            ('y', 1.0, 5.0, 6.0, [full(5.07, 5.92)]),       # C1 -> C2 (dead end)
            ('x', 6.0, 0.0, 2.0, [full(1.07, 1.92)]),       # C1 -> L1
            ('y', 1.0, 6.0, 8.0, [full(7.08, 7.92)]),       # L1 -> L2 (dead end, far side)
        ]
    else:
        raise ValueError(layout)
    for axis, at, a0, a1, gaps in inner:
        # interior walls centred on their line
        for lo, hi, tg in _wall(axis, at - T, a0, a1, INNER_T, gaps, inward=1, tag='inner'):
            boxes.append((lo, hi, tg))
    return boxes, openings


def build(layout='nominal', mirror=False, rot90=0, yaw0=0.0):
    """World in its final pose. rot90 turns the layout about the arena centre."""
    boxes, openings = canonical(layout)
    c = np.array([4.0, 4.0])

    def tf(xy):
        xy = np.array(xy, dtype=float)
        if mirror:
            xy = np.array([xy[0], 8.0 - xy[1]])
        for _ in range(rot90 % 4):
            d = xy - c
            xy = c + np.array([-d[1], d[0]])
        return xy

    lo, hi, tags = [], [], []
    for a, b, tg in boxes:
        p, q = tf(a[:2]), tf(b[:2])
        lo.append([min(p[0], q[0]), min(p[1], q[1]), a[2]])
        hi.append([max(p[0], q[0]), max(p[1], q[1]), b[2]])
        tags.append(tg)
    s0, s1 = tf((2.0, 0.0)), tf((8.0, 2.0))
    structure = (min(s0[0], s1[0]), max(s0[0], s1[0]), min(s0[1], s1[1]), max(s0[1], s1[1]))
    ops = {k: (tf(a), tf(b), z0, z1) for k, (a, b, z0, z1) in openings.items()}
    t = tf((1.0, 1.0))
    return World(lo=np.array(lo), hi=np.array(hi), tags=tags, structure=structure, openings=ops,
                 arena_center=c.copy(), takeoff=np.array([t[0], t[1], 0.54 + 0.08]), yaw0=yaw0,
                 layout=layout)


def interior_cells(world, res=0.1, z=1.1, clearance=0.15):
    """Ground-truth interior free cells at flight height: world xy centres (K,2).

    Cells inside the structure whose centre is at least `clearance` from every
    wall box that crosses height z.
    """
    x0, x1, y0, y1 = world.structure
    xs = np.arange(x0 + res / 2, x1, res)
    ys = np.arange(y0 + res / 2, y1, res)
    xx, yy = np.meshgrid(xs, ys, indexing='ij')
    pts = np.stack([xx.ravel(), yy.ravel()], axis=1)
    ok = np.ones(len(pts), dtype=bool)
    for lo, hi in zip(world.lo, world.hi):
        if not (lo[2] <= z <= hi[2]):
            continue
        dx = np.maximum(np.maximum(lo[0] - pts[:, 0], 0), pts[:, 0] - hi[0])
        dy = np.maximum(np.maximum(lo[1] - pts[:, 1], 0), pts[:, 1] - hi[1])
        ok &= np.hypot(dx, dy) >= clearance
    return pts[ok]


def _wall_boxes(w, gaps, tag):
    """Boxes of one axis-aligned KnownMap wall, centred on its line."""
    t = max(w.t, 0.02)
    if abs(w.a[0] - w.b[0]) < 1e-6:
        axis, at, a0, a1 = 'x', w.a[0] - t / 2, w.a[1], w.b[1]
    elif abs(w.a[1] - w.b[1]) < 1e-6:
        axis, at, a0, a1 = 'y', w.a[1] - t / 2, w.a[0], w.b[0]
    else:
        raise ValueError(f'wall {w.name} is not axis-aligned; the box sim needs it to be')
    sgn = 1.0 if a1 >= a0 else -1.0
    lo_, hi_ = min(a0, a1), max(a0, a1)
    g = [(min(a0 + sgn * u0, a0 + sgn * u1), max(a0 + sgn * u0, a0 + sgn * u1), z0, z1)
         for u0, u1, z0, z1 in gaps]
    return _wall(axis, at, lo_, hi_, t, g, h=w.z[1], inward=1, tag=tag)


def from_known_map(km, yaw_error=0.0, xy_error=(0.0, 0.0), perturb=None, qr=None, seed=0,
                   event_walls=True):
    """A World in the map's own frame (world = map frame).

    yaw_error / xy_error: how far the drone really sits from the map's takeoff pose.
    perturb: {opening name: None (walled up) | {'u': .., 'z': ..} (moved/resized)}.
    qr: {letter: (xy, z, normal)} or an int n to scatter n codes on inner walls.
    """
    perturb = perturb or {}
    boxes, openings = [], {}
    for w in km.walls:
        gaps = []
        for o in w.openings:
            if o.name in perturb and perturb[o.name] is None:
                continue
            q = perturb.get(o.name) or {}
            u, z = tuple(q.get('u', o.u)), tuple(q.get('z', o.z))
            gaps.append((u[0], u[1], z[0], z[1]))
            openings[o.name] = (w.at(u[0]), w.at(u[1]), z[0], z[1])
        tag = 'wall' if w.kind == 'outer' else 'inner'
        boxes += _wall_boxes(w, gaps, tag)
    x0, x1, y0, y1 = km.structure
    h = km.roof_z or 1.5
    boxes.append(((x0, y0, h), (x1, y1, h + 0.06), 'roof'))
    to = km.takeoff
    c, sz, top = to['center'], to['size'], float(to.get('top', 0.54))
    boxes.append(((c[0] - sz[0] / 2, c[1] - sz[1] / 2, 0.0), (c[0] + sz[0] / 2, c[1] + sz[1] / 2, top),
                  'takeoff_pad'))
    if km.landing is not None:
        lp = km.landing
        boxes.append(((lp[0] - 0.5, lp[1] - 0.5, 0.0), (lp[0] + 0.5, lp[1] + 0.5, 0.02), 'landing_pad'))
    if event_walls and km.arena is not None:
        # the hall around the arena: something for the lidar to hit outside
        a0, a1, b0, b1 = km.arena
        m = 1.5
        for lo, hi in (((a0 - m - 0.1, b0 - m - 0.1, 0), (a1 + m + 0.1, b0 - m, 3.0)),
                       ((a0 - m - 0.1, b1 + m, 0), (a1 + m + 0.1, b1 + m + 0.1, 3.0)),
                       ((a0 - m - 0.1, b0 - m, 0), (a0 - m, b1 + m, 3.0)),
                       ((a1 + m, b0 - m, 0), (a1 + m + 0.1, b1 + m, 3.0))):
            boxes.append((lo, hi, 'event_wall'))
    lo = np.array([b[0] for b in boxes], dtype=float)
    hi = np.array([b[1] for b in boxes], dtype=float)
    tags = [b[2] for b in boxes]
    takeoff = np.array([c[0] + xy_error[0], c[1] + xy_error[1], top + 0.08])
    arena_c = (np.array([0.5 * (km.arena[0] + km.arena[1]), 0.5 * (km.arena[2] + km.arena[3])])
               if km.arena else (km.landing if km.landing is not None else np.zeros(2)))
    world = World(lo=lo, hi=hi, tags=tags, structure=tuple(km.structure), openings=openings,
                  arena_center=arena_c, takeoff=takeoff, yaw0=float(to.get('yaw', 0.0)) + yaw_error,
                  layout=km.name or 'known_map')
    if isinstance(qr, int):
        qr = scatter_qr(km, world, qr, seed)
    world.meta['qr'] = qr or {}
    return world


def scatter_qr(km, world, n, seed=0, letters='ABCDE'):
    """n QR codes on inner wall faces of distinct rooms, not in openings.

    Each one: (xy on the wall face, z, normal pointing into the room).
    """
    from hydrone_confined.sim import lidar
    rng = np.random.default_rng(seed)
    rooms = list(km.rooms)
    rng.shuffle(rooms)
    out = {}
    tries = 0
    while len(out) < n and tries < 500:
        tries += 1
        r = rooms[len(out) % len(rooms)] if len(out) < len(rooms) else rooms[rng.integers(len(rooms))]
        c = r.center
        side = rng.integers(4)
        normal = [np.array([1.0, 0.0]), np.array([-1.0, 0.0]), np.array([0.0, 1.0]), np.array([0.0, -1.0])][side]
        d = -normal            # look from the centre toward that wall
        z = float(rng.uniform(0.35, 1.15))
        # where does that wall face sit, at some point along it
        along = np.array([-d[1], d[0]])
        half = 0.5 * (abs((r.x[1] - r.x[0]) * along[0]) + abs((r.y[1] - r.y[0]) * along[1])) - 0.15
        p0 = c + along * rng.uniform(-half, half)
        eye = np.array([p0[0], p0[1], z])
        rngs, which = lidar.cast(eye, np.array([[d[0], d[1], 0.0]]), world.lo, world.hi)
        if not np.isfinite(rngs[0]) or rngs[0] > 1.5 or world.tags[which[0]] not in ('inner', 'wall'):
            continue
        xy = p0 + d * (rngs[0] - 0.005)
        if any(np.linalg.norm(xy - q[0]) < 0.4 for q in out.values()):
            continue
        out[letters[len(out)]] = (xy, z, normal)
    return out
