"""The Phase 4 maze mission as a pure class: `Mission.step(t, pose, scan) -> Command`.

TAKEOFF   climb above flight height (sills are seen best while low, so the
          climb itself is part of the survey)
SURVEY    hover high, then at flight height, over the takeoff point
FIND_ENTRANCE  a window reachable from here, nearest the takeoff point. No
          window yet: look lower, then from a vantage point outside, then ask
          the rulebook fallback
ALIGN     fly outside to align_dist in front of it, along its normal
ALIGN_HOLD settle at the window-centre height
ENTER     straight through at a crawl, re-checking the gap
EXPLORE   frontier exploration inside the covered region until no reachable
          frontier is left; openings > exit_min_dist from the entry are exits
GO_EXIT / ALIGN_EXIT / PASS_EXIT   same as entering, the other way
GO_LAND   to the arena middle outside the structure (climb over it if needed)
LAND

Commands are velocities in odom with yaw held constant (the lidar is 360 deg).
"""
from collections import deque
from dataclasses import dataclass, field, replace
import math
import time

import numpy as np

from hydrone_mission.maze import frontiers as fr
from hydrone_mission.maze import geometry_fallback as gf
from hydrone_mission.maze.arena import arena_from_grid
from hydrone_mission.maze.grid import GridParams, OccupancyGrid
from hydrone_mission.maze.morph import dilate, flood
from hydrone_mission.maze.openings import OpeningParams, detect_openings, window_extent
from hydrone_mission.maze.planner import GridPlanner, inflate


@dataclass
class MissionParams:
    flight_z: float = 0.5           # odom z to fly at (window centre height guess)
    inside_bands: tuple = (0.40, 0.75)   # interior bands, m above the floor
    survey_rise: float = 0.6        # takeoff climbs this far above flight_z
    survey_hold: float = 2.0        # s hovering at each survey height
    inflate: float = 0.22           # obstacle inflation outside, m (330 mm drone)
    inflate_in: float = 0.16        # and inside, where 0.6 m doorways close up
                                    # entirely at 0.22 (the body radius is 0.165
                                    # and the wall repulsion does the rest)
    v_in: float = 0.25              # speed cap inside, m/s
    v_out: float = 0.5              # and outside
    v_crawl: float = 0.15           # through a window
    v_z: float = 0.3
    kp_xy: float = 0.8
    kp_z: float = 1.0
    lookahead: float = 0.4
    wall_slow: float = 0.35         # slow down and push off walls closer than this
    repulse_gain: float = 0.15      # m/s at contact
    goal_tol: float = 0.15
    replan_s: float = 1.0
    perceive_s: float = 0.5         # openings/frontiers refresh period
    align_dist: float = 0.8         # stand-off in front of a window
    enter_depth: float = 0.6        # inside past the window before exploring
    exit_out: float = 1.0           # outside past the exit before going to land
    window_z_band: float = 0.25     # measured window centre accepted within this of flight_z
    exit_min_dist: float = 4.0
    find_look: float = 6.0          # s per FIND_ENTRANCE stage (here, low, vantage)
    vantage_drop: float = 0.35      # lower hover for a better look at sills
    min_hover_z: float = 0.15       # above the takeoff height
    explore_budget: float = 330.0   # s
    mission_budget: float = 560.0   # s, then land wherever we are
    stuck_s: float = 35.0           # give up a frontier goal after this long
    give_up_cycles: int = 12        # planning cycles with nothing to explore
    probe_s: float = 40.0           # s to reach a place to look from
    probe_radius: float = 1.0       # unexplored spots this near one already probed
    sweep_hold: float = 3.0         # s hovering per band while looking around
    view_tries: int = 3             # view cells tried before a frontier is dropped
    climb_margin: float = 0.5       # over the roof when there's no way round
    use_fallback: bool = True
    buffer_scans: int = 20          # recent scans kept in 3D for window heights
    grid: GridParams = field(default_factory=GridParams)
    openings: OpeningParams = field(default_factory=OpeningParams)
    frontiers: fr.FrontierParams = field(default_factory=fr.FrontierParams)
    fallback: gf.FallbackParams = field(default_factory=gf.FallbackParams)


@dataclass
class Command:
    kind: str                                   # velocity / hold / land / takeoff
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))   # odom m/s
    z: float = None                             # takeoff altitude (odom)
    yaw_rate: float = 0.0
    debug: dict = field(default_factory=dict)


class Mission:

    def __init__(self, params=None, logger=None):
        self.p = params or MissionParams()
        self.p.grid.flight_z = self.p.flight_z
        self.grid = OccupancyGrid(self.p.grid)
        # More grids, lower down, for planning inside. A window sits high in its
        # wall while the passages behind it can be much lower (measured in
        # BiguaSim: window 0.0..0.8, some inner doorways floor..0.15), and
        # different passages sit at different heights, so the mission keeps one
        # grid per band and explores in whichever one has somewhere to go.
        self.bands = []
        self.ib = 0
        self.logger = logger
        self.log = []                           # (t, message)
        self.phase = 'TAKEOFF'
        self.t_phase = None
        self.t0 = None
        self.t = 0.0
        self.takeoff = None                     # (3,) odom
        self.buffer = deque(maxlen=self.p.buffer_scans)
        self.scan_ms = []
        self.perceive_ms = []
        self.step_ms = []                       # whole step, for CPU budgets
        self.fallback_used = []
        # perception
        self.crop = None
        self.covered = None
        self.openings = []
        self.arena = None
        self.roof_top = None
        self._t_perceive = -1e9
        # plans
        self.path = None
        self._t_plan = -1e9
        self.entrance = None
        self.entry_point = None
        self.exit = None
        self.pass_z = self.p.flight_z
        self.frontiers = []
        self.frontier_goal = None
        self._goal_since = None
        self._ignore = []
        self._bad_views = []
        self._view_of = []
        self._no_frontier = 0
        self.landing = None
        self.done = False
        self._retries = 0
        self._blocked_since = None
        self._bad_exits = []
        self._shift_back = 'EXPLORE'
        self._probed = []
        self._ib_sweep = 0
        self.probe_target = None
        self.probe_goal = None

    # ── bookkeeping ─────────────────────────────────────────────────────────
    def say(self, msg):
        self.log.append((self.t, msg))
        if self.logger:
            self.logger(msg)

    def go(self, phase, why=''):
        self.say(f'{self.phase} -> {phase}' + (f' ({why})' if why else ''))
        self.phase, self.t_phase = phase, self.t
        self.path = None
        self._t_plan = -1e9
        self._blocked_since = None

    def elapsed(self):
        return self.t - self.t_phase

    # ── the tick ────────────────────────────────────────────────────────────
    def step(self, t, pose, scan=None):
        """pose = (position (3,), yaw); scan = (points (N,3), sensor_origin (3,)) or None."""
        c_step = time.perf_counter()
        pos = np.asarray(pose[0], dtype=float)
        self.t = t
        if self.t0 is None:
            self.t0, self.t_phase = t, t
            self.takeoff = pos.copy()
        if scan is not None:
            self._ingest(*scan, pos)
        if t - self._t_perceive >= self.p.perceive_s and scan is not None:
            self._t_perceive = t
            c0 = time.perf_counter()
            self._perceive(pos)
            self.perceive_ms.append((time.perf_counter() - c0) * 1e3)
        if t - self.t0 > self.p.mission_budget and self.phase not in ('LAND', 'GO_LAND', 'CLIMB', 'PASS_EXIT'):
            self.go('LAND', 'mission budget spent')
        cmd = getattr(self, '_' + self.phase.lower())(pos)
        cmd.debug = self.debug()
        self.step_ms.append((time.perf_counter() - c_step) * 1e3)
        return cmd

    @property
    def igrid(self):
        return self.bands[self.ib] if self.bands else None

    @property
    def inside_z(self):
        return self.igrid.p.flight_z if self.bands else self.p.flight_z

    def _make_bands(self):
        floor = self.grid.floor_z
        for dz in self.p.inside_bands:
            z = floor + dz
            if z < self.p.flight_z - 0.15:      # the window band already covers the top
                self.bands.append(OccupancyGrid(replace(self.p.grid, flight_z=z, floor_z=floor)))
        if self.bands:
            self.say('interior bands at z '
                     + ', '.join(f'{g.p.flight_z:.2f}' for g in self.bands)
                     + f' (floor {floor:.2f})')

    def _ingest(self, points, origin, pos):
        c0 = time.perf_counter()
        pts = np.asarray(points, dtype=np.float32)
        self.grid.update(pts, origin)
        if not self.bands and self.grid.floor_z is not None:
            self._make_bands()
        for g in self.bands:
            g.update(pts, origin)
        fz = self.p.flight_z
        # a thin 3D memory around the drone, for window heights
        near = (np.abs(pts[:, 0] - pos[0]) < 3.0) & (np.abs(pts[:, 1] - pos[1]) < 3.0) \
            & (pts[:, 2] < fz + 1.0)
        self.buffer.append(pts[near][::2])
        roof = pts[(pts[:, 2] > fz + self.p.grid.roof_lo) & (pts[:, 2] < fz + self.p.grid.roof_hi), 2]
        if len(roof) > 20:
            top = float(np.percentile(roof, 98))
            self.roof_top = top if self.roof_top is None else max(self.roof_top, top)
        self.scan_ms.append((time.perf_counter() - c0) * 1e3)

    def _perceive(self, pos):
        self.openings, self.covered, self.crop = detect_openings(self.grid, self.p.openings, viewer=pos[:2])
        if self.arena is None or int(self.t * 2) % 4 == 0:
            self.arena = arena_from_grid(self.grid, self._inside_points()) or self.arena

    def debug(self):
        return {'phase': self.phase, 'grid': self.grid, 'covered': self.covered, 'crop': self.crop,
                'openings': self.openings, 'frontiers': self.frontiers, 'path': self.path,
                'entrance': self.entrance, 'exit': self.exit, 'landing': self.landing,
                'arena': self.arena, 'fallback_used': list(self.fallback_used)}

    # ── maps for planning ───────────────────────────────────────────────────
    def _masks(self, grid=None):
        """Crop plus free/occupied/covered/clear. `grid` picks the band; the
        covered region always comes from the detection band above, and an
        interior band is inflated less."""
        crop = self.crop or self.grid.crop(1.5)
        i0, i1, j0, j1 = crop
        g = grid if grid is not None else self.grid
        radius = self.p.inflate_in if g is not self.grid else self.p.inflate
        free = g.free()[i0:i1, j0:j1]
        occ = g.occupied()[i0:i1, j0:j1]
        covered = self.covered if self.covered is not None and self.covered.shape == free.shape \
            else np.zeros_like(free)
        clear = free & ~inflate(occ, radius / g.p.res)
        return crop, free, occ, covered, clear

    def _cells_xy(self, crop, shape):
        i0, _, j0, _ = crop
        ii, jj = np.meshgrid(np.arange(shape[0]), np.arange(shape[1]), indexing='ij')
        return (self.grid.origin + (ii + i0 + 0.5) * self.grid.p.res,
                self.grid.origin + (jj + j0 + 0.5) * self.grid.p.res)

    def _outside(self, crop, free, occ, covered):
        """Uncovered free space connected to where we took off."""
        seed = np.zeros_like(free)
        c = self.grid.to_cell(self.takeoff[:2]) - np.array([crop[0], crop[2]])
        if not (0 <= c[0] < free.shape[0] and 0 <= c[1] < free.shape[1]):
            return seed
        seed[c[0], c[1]] = True
        return flood(seed | (free & ~covered & seed), ~covered & ~occ)

    def _half_disc(self, crop, shape, opening, radius=0.6, into=0.1):
        """Cells on the outside half of an opening (plus the gap itself)."""
        x, y = self._cells_xy(crop, shape)
        dx, dy = x - opening.center[0], y - opening.center[1]
        return (dx * dx + dy * dy < radius * radius) & (dx * opening.normal[0] + dy * opening.normal[1] < into)

    def planner(self, mode, pos, allow=None, grid=None):
        """GridPlanner for 'out' (outside the covered region), 'in' (inside it) or 'any'."""
        crop, free, occ, covered, clear = self._masks(grid)
        if mode == 'out':
            passable = clear & ~dilate(covered, 1)
        elif mode == 'in':
            # the footprint, not just the roofed part: a room whose roof isn't
            # mapped yet can be poked into. But never the open space the takeoff
            # point is in (that's outside), and real openings are fenced off below
            passable = clear & fr.footprint(covered, self.grid.p.res, self.p.frontiers.footprint_margin)
            passable &= ~dilate(self._outside(crop, free, occ, covered), 2)
            for o in self.openings:
                if not o.confirmed:
                    continue    # might be a passage into a room we haven't roofed yet
                if allow is not None and np.linalg.norm(o.center - allow.center) < 0.4:
                    continue
                passable &= ~self._half_disc(crop, free.shape, o)
            if allow is not None:
                passable |= clear & self._half_disc(crop, free.shape, allow, radius=1.2, into=1.0)
        else:
            passable = clear
        # the drone's own neighbourhood is always usable if clear
        x, y = self._cells_xy(crop, free.shape)
        passable |= clear & ((x - pos[0]) ** 2 + (y - pos[1]) ** 2 < 0.3 ** 2)
        pl = GridPlanner(passable, (crop[0], crop[2]), self.grid.origin, self.grid.p.res)
        pl.start_from(pos[:2], snap=0.5)
        return pl

    # ── motion ──────────────────────────────────────────────────────────────
    def _z_rate(self, pos, z):
        return float(np.clip(self.p.kp_z * (z - pos[2]), -self.p.v_z, self.p.v_z))

    def _walls(self, pos, radius):
        """(min distance to an occupied cell, repulsion vector) within radius."""
        g = self.grid
        r = int(math.ceil(radius / g.p.res))
        ci, cj = g.to_cell(pos[:2])
        i0, i1, j0, j1 = max(ci - r, 0), min(ci + r + 1, g.n), max(cj - r, 0), min(cj + r + 1, g.n)
        cells = np.argwhere(g.occupied()[i0:i1, j0:j1]) + np.array([i0, j0])
        if not len(cells):
            return np.inf, np.zeros(2)
        d = pos[:2] - g.cell_center(cells)
        dist = np.linalg.norm(d, axis=1)
        m = dist < radius
        if not m.any():
            return float(dist.min()), np.zeros(2)
        w = (radius - dist[m]) / radius
        push = (d[m] / np.maximum(dist[m], 1e-3)[:, None] * w[:, None]).sum(axis=0)
        n = np.linalg.norm(push)
        push = push / n * min(n, 1.0) if n > 0 else push
        return float(dist.min()), push * self.p.repulse_gain

    def _velocity(self, v_xy, pos, z, cap):
        dmin, push = self._walls(pos, self.p.wall_slow)
        v = np.asarray(v_xy, dtype=float)
        if dmin < self.p.wall_slow:
            v = v * max(0.4, dmin / self.p.wall_slow)
        s = np.linalg.norm(v)
        if s > 1e-6:
            # slide along the wall instead of being pushed back off it: in a
            # 0.6 m doorway the jambs push straight back down the path, and the
            # drone sat in front of the gap for a minute and a half
            d = v / s
            against = float(push @ d)
            if against < 0:
                push = push - against * d
        v = v + push
        s = np.linalg.norm(v)
        if s > cap:
            v = v / s * cap
        return Command('velocity', velocity=np.array([v[0], v[1], self._z_rate(pos, z)]))

    def _hover(self, pos, xy, z, cap=None):
        v = self.p.kp_xy * (np.asarray(xy) - pos[:2])
        return self._velocity(v, pos, z, cap or self.p.v_out)

    def _follow(self, pos, path, z, cap):
        """Pure pursuit on a polyline; slows to a stop at its end."""
        pts = np.asarray(path)
        seg = np.diff(pts, axis=0)
        if len(pts) < 2:
            return self._hover(pos, pts[-1], z, cap)
        # closest point on the polyline, then walk lookahead along it
        best_k, best_s, best_d = 0, 0.0, np.inf
        for k, (a, s) in enumerate(zip(pts[:-1], seg)):
            L2 = float(s @ s) or 1e-9
            u = float(np.clip((pos[:2] - a) @ s / L2, 0, 1))
            d = np.linalg.norm(a + u * s - pos[:2])
            if d < best_d:
                best_k, best_s, best_d = k, u, d
        left = self.p.lookahead
        k, u = best_k, best_s
        carrot = pts[-1]
        while k < len(seg):
            L = np.linalg.norm(seg[k])
            rest = (1 - u) * L
            if rest >= left:
                carrot = pts[k] + seg[k] * (u + left / max(L, 1e-9))
                break
            left -= rest
            k, u = k + 1, 0.0
        to_goal = np.linalg.norm(pts[-1] - pos[:2])
        d = carrot - pos[:2]
        n = np.linalg.norm(d)
        speed = min(cap, 0.8 * to_goal + 0.03)
        v = d / n * speed if n > 1e-6 else np.zeros(2)
        return self._velocity(v, pos, z, cap)

    def _path_blocked(self, pl, path, pos, ahead=1.5):
        if path is None:
            return True
        walked = 0.0
        prev = pos[:2]
        for q in path:
            c = pl.cell(q)
            h, w = pl.blocked.shape
            if walked > 0.25 and 0 <= c[0] < h and 0 <= c[1] < w and pl.blocked[c]:
                return True
            walked += np.linalg.norm(q - prev)
            prev = q
            if walked > ahead:
                break
        return False

    def _go_to(self, pos, goal, mode, z, cap, allow=None, grid=None):
        """Replanning path follower. Returns (Command, arrived, planned_ok)."""
        if np.linalg.norm(goal - pos[:2]) < self.p.goal_tol:
            return self._hover(pos, goal, z, cap), True, True
        if self.t - self._t_plan >= self.p.replan_s or self.path is None:
            self._t_plan = self.t
            pl = self.planner(mode, pos, allow, grid)
            path = pl.path_to(goal, snap=0.4)
            if path is not None:
                path[-1] = goal if np.linalg.norm(path[-1] - goal) < 0.4 else path[-1]
                self.path = np.vstack([pos[:2], path])
            elif self.path is not None and not self._path_blocked(pl, self.path, pos):
                pass    # keep the old one while it's still clear
            else:
                self.path = None
        if self.path is None:
            return self._hover(pos, pos[:2], z, cap), False, False
        return self._follow(pos, self.path, z, cap), False, True

    # ── phases ──────────────────────────────────────────────────────────────
    def _takeoff(self, pos):
        z = self.p.flight_z + self.p.survey_rise
        if pos[2] >= z - 0.1:
            self.go('SURVEY')
        return Command('takeoff', z=z)

    def _survey(self, pos):
        hi = self.p.flight_z + self.p.survey_rise
        if self.elapsed() < self.p.survey_hold:
            return self._hover(pos, self.takeoff[:2], hi)
        if abs(pos[2] - self.p.flight_z) < 0.06 and self.elapsed() > 2 * self.p.survey_hold:
            self.go('FIND_ENTRANCE')
        return self._hover(pos, self.takeoff[:2], self.p.flight_z)

    def _find_entrance(self, pos):
        choice = self._choose_entrance(pos)
        if choice is not None:
            self.entrance = choice
            self.say(f'entrance {choice.kind} at {np.round(choice.center, 2)} width {choice.width:.2f}')
            self.go('ALIGN')
            return self._hover(pos, pos[:2], self.p.flight_z)
        el, look = self.elapsed(), self.p.find_look
        low = max(self.p.flight_z - self.p.vantage_drop, self.takeoff[2] + self.p.min_hover_z)
        if el < look:
            return self._hover(pos, self.takeoff[:2], self.p.flight_z)
        if el < 2 * look:
            return self._hover(pos, self.takeoff[:2], low)
        if el < 3 * look:
            # look at the nearest undecided gap from a bit further out
            cands = sorted([o for o in self.openings if o.kind == 'unknown'],
                           key=lambda o: np.linalg.norm(o.center - self.takeoff[:2]))
            if cands:
                cmd, _, ok = self._go_to(pos, cands[0].outside(1.2), 'out', low, self.p.v_in)
                return cmd
            return self._hover(pos, self.takeoff[:2], low)
        if self.p.use_fallback:
            pred = self._fallback()
            if pred is not None:
                o = gf.match(pred['entrance'], self.openings, self.p.fallback, wall_span=1.0,
                             skip_doors=False)
                self.entrance = o if o is not None else pred['entrance']
                self.fallback_used.append('entrance')
                self.say('FALLBACK: no window confirmed, entrance from the rulebook box: '
                         f'{np.round(self.entrance.center, 2)} ({"detected gap" if o else "predicted"})')
                self.go('ALIGN')
                return self._hover(pos, pos[:2], self.p.flight_z)
        self.go('LAND', 'no entrance found')
        return Command('hold')

    def _reachable_out(self, pos, o):
        pl = self.planner('out', pos)
        return pl.path_to(o.outside(self.p.align_dist), snap=0.35) is not None

    def _choose_entrance(self, pos):
        wins = [o for o in self.openings
                if o.kind == 'window' and o.confirmed and self._reachable_out(pos, o)]
        if not wins:
            return None
        wins.sort(key=lambda o: np.linalg.norm(o.center - self.takeoff[:2]))
        if len(wins) > 1:
            d0 = np.linalg.norm(wins[0].center - self.takeoff[:2])
            d1 = np.linalg.norm(wins[1].center - self.takeoff[:2])
            if d1 - d0 < 0.5 and self.p.use_fallback:
                pred = self._fallback()
                o = gf.match(pred['entrance'], wins, self.p.fallback, wall_span=1.0) if pred else None
                if o is not None:
                    self.fallback_used.append('entrance-tiebreak')
                    self.say('FALLBACK: two windows equally near, the rulebook box picked one')
                    return o
        return wins[0]

    def _fallback(self):
        g = self.grid
        # the covered region, not the raw roof layer: that one also holds the
        # arena's wall tops, and the box would fit itself to the arena
        if self.covered is not None and self.covered.any():
            i0, _, j0, _ = self.crop
            roofed = self.covered & ~g.occupied()[self.crop[0]:self.crop[1],
                                                  self.crop[2]:self.crop[3]]
            ij = np.argwhere(roofed) + np.array([i0, j0])
        else:
            ij = np.argwhere(g.roof >= self.p.openings.min_roof_hits)
        if not len(ij):
            return None

        def free_score(pts):
            c = g.to_cell(np.asarray(pts))
            ok = g.inside(c)
            return int(g.free()[c[ok, 0], c[ok, 1]].sum())
        box = gf.fit_box(g.cell_center(ij), self.takeoff[:2], self.p.fallback, free_score)
        return None if box is None else gf.predictions(box, self.p.fallback)

    def _window_z(self, o):
        pts = np.concatenate(list(self.buffer)) if self.buffer else None
        bot, top = window_extent(pts, o, self.p.flight_z)
        if bot is not None and top is not None and top - bot > 0.4:
            z = (bot + top) / 2
            if abs(z - self.p.flight_z) <= self.p.window_z_band:
                return z
        return self.p.flight_z

    def _refresh(self, o, tol=0.35):
        """Track a chosen opening with the latest detections."""
        if o.kind == 'predicted':
            # only a guess at the wall: switch to a real gap on it as soon as one shows
            q = gf.match(o, self.openings, self.p.fallback, wall_span=1.0, skip_doors=False)
            if q is not None:
                self.say(f'predicted opening replaced by the gap at {np.round(q.center, 2)}')
                o.center, o.normal, o.width, o.kind = q.center.copy(), q.normal.copy(), q.width, q.kind
                o.confirmed = q.confirmed
            return o
        best = None
        for q in self.openings:
            if np.linalg.norm(q.center - o.center) < tol and np.dot(q.normal, o.normal) > 0.8:
                if best is None or np.linalg.norm(q.center - o.center) < np.linalg.norm(best.center - o.center):
                    best = q
        if best is not None:
            o.center = 0.7 * o.center + 0.3 * best.center
            o.width = best.width
            o.confirmed = best.confirmed
            if best.kind != 'unknown':
                o.kind = best.kind if o.kind != 'predicted' else o.kind
        return o

    def _align(self, pos):
        o = self._refresh(self.entrance)
        goal = o.outside(self.p.align_dist)
        cmd, arrived, ok = self._go_to(pos, goal, 'out', self.p.flight_z, self.p.v_out)
        if arrived:
            self.pass_z = self._window_z(o)
            self.go('ALIGN_HOLD', f'pass height {self.pass_z:.2f}')
        elif not ok and self.elapsed() > 20.0:
            self.go('LAND', 'no way to the entrance')
        return cmd

    def _align_hold(self, pos):
        o = self._refresh(self.entrance)
        goal = o.outside(self.p.align_dist)
        if self.elapsed() > 2.0 and abs(pos[2] - self.pass_z) < 0.06 and np.linalg.norm(goal - pos[:2]) < 0.1:
            self.go('ENTER')
        return self._hover(pos, goal, self.pass_z, self.p.v_in)

    def _cross(self, pos, o, sign, depth_goal, next_phase, back_phase):
        """Crawl through an opening along sign * normal until depth_goal past it."""
        o = self._refresh(o)
        n, t = o.normal * sign, o.tangent
        rel = pos[:2] - o.center
        depth, lateral = rel @ n, rel @ t
        if self.elapsed() > 3.0 * (self.p.align_dist + depth_goal) / self.p.v_crawl:
            self._retries += 1
            self.say(f'crossing is taking too long, backing off (retry {self._retries})')
            if self._retries > 3:
                self.go('LAND', 'could not pass the opening')
                return Command('hold')
            self.go(back_phase)
            return self._hover(pos, pos[:2], self.pass_z, self.p.v_crawl)
        if depth >= depth_goal:
            self.go(next_phase)
            return self._hover(pos, pos[:2], self.pass_z, self.p.v_crawl)
        v = n * self.p.v_crawl - t * np.clip(1.2 * lateral, -0.1, 0.1)
        # something right in front of us (the gap closed up, or we're off it)
        if -0.6 < depth < 0.2 and self._blocked_ahead(pos, n):
            if self._blocked_since is None:
                self._blocked_since = self.t
            if self.t - self._blocked_since > 4.0:
                self._retries += 1
                self.say(f'gap looks blocked, backing off (retry {self._retries})')
                if self._retries > 3:
                    self.go('LAND', 'could not pass the opening')
                    return Command('hold')
                self.go(back_phase)
            return self._hover(pos, o.center - n * max(-depth, 0.5), self.pass_z, self.p.v_crawl)
        self._blocked_since = None
        return self._velocity(v, pos, self.pass_z, self.p.v_crawl)

    def _blocked_ahead(self, pos, n):
        g = self.grid
        t = np.array([-n[1], n[0]])
        pts = [pos[:2] + n * a + t * b for a in (0.2, 0.28) for b in (-0.06, 0.0, 0.06)]
        c = g.to_cell(np.array(pts))
        return bool(g.occupied()[c[:, 0], c[:, 1]].sum() >= 2)

    def _enter(self, pos):
        cmd = self._cross(pos, self.entrance, 1.0, self.p.enter_depth, 'SINK', 'ALIGN')
        if self.phase == 'SINK':
            self.entry_point = self.entrance.center.copy()
        return cmd

    def _sink(self, pos):
        """Settle onto the interior band before exploring: the passages inside
        can be a lot lower than the window we came through."""
        z = self.inside_z
        if abs(pos[2] - z) < 0.08 or self.elapsed() > 20.0:
            self.go('EXPLORE', f'at the interior band {z:.2f}')
            self._t_explore = self.t
        return self._hover(pos, pos[:2], z, self.p.v_crawl)

    def _shift(self, pos):
        """Change interior band in place, then carry on where we left off."""
        z = self.inside_z
        if abs(pos[2] - z) < 0.08 or self.elapsed() > 15.0:
            self.go(self._shift_back)
        return self._hover(pos, pos[:2], z, self.p.v_crawl)

    def _to_band(self, k, back):
        self.ib = k
        self._shift_back = back
        self.say(f'switching to the interior band at z {self.inside_z:.2f}')
        self.go('SHIFT')

    def _unexplored_target(self, pos):
        """((target xy, goal xy)) for the nearest bit of footprint no band has
        seen, and the reachable cell closest to it, or None."""
        crop, free, occ, covered, clear = self._masks(self.igrid)
        i0, i1, j0, j1 = crop
        foot = covered     # the roofed footprint itself, not the margin round it
        seen = self.grid.observed[i0:i1, j0:j1].copy()
        for g in self.bands:
            seen |= g.observed[i0:i1, j0:j1]
        un = foot & ~seen & ~occ
        if not un.any():
            return None
        x, y = self._cells_xy(crop, free.shape)
        xy = np.stack([x[un], y[un]], axis=1)
        for q in self._probed:
            xy = xy[np.linalg.norm(xy - q, axis=1) > self.p.probe_radius]
            if not len(xy):
                return None
        target = xy[np.argmin(np.linalg.norm(xy - pos[:2], axis=1))]
        # the closest place we can actually get to, on any band
        best = None
        for k in range(len(self.bands) or 1):
            pl = self.planner('in', pos, grid=self.bands[k] if self.bands else None)
            if pl.cost is None:
                continue
            cells = np.argwhere(np.isfinite(pl.cost))
            if not len(cells):
                continue
            q = self.grid.origin + (cells + np.array([i0, j0]) + 0.5) * self.grid.p.res
            d = np.linalg.norm(q - target, axis=1)
            m = int(np.argmin(d))
            if best is None or d[m] < best[0]:
                best = (d[m], q[m], k)
        if best is None:
            return None
        if best[2] != self.ib:
            self.ib = best[2]
        return target, best[1]

    def _probe(self, pos):
        """Fly to the nearest reachable point to unexplored footprint, then look
        at it from every band before deciding the maze is done."""
        target, goal = self.probe_target, self.probe_goal
        cmd, arrived, ok = self._go_to(pos, goal, 'in', self.inside_z, self.p.v_in,
                                       grid=self.igrid)
        if arrived or not ok or self.elapsed() > self.p.probe_s:
            self._probed.append(target)
            self._ib_sweep = 0
            self.go('SWEEP')
        return cmd

    def _sweep(self, pos):
        """Hover here at each band in turn, so a gap at any height gets seen."""
        while self._ib_sweep < len(self.bands) and not self._cell_clear(self.bands[self._ib_sweep], pos):
            self._ib_sweep += 1         # no room at that height right here
        if self._ib_sweep >= len(self.bands):
            self.go('EXPLORE', 'looked from every band')
            return self._hover(pos, pos[:2], self.inside_z, self.p.v_in)
        self.ib = self._ib_sweep
        z = self.inside_z
        if self.elapsed() > self.p.sweep_hold and abs(pos[2] - z) < 0.12:
            self._ib_sweep += 1
            self.t_phase = self.t
        return self._hover(pos, pos[:2], z, self.p.v_crawl)

    def _frontiers_on(self, k, pos):
        """(planner, reachable frontiers) for interior band k."""
        g = self.bands[k] if self.bands else self.grid
        pl = self.planner('in', pos, grid=g)
        if pl.cost is None:
            return pl, []
        fs, _ = fr.find_frontiers(g, self.crop, self.covered, pl.cost, self.p.frontiers, pl,
                                  self._ignore, self._bad_views)
        return pl, fs

    def _cell_clear(self, grid, pos):
        """Is the drone's own cell free of obstacles on that band?"""
        crop, free, occ, covered, clear = self._masks(grid)
        c = self.grid.to_cell(pos[:2]) - np.array([crop[0], crop[2]])
        if not (0 <= c[0] < clear.shape[0] and 0 <= c[1] < clear.shape[1]):
            return False
        return bool(clear[c[0], c[1]])

    def _band_with_frontiers(self, pos):
        """Another band with somewhere to explore, that we can rise or sink into."""
        for k in range(len(self.bands)):
            if k == self.ib or not self._cell_clear(self.bands[k], pos):
                continue
            _, fs = self._frontiers_on(k, pos)
            if fs:
                return k
        return None

    def _explore(self, pos):
        self._track_exit()
        z = self.inside_z
        if self.t - self._t_explore > self.p.explore_budget:
            self.go('GO_EXIT', 'exploration budget spent')
            return self._hover(pos, pos[:2], z, self.p.v_in)
        if self.t - self._t_plan >= self.p.replan_s or self.path is None:
            self._t_plan = self.t
            pl, fs = self._frontiers_on(self.ib, pos)
            self.frontiers = fs
            if not fs:
                # nothing left here: the way on may be a band up or down
                k = self._band_with_frontiers(pos)
                if k is not None:
                    self._to_band(k, 'EXPLORE')
                    return self._hover(pos, pos[:2], self.inside_z, self.p.v_crawl)
                # nothing to explore anywhere, but the footprint is not covered:
                # the way on is a gap none of the bands has looked at yet
                target = self._unexplored_target(pos)
                if target is not None:
                    self.probe_target, self.probe_goal = target
                    self.say(f'nothing left to explore; going to look at {np.round(target[0], 2)}')
                    self.go('PROBE')
                    return self._hover(pos, pos[:2], z, self.p.v_in)
                self._no_frontier += 1
                if self._no_frontier >= self.p.give_up_cycles:
                    self.go('GO_EXIT', 'no reachable frontier left on any band')
                return self._hover(pos, pos[:2], z, self.p.v_in)
            self._no_frontier = 0
            best = fs[0]
            if self.frontier_goal is not None:
                # stick with the current goal unless something much cheaper shows up
                same = [f for f in fs if np.linalg.norm(f.goal - self.frontier_goal) < 0.5]
                if same and same[0].cost < best.cost + 1.0:
                    best = same[0]
            if self.frontier_goal is None or np.linalg.norm(best.goal - self.frontier_goal) > 0.5:
                self._goal_since = self.t
            self.frontier_goal = best.goal
            self._frontier_centroid = best.centroid
            self.path = pl.path_to_cell(best.goal_cell)
            if self.path is not None:
                self.path = np.vstack([pos[:2], self.path])
        if self.path is None or self.frontier_goal is None:
            return self._hover(pos, pos[:2], z, self.p.v_in)
        close = np.linalg.norm(self.frontier_goal - pos[:2]) < self.p.goal_tol + 0.1
        if (close and self.t - self._goal_since > 3.0) or self.t - self._goal_since > self.p.stuck_s:
            # we're there (or can't get there) and it's still a frontier: try
            # another view of it, and after a few, drop the frontier
            self._bad_views.append(self.frontier_goal.copy())
            c = self._frontier_centroid
            tries = sum(np.linalg.norm(q - c) < 1.0 for q in self._view_of)
            self._view_of.append(c)
            if tries + 1 >= self.p.view_tries:
                self._ignore.append(c)
                self.say(f'frontier at {np.round(c, 2)} given up')
            self.frontier_goal, self.path = None, None
            return self._hover(pos, pos[:2], z, self.p.v_in)
        return self._follow(pos, self.path, z, self.p.v_in)

    def _track_exit(self, locked=False):
        if self.entry_point is None:
            return
        best = self.exit
        if locked and best is not None:
            return
        # a window beats an undecided gap beats something that looked like a door
        rank_of = lambda o: (o.confirmed, o.kind == 'window', o.kind != 'door', o.confidence)  # noqa: E731
        for o in self.openings:
            if np.linalg.norm(o.center - self.entry_point) <= self.p.exit_min_dist:
                continue
            if any(np.linalg.norm(o.center - b) < 0.5 for b in self._bad_exits):
                continue
            same = best is not None and np.linalg.norm(best.center - o.center) < 0.4
            # only a better kind of evidence switches exits, not a confidence wobble
            if best is None or (same and rank_of(o) >= rank_of(best)) \
                    or rank_of(o)[:3] > rank_of(best)[:3]:
                if best is None or np.linalg.norm(best.center - o.center) > 0.4:
                    self.say(f'exit candidate {o.kind} at {np.round(o.center, 2)}'
                             + ('' if o.confirmed else ' (outside not seen yet)'))
                best = o
        self.exit = best

    def _go_exit(self, pos):
        self._track_exit(locked=self.elapsed() > 1.0)
        if self.exit is None:
            pred = self._fallback() if self.p.use_fallback else None
            if pred is None:
                self.go('LAND', 'no exit found')
                return Command('hold')
            self.exit = pred['exit']
            self.fallback_used.append('exit')
            self.say(f'FALLBACK: no exit seen, using the rulebook one at {np.round(self.exit.center, 2)}')
        o = self._refresh(self.exit)
        goal = self._inside_point(o)
        cmd, arrived, ok = self._go_to(pos, goal, 'in', self.inside_z, self.p.v_in, allow=o,
                                       grid=self.igrid)
        if arrived:
            self.pass_z = self._window_z(o)
            self.go('ALIGN_EXIT', f'pass height {self.pass_z:.2f}')
        elif not ok and self.elapsed() > 8.0:
            # the way there may be on another band
            k = self._band_reaching(pos, goal)
            if k is not None:
                self._to_band(k, 'GO_EXIT')
            elif self.elapsed() > 30.0:
                self.go('LAND', 'no way to the exit')
        return cmd

    def _band_reaching(self, pos, goal):
        for k in range(len(self.bands)):
            if k == self.ib or not self._cell_clear(self.bands[k], pos):
                continue
            pl = self.planner('in', pos, grid=self.bands[k])
            if pl.cost is not None and pl.path_to(goal, snap=0.4) is not None:
                return k
        return None

    def _inside_point(self, o):
        """A clear spot just inside an opening, on the band we fly inside."""
        crop, free, occ, covered, clear = self._masks(self.igrid)
        # nearest clear spot first: the drone climbs to the window height there,
        # and right under the window is the part we know is open to the top
        for d in np.arange(0.35, self.p.align_dist + 0.01, 0.1):
            q = o.inside(d)
            c = self.grid.to_cell(q) - np.array([crop[0], crop[2]])
            if 0 <= c[0] < clear.shape[0] and 0 <= c[1] < clear.shape[1] and clear[c[0], c[1]]:
                return q
        return o.inside(0.4)

    def _align_exit(self, pos):
        o = self._refresh(self.exit)
        goal = self._inside_point(o)
        settled = self.elapsed() > 2.0 and abs(pos[2] - self.pass_z) < 0.06 \
            and np.linalg.norm(goal - pos[:2]) < 0.12
        if settled and (o.confirmed or o.kind == 'predicted'):
            self.go('PASS_EXIT')
        elif settled and self.elapsed() > 6.0:
            # right in front of it and still no open space beyond: not an exit
            self.say(f'exit candidate at {np.round(o.center, 2)} leads nowhere, dropping it')
            self._bad_exits.append(o.center.copy())
            self.exit = None
            self.go('GO_EXIT')
        return self._hover(pos, goal, self.pass_z, self.p.v_in)

    def _pass_exit(self, pos):
        cmd = self._cross(pos, self.exit, -1.0, self.p.exit_out, 'GO_LAND', 'GO_EXIT')
        if self.phase == 'GO_EXIT':
            # couldn't get through: try the next candidate rather than the same gap again
            self.say(f'exit at {np.round(self.exit.center, 2)} dropped')
            self._bad_exits.append(self.exit.center.copy())
            self.exit = None
            self._retries = 0
        return cmd

    def _inside_points(self):
        """Points surely inside the arena: the structure footprint and the takeoff spot."""
        pts = [self.takeoff[:2][None, :]]
        if self.covered is not None and self.covered.any():
            ij = np.argwhere(self.covered) + np.array([self.crop[0], self.crop[2]])
            pts.append(self.grid.cell_center(ij))
        return np.vstack(pts)

    def _landing_point(self):
        self.arena = arena_from_grid(self.grid, self._inside_points()) or self.arena
        if self.arena is not None:
            return self.arena.center
        if self.p.use_fallback:
            pred = self._fallback()
            if pred is not None:
                if 'landing' not in self.fallback_used:
                    self.fallback_used.append('landing')
                    self.say('FALLBACK: arena not seen, landing point from the rulebook box')
                return pred['landing']
        return None

    def _go_land(self, pos):
        if self.landing is None or int(self.elapsed() * 10) % 20 == 0:
            lp = self._landing_point()
            if lp is None:
                self.go('LAND', 'no landing point, landing here')
                return Command('land')
            self.landing = np.asarray(lp, dtype=float)
        cmd, arrived, ok = self._go_to(pos, self.landing, 'out', self.p.flight_z, self.p.v_out)
        if arrived:
            self.go('LAND', f'at {np.round(self.landing, 2)}')
        elif not ok and self.elapsed() > 5.0:
            self.go('CLIMB', 'no way round outside')
        return cmd

    def _climb(self, pos):
        top = (self.roof_top if self.roof_top is not None else self.p.flight_z + 0.5) + self.p.climb_margin
        if pos[2] < top - 0.05 and np.linalg.norm(self.landing - pos[:2]) > self.p.goal_tol:
            return self._hover(pos, pos[:2], top, self.p.v_in)
        if np.linalg.norm(self.landing - pos[:2]) > self.p.goal_tol:
            return self._hover(pos, self.landing, top, self.p.v_out)
        self.go('LAND', 'over the landing point')
        return Command('land')

    def _land(self, pos):
        self.done = True
        return Command('land')
