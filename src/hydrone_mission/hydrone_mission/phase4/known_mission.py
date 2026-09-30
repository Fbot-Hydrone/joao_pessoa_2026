"""Phase 4 on the rulebook map: fly the maze by its walls, not by LIO.

The route (route.py) says which room to see next and which passage to take.
Inside, the pose comes from the beam localizer against the known map (the
stack's pose mux is asked for 'map'); outside, from LIO re-anchored to the
map ('odom'). Every velocity goes through the beam guard.

TAKEOFF    climb to out_z over the pad
TO_ENTRY   fly outside to the entrance's align point
ENTRY_HOLD hover there until the map pose is flying (or give up waiting)
GOTO       fly to a point inside a room (planner on the known raster)
LOOK       turn around once (camera sees every wall), or just hold
ALIGN      stand in front of a passage at its own height (holes are low,
           windows high)
VERIFY     the beam through it, and the ones 15 deg either side, must see
           past the wall; if not, SEARCH: sweep the wall at pass height and
           centre on the longest open stretch (soft passages may sit
           elsewhere); nothing open: walled up, replan without it
CRAWL      straight through at a crawl
GO_LAND    out of the exit, back on LIO, to the landing pad; LAND
EXPLORE    the prior was wrong enough that no route is left: the generic
           explorer (fed in the background all along) takes over

Each QR code read counts once; with qr_target read, the rest of the tour is
dropped for the shortest way out.
"""
from dataclasses import dataclass, field
import time

import numpy as np

from hydrone_confined.core.guard import GuardParams
from hydrone_confined.core.morph import dilate
from hydrone_confined.core.navigator import BeamObstacles, Navigator, NavParams
from hydrone_confined.core.planner import GridPlanner
from hydrone_mission.phase4.route import RoomGraph, exit_legs, plan_tour


# Interior bands for the explorer when it takes over (m above the floor): low
# holes are passed at ~0.4, and windows between rooms (sill ~0.7) need the
# body's bottom above the sill: 0.9, not the explorer's default 0.75, which
# flew into a sill in the handover test.
HANDOVER_BANDS = (0.40, 0.90)


@dataclass
class KnownParams:
    out_z: float = 1.1              # above the floor, outside (window centres)
    cruise_z: float = 0.75          # inside, between passages
    align_dist: float = 0.45        # stand-off from a passage's wall line
    exit_out: float = 0.9           # past the exit window before heading to land
    entry_out: float = 0.9          # align point outside the entrance
    v_in: float = 0.25
    v_out: float = 0.5
    v_crawl: float = 0.15
    inflate: float = 0.20           # planning clearance from walls
    res: float = 0.05
    spin: bool = True               # turn around at each look point (camera)
    spin_rate: float = 0.8          # rad/s
    look_hold: float = 1.0          # s when not spinning
    qr_target: int = 5
    verify_s: float = 1.5
    passage_clear: float = 0.3      # forward beam must reach this past the wall line
    search_span: float = 0.6       # sweep this far either side of where the gap should be
    search_step: float = 0.2
    map_wait_s: float = 8.0         # waiting for a flyable map pose before going in
    pad_wait_s: float = 6.0         # and on the pad before taking off
    goal_tol: float = 0.12
    z_tol: float = 0.06
    goto_timeout: float = 40.0
    mission_budget: float = 560.0
    explore_fallback: bool = True
    nav: NavParams = field(default_factory=NavParams)
    guard: GuardParams = field(default_factory=GuardParams)


@dataclass
class Command:
    kind: str                                   # velocity / hold / land / takeoff
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    z: float = None
    yaw_rate: float = 0.0
    source: str = 'odom'                        # pose the mux should fly on
    debug: dict = field(default_factory=dict)


class KnownMission:

    def __init__(self, known_map, params=None, explore=None, logger=None):
        self.km = known_map
        self.p = params or KnownParams()
        self.graph = RoomGraph(known_map, self.p.cruise_z)
        self.explore = explore if self.p.explore_fallback else None
        self.obst = BeamObstacles(self.p.guard)
        self.nav = Navigator(self.p.nav, self.obst)
        self.nav.p.goal_tol = self.p.goal_tol
        self.logger = logger
        self.log = []
        self.fallback_used = []
        self.phase = 'TAKEOFF'
        self.t = 0.0
        self.t0 = None
        self.t_phase = 0.0
        self.source = 'odom'
        self.legs = []
        self.leg = None
        self.visited = set()
        self.qr_seen = []
        self.heading_out = False
        self.entrance = known_map.passages('entry')[0]
        self.exit = known_map.passages('exit')[0]
        self.landing = known_map.landing
        self.step_ms = []
        self._rasters = {}
        self._spun = 0.0
        self._last_yaw = None
        self._blocked_since = None
        self._search = []
        self._shift = {}                        # passage name -> offset found along its wall
        self._search_open = []
        self._crawl_fails = {}
        self._goal = None
        self._pos = None
        self.fix = None
        self.mode = 'odom'
        self._scan_nav = None
        self._cross = None

    # ── bookkeeping ─────────────────────────────────────────────────────────
    def say(self, msg):
        self.log.append((self.t, msg))
        if self.logger:
            self.logger(msg)

    def go(self, phase, why=''):
        self.say(f'{self.phase} -> {phase}' + (f' ({why})' if why else ''))
        self.phase, self.t_phase = phase, self.t
        self.nav.reset()
        self._blocked_since = None

    def elapsed(self):
        return self.t - self.t_phase

    def room(self, xy):
        r = self.km.room_at(xy)
        return r.name if r else None

    # ── planning maps ───────────────────────────────────────────────────────
    def _raster(self, z, mode):
        """Passable cells at height z: 'in' = inside the structure, 'out' = outside it."""
        key = (round(z, 1), mode)
        if key in self._rasters:
            return self._rasters[key]
        occ, origin, res = self.km.rasterize((z - 0.15, z + 0.15), res=self.p.res, margin=1.0)
        # walled up: forbidden openings (the door) never get planned through
        for p in self.km.passages('forbidden'):
            c, t = p.center, p.tangent
            for u in np.arange(-p.opening.width / 2 - 0.1, p.opening.width / 2 + 0.1, res / 2):
                q = c + t * u
                ij = np.floor((q - origin) / res).astype(int)
                if 0 <= ij[0] < occ.shape[0] and 0 <= ij[1] < occ.shape[1]:
                    occ[ij[0], ij[1]] = True
        clear = ~dilate(occ, self.p.inflate / res)
        xs = origin[0] + (np.arange(occ.shape[0]) + 0.5) * res
        ys = origin[1] + (np.arange(occ.shape[1]) + 0.5) * res
        X, Y = np.meshgrid(xs, ys, indexing='ij')
        x0, x1, y0, y1 = self.km.structure
        foot = (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
        if mode == 'in':
            passable = clear & foot
        else:
            m = self.p.inflate + 0.1
            passable = clear & ~((X >= x0 - m) & (X <= x1 + m) & (Y >= y0 - m) & (Y <= y1 + m))
        self._rasters[key] = (passable, origin, res)
        return self._rasters[key]

    def _planner(self, z, mode):
        passable, origin, res = self._raster(z, mode)

        def make(pos):
            pl = GridPlanner(passable, (0, 0), origin, res)
            # standing inside the inflation (next to a passage): start from the nearest clear cell
            pl.start_from(pos[:2], snap=0.5)
            return pl
        return make

    # ── the tick ────────────────────────────────────────────────────────────
    def step(self, t, pose, fix=None, qr=(), scan_nav=None, mode='odom'):
        """pose (x, y, z, yaw) in the map frame, as the mux flies it.
        fix: MapFix (beams, sectors); scan_nav: (points, origin) registered in the map frame."""
        c0 = time.perf_counter()
        self.t = t
        if self.t0 is None:
            self.t0, self.t_phase = t, t
        pos = np.asarray(pose[:3], dtype=float)
        yaw = float(pose[3])
        self._pos, self.fix, self.mode = pos, fix, mode
        if fix is not None and fix.sectors is not None:
            self.obst.update(fix.sectors, yaw)
        if self._last_yaw is not None:
            self._spun += abs((yaw - self._last_yaw + np.pi) % (2 * np.pi) - np.pi)
        self._last_yaw = yaw
        for q in qr:
            if q not in self.qr_seen:
                self.qr_seen.append(q)
                self.say(f'QR {q} read ({len(self.qr_seen)}/{self.p.qr_target})')
        if self.explore is not None and scan_nav is not None and self.phase != 'EXPLORE':
            self.explore.observe(t, (pos, yaw), scan_nav)
        if (len(self.qr_seen) >= self.p.qr_target or t - self.t0 > self.p.mission_budget) \
                and not self.heading_out and self.phase in ('GOTO', 'LOOK', 'ALIGN', 'VERIFY', 'SEARCH'):
            self._head_out('all QR codes read' if len(self.qr_seen) >= self.p.qr_target
                           else 'mission budget spent')
        self._scan_nav = scan_nav
        cmd = getattr(self, '_ph_' + self.phase.lower())(pos, yaw)
        # the map pose is wanted from the entrance on, until out of the exit;
        # whether it's healthy enough to fly on is the mux's call
        if self.phase != 'EXPLORE':         # the explorer says for itself
            self.source = 'odom' if self.phase in ('TAKEOFF', 'TO_ENTRY', 'GO_LAND', 'LAND') else 'map'
        cmd.source = self.source
        cmd.debug = self.debug()
        self.step_ms.append((time.perf_counter() - c0) * 1e3)
        return cmd

    def debug(self):
        return {'phase': self.phase, 'path': self.nav.path, 'leg': self.leg, 'legs': list(self.legs),
                'goal': self._goal, 'qr': list(self.qr_seen), 'source': self.source,
                'fallback_used': list(self.fallback_used), 'landing': self.landing}

    # ── motion helpers ──────────────────────────────────────────────────────
    def _vel(self, v, pos, z, cap, yaw_rate=0.0):
        return Command('velocity', velocity=self.nav.velocity(v, pos, z, cap), yaw_rate=yaw_rate)

    def _hover(self, pos, xy, z, cap, yaw_rate=0.0):
        return Command('velocity', velocity=self.nav.hover(pos, xy, z, cap), yaw_rate=yaw_rate)

    def _go_to(self, pos, goal, z, cap, mode):
        v, arrived, ok = self.nav.go_to(self.t, pos, np.asarray(goal, float), z, cap,
                                        self._planner(z, mode))
        return Command('velocity', velocity=v), arrived and abs(pos[2] - z) < 0.1, ok

    # ── legs ────────────────────────────────────────────────────────────────
    def _next_leg(self, pos):
        if not self.legs:
            self.leg = ('exit', self.exit, self.graph.exit)
            self._start_cross(self.exit, self.graph.exit, None)
            return
        self.leg = self.legs.pop(0)
        kind = self.leg[0]
        if kind == 'look':
            self._goal = self.leg[2]
            self.go('GOTO', f'look point in {self.leg[1]}')
        elif kind == 'move':
            self._goal = self.leg[2]
            self.go('GOTO', f'into {self.leg[1]}')
        elif kind == 'cross':
            self._start_cross(self.leg[1], self.leg[2], self.leg[3])

    def _start_cross(self, passage, a, b):
        self._cross = {'p': passage, 'from': a, 'to': b}
        self.go('ALIGN', f'{passage.name}: {a} -> {b or "outside"} at z {passage.opening.pass_z:.2f}')

    def _cross_frame(self):
        """(centre, direction through, tangent) of the passage being crossed."""
        c = self._cross
        p = c['p']
        center = p.center + p.tangent * self._shift.get(p.name, 0.0)
        n = p.normal
        # point from the side we're on to the other one
        ref = self.graph.rooms[c['from']].center if c['from'] else self.km.takeoff['center']
        if (center - ref) @ n < 0:
            n = -n
        return center, n, p.tangent

    def _head_out(self, why):
        here = self.room(self._pos[:2]) or (self.leg[1] if self.leg and self.leg[0] in ('look', 'move') else None)
        legs = exit_legs(self.graph, here) if here else None
        if legs is None:
            self.say(f'{why}, but no way from {here} to the exit found; carrying on')
            return
        self.heading_out = True
        self.legs = legs
        self.say(f'{why}: shortest way out from {here} ({len(legs)} legs)')
        if self.phase in ('GOTO', 'LOOK'):
            self._next_leg(self._pos)

    def _replan(self, pos, why):
        here = self._cross['from'] if self.phase in ('VERIFY', 'SEARCH', 'ALIGN') else self.room(pos[:2])
        if self.heading_out:
            legs = exit_legs(self.graph, here)
        else:
            legs = plan_tour(self.graph, here, self.visited)
        if legs is None or self.graph.exit not in self.graph.reachable(here):
            self._to_explore(pos, f'{why}; no route to the exit on the known map')
            return
        self.legs = legs
        self.say(f'{why}: replanned from {here}, {len(legs)} legs')
        self._next_leg(pos)

    def _to_explore(self, pos, why):
        if self.explore is None:
            self.fallback_used.append('land')
            self.go('LAND', why)
            return
        self.fallback_used.append('explore')
        self.say(f'FALLBACK: {why}: the generic explorer takes over')
        self.explore.take_over(self.t, self.entrance.center, self.entrance.normal)
        self.go('EXPLORE')

    # ── phases ──────────────────────────────────────────────────────────────
    def _ph_takeoff(self, pos, yaw):
        healthy = self.fix is not None and self.fix.healthy
        if not healthy and self.elapsed() < self.p.pad_wait_s:
            return Command('hold')
        if not healthy and 'takeoff-unlocalized' not in self.fallback_used:
            self.fallback_used.append('takeoff-unlocalized')
            self.say('map pose not healthy on the pad, taking off on odometry')
        if pos[2] >= self.p.out_z - 0.1:
            self.go('TO_ENTRY')
        return Command('takeoff', z=self.p.out_z)

    def _ph_to_entry(self, pos, yaw):
        c, n = self.entrance.center, self.entrance.normal
        # the normal that points out of the structure, toward the pad
        if (self.km.takeoff['center'] - c) @ n < 0:
            n = -n
        goal = c + n * self.p.entry_out
        cmd, arrived, ok = self._go_to(pos, goal, self.p.out_z, self.p.v_out, 'out')
        if arrived:
            self.go('ENTRY_HOLD')
        elif not ok and self.elapsed() > 20.0:
            self.go('LAND', 'no way to the entrance')
        return cmd

    def _ph_entry_hold(self, pos, yaw):
        self.source = 'map'
        c, n = self.entrance.center, self.entrance.normal
        if (self.km.takeoff['center'] - c) @ n < 0:
            n = -n
        goal = c + n * self.p.entry_out
        if self.mode == 'map' and self.elapsed() > 1.5:
            self.leg = ('enter', self.entrance, self.graph.entry)
            self.legs = plan_tour(self.graph, self.graph.entry)
            self.say(f'route: {len(self.legs)} legs, ' + ' '.join(
                l[0][0] + ':' + (l[1] if isinstance(l[1], str) else l[1].name) for l in self.legs))
            self._start_cross(self.entrance, None, self.graph.entry)
        elif self.elapsed() > self.p.map_wait_s:
            self.fallback_used.append('enter-on-odom')
            self.say('FALLBACK: map pose never became flyable here, crossing on odometry '
                     '(the mux joins the map pose once it is)')
            self.leg = ('enter', self.entrance, self.graph.entry)
            self.legs = plan_tour(self.graph, self.graph.entry)
            self._start_cross(self.entrance, None, self.graph.entry)
        return self._hover(pos, goal, self.p.out_z, self.p.v_in)

    def _ph_goto(self, pos, yaw):
        z = self.p.cruise_z
        cmd, arrived, ok = self._go_to(pos, self._goal, z, self.p.v_in, 'in')
        if arrived:
            if self.leg and self.leg[0] == 'look':
                self._spun = 0.0
                self.visited.add(self.leg[1])
                self.go('LOOK')
            else:
                if self.leg and self.leg[0] == 'move':
                    self.visited.add(self.leg[1])
                self._next_leg(pos)
        elif self.elapsed() > self.p.goto_timeout:
            self.say(f'could not reach {np.round(self._goal, 2)}, skipping it')
            self._next_leg(pos)
        return cmd

    def _ph_look(self, pos, yaw):
        z = self.p.cruise_z
        if self.p.spin:
            if self._spun >= 2 * np.pi:
                self._next_leg(pos)
                return self._hover(pos, self._goal, z, self.p.v_in)
            return self._hover(pos, self._goal, z, self.p.v_in, yaw_rate=self.p.spin_rate)
        if self.elapsed() > self.p.look_hold:
            self._next_leg(pos)
        return self._hover(pos, self._goal, z, self.p.v_in)

    def _ph_align(self, pos, yaw):
        center, n, t = self._cross_frame()
        z = self._cross['p'].opening.pass_z
        a = center - n * self.p.align_dist
        outside = self._cross['from'] is None
        mode = 'out' if outside else 'in'
        far = np.linalg.norm(a - pos[:2]) > 0.25
        if far:
            # get there at a safe height first, then change height in place
            cz = self.p.out_z if outside else self.p.cruise_z
            cmd, arrived, ok = self._go_to(pos, a, cz, self.p.v_in, mode)
            return cmd
        cmd = self._hover(pos, a, z, self.p.v_crawl)
        if np.linalg.norm(a - pos[:2]) < self.p.goal_tol and abs(pos[2] - z) < self.p.z_tol \
                and self.elapsed() > 1.0:
            self.go('VERIFY')
        elif self.elapsed() > 30.0:
            self._replan(pos, f'could not line up with {self._cross["p"].name}')
        return cmd

    def _beam_along(self, yaw, d):
        """Range of the beam most nearly along world direction d (unit 2-vector)."""
        b = self.fix.beams if self.fix is not None else None
        if b is None:
            return None
        want = np.arctan2(d[1], d[0]) - yaw
        k = int(np.argmin(np.abs((b.angles - want + np.pi) % (2 * np.pi) - np.pi)))
        return float(b.ranges[k])

    def _passage_open(self, pos, yaw, gap_center, n):
        """(open, centre range): the beam through the gap and the ones 15 deg to
        either side all reach past the wall line. One beam alone passes along
        the edge of a gap the body won't fit through."""
        depth = (gap_center - pos[:2]) @ n          # to the wall line
        r0 = None
        ok = True
        for da in (0.0, np.deg2rad(15.0), -np.deg2rad(15.0)):
            c, s_ = np.cos(da), np.sin(da)
            d = np.array([c * n[0] - s_ * n[1], s_ * n[0] + c * n[1]])
            r = self._beam_along(yaw, d)
            if da == 0.0:
                r0 = r
            if r is None or r <= depth / np.cos(da) + self.p.passage_clear:
                ok = False
        return ok, r0

    def _start_search(self, name):
        base = self._shift.get(name, 0.0)
        k = int(round(self.p.search_span / self.p.search_step))
        offs = [base + i * self.p.search_step for i in range(-k, k + 1)]
        # only stand-off points in the room we're in (or outside, for the
        # entrance): past the end of the room is through a wall
        p = self._cross['p']
        _, n, t = self._cross_frame()
        room = self.graph.rooms.get(self._cross['from']) if self._cross['from'] else None
        keep = []
        for o in offs:
            a = p.center + t * o - n * self.p.align_dist
            inside = room.contains(a, margin=-0.12) if room is not None else not self.km.inside_structure(a)
            if inside:
                keep.append(o)
        self._search = keep
        self._search_open = []

    def _ph_verify(self, pos, yaw):
        center, n, t = self._cross_frame()
        z = self._cross['p'].opening.pass_z
        a = center - n * self.p.align_dist
        open_, r = self._passage_open(pos, yaw, center, n)
        if open_ and self.elapsed() > 0.5:
            self.go('CRAWL', f'{self._cross["p"].name} is open (beam {r:.2f} m)')
        elif self.elapsed() > self.p.verify_s:
            self._start_search(self._cross['p'].name)
            self.say(f'{self._cross["p"].name} looks closed (beam {r if r is None else round(r, 2)} m), '
                     'sweeping along the wall')
            self.go('SEARCH')
        return self._hover(pos, a, z, self.p.v_crawl)

    def _ph_search(self, pos, yaw):
        """Sweep the wall at pass height, then centre on the longest open stretch."""
        p = self._cross['p']
        _, n, t = self._cross_frame()
        base = p.center
        if not self._search:
            opens = [o for o, ok in self._search_open if ok]
            # longest run of consecutive open offsets
            best, run = [], []
            for o, ok in self._search_open:
                run = run + [o] if ok else []
                if len(run) > len(best):
                    best = run
            if not best:
                self.graph_block(p)
                self._replan(pos, f'{p.name} is walled up')
                return self._hover(pos, pos[:2], pos[2], self.p.v_crawl)
            off = 0.5 * (best[0] + best[-1])
            if abs(off - self._shift.get(p.name, 0.0)) > 0.05:
                self.fallback_used.append(f'moved:{p.name}')
            self._shift[p.name] = off
            self.say(f'{p.name}: open from {best[0]:+.1f} to {best[-1]:+.1f} m along its wall '
                     f'({len(opens)} of {len(self._search_open)} looks), centring at {off:+.2f}')
            self.go('ALIGN')
            return self._hover(pos, pos[:2], pos[2], self.p.v_crawl)
        off = self._search[0]
        gap = base + t * off
        a = gap - n * self.p.align_dist
        z = p.opening.pass_z
        if np.linalg.norm(a - pos[:2]) > 0.25:
            # to the next look point on the planner, not straight through anything
            cmd, _, _ = self._go_to(pos, a, pos[2], self.p.v_crawl, 'out' if self._cross['from'] is None else 'in')
        else:
            cmd = self._hover(pos, a, z, self.p.v_crawl)
        there = np.linalg.norm(a - pos[:2]) < self.p.goal_tol and abs(pos[2] - z) < self.p.z_tol
        if there:
            if self._blocked_since is None:
                self._blocked_since = self.t
            if self.t - self._blocked_since > 0.8:
                self._search_open.append((off, self._passage_open(pos, yaw, gap, n)[0]))
                self._search.pop(0)
                self._blocked_since = None
        elif self.t - self.t_phase > 15.0 * (len(self._search_open) + 1):
            # can't get to this spot: count it as closed and move on
            self._search_open.append((off, False))
            self._search.pop(0)
        return cmd

    def graph_block(self, passage):
        for e in self.graph.edges:
            if e.passage is passage:
                e.blocked = True
        self.fallback_used.append(f'blocked:{passage.name}')

    def _ph_crawl(self, pos, yaw):
        center, n, t = self._cross_frame()
        z = self._cross['p'].opening.pass_z
        rel = pos[:2] - center
        depth, lateral = rel @ n, rel @ t
        goal = self.p.exit_out if self._cross['to'] is None else self.p.align_dist
        if depth >= goal:
            if self._cross['to'] is None:
                self.source = 'odom'
                self.go('GO_LAND', 'out of the maze, back on odometry')
            else:
                if self._cross['from'] is None:
                    self.say(f'inside ({self._cross["to"]})')
                self._next_leg(pos)
            return self._hover(pos, pos[:2], z, self.p.v_crawl)
        if self.elapsed() > 4.0 * (self.p.align_dist + goal) / self.p.v_crawl:
            name = self._cross['p'].name
            self._crawl_fails[name] = self._crawl_fails.get(name, 0) + 1
            if self._crawl_fails[name] >= 2:
                # open to the beam but not to the drone: narrower or off-centre
                # from the map; look for the real gap along the wall
                self._start_search(name)
                self.say(f'crossing {name} failed twice, sweeping along the wall')
                self._crawl_fails[name] = 0
                self.go('SEARCH')
            else:
                self.say(f'crossing {name} is taking too long, lining up again')
                self.go('ALIGN')
        v = n * self.p.v_crawl - t * np.clip(1.2 * lateral, -0.1, 0.1)
        return self._vel(v, pos, z, self.p.v_crawl)

    def _ph_go_land(self, pos, yaw):
        self.source = 'odom'
        cmd, arrived, ok = self._go_to(pos, self.landing, self.p.out_z, self.p.v_out, 'out')
        if arrived:
            self.go('LAND', f'over the pad at {np.round(self.landing, 2)}')
            return Command('land')
        if not ok and self.elapsed() > 10.0:
            self.go('LAND', 'no way to the landing pad, landing here')
            return Command('land')
        return cmd

    def _ph_explore(self, pos, yaw):
        cmd = self.explore.step(self.t, (pos, yaw), self._scan_nav)
        self.source = 'map' if self.explore.phase not in ('GO_LAND', 'LAND', 'CLIMB') else 'odom'
        if self.explore.landing is not None:
            self.landing = self.explore.landing
        return Command(cmd.kind, velocity=np.asarray(cmd.velocity), z=cmd.z, yaw_rate=cmd.yaw_rate)

    def _ph_land(self, pos, yaw):
        return Command('land')
