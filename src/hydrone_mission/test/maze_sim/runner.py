"""Kinematic drone + the mission loop, with the numbers the tests check."""
from dataclasses import dataclass, field

import numpy as np

from hydrone_mission.maze.mission import Mission, MissionParams
from maze_sim import lidar
from maze_sim.world import interior_cells

BODY_R = 0.165
BODY_DOWN, BODY_UP = 0.07, 0.10     # base_link to feet / top of the prop guards
LIDAR_UP = 0.1


@dataclass
class SimParams:
    dt: float = 0.1
    scan_every: int = 1             # steps per lidar sweep
    n_rays: int = 12000
    range_noise: float = 0.02
    tau: float = 0.3                # velocity response lag
    accel_noise: float = 0.05       # m/s^2 white
    pose_noise: float = 0.02        # m white on the reported pose
    pose_drift: float = 0.002       # m/sqrt(s) random walk on the reported pose
    max_time: float = 600.0
    seed: int = 1
    drop_tags: tuple = ()           # e.g. ('sill',) to hide the sills from the lidar


@dataclass
class Result:
    phases: list = field(default_factory=list)
    log: list = field(default_factory=list)
    min_clearance: float = np.inf
    min_clearance_at: tuple = None     # (t, phase, world xyz)
    crashed: bool = False
    entry_cross: np.ndarray = None      # world xy where the drone first got inside
    exit_cross: np.ndarray = None       # and where it left
    crossings: list = field(default_factory=list)   # (t, 'in' / 'out', world xy), every one
    landed_at: np.ndarray = None        # world xy
    landing_target: np.ndarray = None   # world xy the mission chose
    coverage: float = 0.0
    time: float = 0.0
    done: bool = False
    fallback_used: list = field(default_factory=list)
    scan_ms: float = 0.0
    scan_ms_max: float = 0.0
    perceive_ms: float = 0.0
    step_ms: float = 0.0                # mean whole Mission.step
    step_ms_p95: float = 0.0
    track: list = field(default_factory=list)
    mission: object = None


def clearance(world, p, landing=False):
    """Smallest gap between the drone body (disc + height band) and any box.

    While taking off or landing, what's straight under the drone is where it lands, not a crash.
    """
    dx = np.maximum(np.maximum(world.lo[:, 0] - p[0], 0), p[0] - world.hi[:, 0])
    dy = np.maximum(np.maximum(world.lo[:, 1] - p[1], 0), p[1] - world.hi[:, 1])
    dxy = np.hypot(dx, dy) - BODY_R
    dz = np.maximum(world.lo[:, 2] - (p[2] + BODY_UP), (p[2] - BODY_DOWN) - world.hi[:, 2])
    c = np.max(np.stack([dxy, dz]), axis=0)
    if landing:
        c = c[~((dx == 0) & (dy == 0) & (world.hi[:, 2] <= p[2]))]
    return float(c.min())


def inside(world, xy):
    x0, x1, y0, y1 = world.structure
    return x0 < xy[0] < x1 and y0 < xy[1] < y1


def run(world, mparams=None, sparams=None, on_step=None):
    sp = sparams or SimParams()
    rng = np.random.default_rng(sp.seed)
    m = Mission(mparams or MissionParams())
    res = Result(mission=m)
    p = world.takeoff.astype(float).copy()
    v = np.zeros(3)
    bias = np.zeros(3)
    hold = None
    airborne = False
    was_inside = False
    t = 0.0
    step = 0
    last_phase = None
    while t < sp.max_time:
        # what the drone thinks (LIO): world pose plus drift and noise
        bias += rng.normal(0, sp.pose_drift * np.sqrt(sp.dt), 3) * np.array([1, 1, 0.3])
        err = bias + rng.normal(0, sp.pose_noise, 3) * np.array([1, 1, 0.5])
        est_world = p + err
        pose = (world.to_odom(est_world), 0.0)
        scan = None
        if step % sp.scan_every == 0:
            origin = p + np.array([0, 0, LIDAR_UP])
            pts = lidar.scan(origin, world.lo, world.hi, sp.n_rays, rng, sp.range_noise,
                             drop_tags=sp.drop_tags, tags=world.tags)
            # registered with the estimated pose, so the map carries the same error
            scan = (world.to_odom(pts + err), world.to_odom(origin + err))
        cmd = m.step(t, pose, scan)
        if m.phase != last_phase:
            res.phases.append((round(t, 1), m.phase))
            last_phase = m.phase

        # command -> desired world velocity
        support = world.support_z(p[:2])
        on_ground = p[2] - BODY_DOWN <= support + 0.02
        if cmd.kind == 'velocity':
            want = world.vec_to_world(cmd.velocity)
            hold = None
        elif cmd.kind == 'takeoff':
            target_z = world.to_world(np.array([0, 0, cmd.z]))[2]
            want = np.array([0.0, 0.0, np.clip(1.0 * (target_z - p[2]), -0.5, 0.5)])
        elif cmd.kind == 'hold':
            hold = p.copy() if hold is None else hold
            want = np.clip(1.0 * (hold - p), -0.3, 0.3)
        elif cmd.kind == 'land':
            if res.landing_target is None:
                lt = m.landing if m.landing is not None else pose[0][:2]
                res.landing_target = world.to_world(np.array([lt[0], lt[1], 0.0]))[:2]
            want = np.array([0.0, 0.0, -0.3])
        else:
            raise ValueError(cmd.kind)
        v += (want - v) * min(sp.dt / sp.tau, 1.0) + rng.normal(0, sp.accel_noise * np.sqrt(sp.dt), 3)
        p = p + v * sp.dt
        support = world.support_z(p[:2])
        if p[2] - BODY_DOWN < support:
            p[2] = support + BODY_DOWN
            v[:] = 0.0
            if cmd.kind == 'land' and airborne:
                res.landed_at = p[:2].copy()
                res.done = True
                break
        if p[2] - BODY_DOWN > world.takeoff[2] - BODY_DOWN + 0.05:
            airborne = True
        if airborne:
            c = clearance(world, p, landing=cmd.kind in ('land', 'takeoff'))
            if c < res.min_clearance:
                res.min_clearance = c
                res.min_clearance_at = (round(t, 1), m.phase, np.round(p, 2))
            if c < 0:
                res.crashed = True
        ins = inside(world, p[:2])
        if ins != was_inside:
            res.crossings.append((round(t, 1), 'in' if ins else 'out', p[:2].copy()))
        if ins and not was_inside and res.entry_cross is None:
            res.entry_cross = p[:2].copy()
        if was_inside and not ins:
            res.exit_cross = p[:2].copy()
        was_inside = ins
        if step % 5 == 0:
            res.track.append(p.copy())
        if on_step:
            on_step(t, p, m, cmd)
        t += sp.dt
        step += 1
    res.time = t
    res.log = m.log
    res.fallback_used = m.fallback_used
    res.scan_ms = float(np.mean(m.scan_ms)) if m.scan_ms else 0.0
    res.scan_ms_max = float(np.percentile(m.scan_ms, 95)) if m.scan_ms else 0.0
    res.perceive_ms = float(np.mean(m.perceive_ms)) if m.perceive_ms else 0.0
    res.step_ms = float(np.mean(m.step_ms))
    res.step_ms_p95 = float(np.percentile(m.step_ms, 95))
    res.coverage = coverage(world, m)
    return res


def coverage(world, m):
    """Fraction of ground-truth interior cells the grid knows as free."""
    gt = interior_cells(world, res=m.grid.p.res, z=world.takeoff[2] + m.p.flight_z)
    odom = world.to_odom(np.column_stack([gt, np.full(len(gt), world.takeoff[2])]))[:, :2]
    ij = m.grid.to_cell(odom)
    return float(m.grid.free()[ij[:, 0], ij[:, 1]].mean())


def opening_hit(world, name, xy, margin=0.0):
    """Is xy on the named opening's gap (within its span, near its wall line)?"""
    a, b, _, _ = world.openings[name]
    a, b = np.asarray(a), np.asarray(b)
    t = (b - a) / np.linalg.norm(b - a)
    rel = np.asarray(xy) - a
    along = rel @ t
    across = abs(rel @ np.array([-t[1], t[0]]))
    return -margin <= along <= np.linalg.norm(b - a) + margin and across < 0.3
