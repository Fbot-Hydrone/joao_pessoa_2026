"""Kinematic drone in a box world: the simple backend every stack is tested on first.

Each step the sim builds a SensorFrame with what the drone's own sensors
would give (a drifting odometry, attitude, a lidar sweep in the body frame and
registered with the odometry, QR codes in view) and hands it to a *stack*: any
callable `stack(frame) -> Command`. The stack also tells the sim the heading it
believes it has in its command frame (`stack.yaw_est`), because a flight
controller executes a velocity in the frame of its own estimate:

    v_world = R(true yaw) R(-estimated yaw) v_cmd

`stack.z0` is the z of the takeoff point in the stack's frame (takeoff only).

Randomness for the original model (drift, noise, rays, lag) comes from one
generator in a fixed order, so old scenarios reproduce exactly; everything
added later (attitude noise, LIO degeneracy, yaw drift) draws from a second one.
"""
from dataclasses import dataclass, field

import numpy as np

from hydrone_confined.core.beams import rot_rp
from hydrone_confined.sim import lidar
from hydrone_confined.sim.world import interior_cells

BODY_R = 0.165
BODY_DOWN, BODY_UP = 0.07, 0.10     # base_link to feet / top of the prop guards
LIDAR_UP = 0.1
G = 9.81


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
    # added with the known-map stack (second generator)
    yaw_tau: float = 0.3            # yaw-rate response lag
    att_noise: float = np.deg2rad(0.3)   # roll/pitch noise the FCU reports
    degen_rate: float = 0.0         # m/s: LIO sliding along a corridor while inside
    degen_axis: tuple = (1.0, 0.0)  # world axis it slides along
    degen_flip_s: float = 8.0       # mean time between direction flips
    odom_yaw_drift: float = 0.0     # rad/sqrt(s) on the odometry heading
    qr_range: float = 1.3
    qr_fov: float = np.deg2rad(70.0)     # full horizontal FOV of the camera
    qr_max_incidence: float = np.deg2rad(60.0)
    camera_yaw: float = 0.0         # camera heading in the body frame
    lio_blackout: tuple = ()        # (t0, t1): no odometry in between, then it comes back
    lio_jump: tuple = (0.0, 0.0)    # ...this far off (m, world), like FAST-LIO after losing it


@dataclass
class SensorFrame:
    t: float
    odom: tuple                     # (xyz (3,), yaw) in odom = the takeoff pose frame; None = LIO down
    odom_vel: np.ndarray            # (3,) odom frame
    attitude: tuple                 # (roll, pitch) as the FCU reports them
    scan_body: np.ndarray = None    # (N,3) sensor frame, tilted with the body
    scan_reg: tuple = None          # (points (N,3), origin (3,)) in odom, via the odometry
    qr: list = field(default_factory=list)   # letters in view this step
    truth: np.ndarray = None        # (x, y, z, yaw) world, for metrics only


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
    qr_seen: set = field(default_factory=set)
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


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def qr_in_view(world, eye, heading, sp):
    """Letters of the world's QR codes the camera would read from eye, looking along heading."""
    seen = []
    for name, (xy, z, normal) in (world.meta.get('qr') or {}).items():
        target = np.array([xy[0], xy[1], z])
        d = target - eye
        r = np.linalg.norm(d)
        if r > sp.qr_range or r < 0.15:
            continue
        az = np.arctan2(d[1], d[0])
        if abs((az - heading + np.pi) % (2 * np.pi) - np.pi) > sp.qr_fov / 2:
            continue
        # facing the camera, not seen edge-on
        cos_inc = -(d[:2] @ np.asarray(normal)) / max(np.linalg.norm(d[:2]), 1e-9)
        if cos_inc < np.cos(sp.qr_max_incidence):
            continue
        # nothing in between (the code itself sits on a wall face: stop just short)
        rng, _ = lidar.cast(eye, (d / r)[None, :], world.lo, world.hi)
        if rng[0] < r - 0.05:
            continue
        seen.append(name)
    return seen


def run(world, stack, sparams=None, on_step=None, landing_world=None):
    """Fly `stack` in `world` until it lands or time runs out.

    landing_world(stack, frame) -> world xy it is going to land on (metrics).
    """
    sp = sparams or SimParams()
    rng = np.random.default_rng(sp.seed)
    rng2 = np.random.default_rng(sp.seed + 1000)
    res = Result(mission=stack)
    p = world.takeoff.astype(float).copy()
    v = np.zeros(3)
    v_prev = v.copy()
    psi, wz = 0.0, 0.0                  # heading turned since takeoff, and its rate
    bias = np.zeros(3)
    degen = np.zeros(3)
    degen_sign = 1.0
    yaw_bias = 0.0
    hold = None
    airborne = False
    was_inside = False
    t = 0.0
    step = 0
    last_phase = None
    axis = np.array([sp.degen_axis[0], sp.degen_axis[1], 0.0])
    axis /= max(np.linalg.norm(axis), 1e-9)
    while t < sp.max_time:
        # what the drone thinks (LIO): world pose plus drift and noise
        bias += rng.normal(0, sp.pose_drift * np.sqrt(sp.dt), 3) * np.array([1, 1, 0.3])
        err = bias + rng.normal(0, sp.pose_noise, 3) * np.array([1, 1, 0.5])
        if sp.degen_rate > 0 and inside(world, p[:2]):
            if rng2.random() < sp.dt / sp.degen_flip_s:
                degen_sign = -degen_sign
            degen += axis * degen_sign * sp.degen_rate * sp.dt
        if sp.odom_yaw_drift > 0:
            yaw_bias += rng2.normal(0, sp.odom_yaw_drift * np.sqrt(sp.dt))
        err = err + degen
        est_world = p + err
        yaw_true = world.yaw0 + psi
        if sp.lio_blackout and t >= sp.lio_blackout[1]:
            est_world = est_world + np.array([sp.lio_jump[0], sp.lio_jump[1], 0.0])
        frame = SensorFrame(t=t, odom=(world.to_odom(est_world), psi + yaw_bias),
                            odom_vel=world.to_odom(v + world.takeoff) - world.to_odom(world.takeoff),
                            attitude=(0.0, 0.0), truth=np.array([p[0], p[1], p[2], yaw_true]))
        # attitude from the acceleration the drone is producing (small-angle)
        acc_body = rz(-yaw_true) @ ((v - v_prev) / sp.dt)
        pitch = float(np.arctan2(acc_body[0], G))
        roll = float(-np.arctan2(acc_body[1], G))
        frame.attitude = (roll + rng2.normal(0, sp.att_noise), pitch + rng2.normal(0, sp.att_noise))
        if step % sp.scan_every == 0:
            origin = p + np.array([0, 0, LIDAR_UP])
            pts = lidar.scan(origin, world.lo, world.hi, sp.n_rays, rng, sp.range_noise,
                             drop_tags=sp.drop_tags, tags=world.tags)
            # registered with the estimated pose, so the map carries the same error
            frame.scan_reg = (world.to_odom(pts + err), world.to_odom(origin + err))
            # the sensor's own view: world -> level body -> tilted body
            lvl = (pts - origin) @ rz(yaw_true)
            frame.scan_body = lvl @ rot_rp(roll, pitch)
        if world.meta.get('qr'):
            eye = p + np.array([0, 0, 0.05])
            frame.qr = qr_in_view(world, eye, yaw_true + sp.camera_yaw, sp)
            res.qr_seen.update(frame.qr)
        if sp.lio_blackout and sp.lio_blackout[0] <= t < sp.lio_blackout[1]:
            frame.odom, frame.scan_reg = None, None
        cmd = stack(frame)
        phase = getattr(stack, 'phase', None)
        if phase != last_phase:
            res.phases.append((round(t, 1), phase))
            last_phase = phase

        # command -> desired world velocity, through the stack's own heading estimate
        yaw_est = getattr(stack, 'yaw_est', frame.odom[1])
        z0 = getattr(stack, 'z0', 0.0)
        w_want = 0.0
        if cmd.kind == 'velocity':
            want = rz(yaw_true) @ rz(-yaw_est) @ np.asarray(cmd.velocity, dtype=float)
            w_want = float(getattr(cmd, 'yaw_rate', 0.0) or 0.0)
            hold = None
        elif cmd.kind == 'takeoff':
            target_z = world.takeoff[2] + (cmd.z - z0)
            want = np.array([0.0, 0.0, np.clip(1.0 * (target_z - p[2]), -0.5, 0.5)])
        elif cmd.kind == 'hold':
            hold = p.copy() if hold is None else hold
            want = np.clip(1.0 * (hold - p), -0.3, 0.3)
        elif cmd.kind == 'land':
            if res.landing_target is None:
                res.landing_target = (landing_world(stack, frame) if landing_world
                                      else p[:2].copy())
            want = np.array([0.0, 0.0, -0.3])
        else:
            raise ValueError(cmd.kind)
        v_prev = v.copy()
        v += (want - v) * min(sp.dt / sp.tau, 1.0) + rng.normal(0, sp.accel_noise * np.sqrt(sp.dt), 3)
        wz += (w_want - wz) * min(sp.dt / sp.yaw_tau, 1.0)
        psi += wz * sp.dt
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
                res.min_clearance_at = (round(t, 1), phase, np.round(p, 2))
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
            res.track.append(np.array([p[0], p[1], p[2], yaw_true]))
        if on_step:
            on_step(t, p, stack, cmd, frame)
        t += sp.dt
        step += 1
    res.time = t
    res.log = list(getattr(stack, 'log', []))
    res.fallback_used = list(getattr(stack, 'fallback_used', []))
    for name, key, fn in (('scan_ms', 'scan_ms', np.mean), ('scan_ms_max', 'scan_ms', lambda a: np.percentile(a, 95)),
                          ('perceive_ms', 'perceive_ms', np.mean), ('step_ms', 'step_ms', np.mean),
                          ('step_ms_p95', 'step_ms', lambda a: np.percentile(a, 95))):
        a = getattr(stack, key, None)
        setattr(res, name, float(fn(a)) if a else 0.0)
    return res


def coverage(world, grid, flight_z):
    """Fraction of ground-truth interior cells a grid (in odom) knows as free."""
    gt = interior_cells(world, res=grid.p.res, z=world.takeoff[2] + flight_z)
    odom = world.to_odom(np.column_stack([gt, np.full(len(gt), world.takeoff[2])]))[:, :2]
    ij = grid.to_cell(odom)
    return float(grid.free()[ij[:, 0], ij[:, 1]].mean())


def opening_hit(world, name, xy, margin=0.0):
    """Is xy on the named opening's gap (within its span, near its wall line)?"""
    a, b, _, _ = world.openings[name]
    a, b = np.asarray(a), np.asarray(b)
    t = (b - a) / np.linalg.norm(b - a)
    rel = np.asarray(xy) - a
    along = rel @ t
    across = abs(rel @ np.array([-t[1], t[0]]))
    return -margin <= along <= np.linalg.norm(b - a) + margin and across < 0.3
