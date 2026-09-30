"""Which pose flies the drone: odometry (re-anchored to a map) or a map pose.

Two sources, one output frame ("nav"):
  odom  a smooth, fast, drifting odometry (LIO), in its own frame. Out:
        T_nav_odom * odom. T is the anchor, the map -> odom correction of the
        usual map/odom split. It can be set outright (at start), or pulled
        toward the map pose at a capped rate while the map pose is healthy.
  map   a pose straight from a map localizer, already in nav.

Switching never jumps the output: going to map, the offset between the two is
kept and decays (tau); going back to odom, the anchor is reset so that odom
continues from where the output was. A stale or unhealthy map pose drops
back to odom on its own, and says why.

Poses are (x, y, z, yaw). Pure logic, no ROS.
"""
from dataclasses import dataclass

import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def compose(a, b):
    """a * b for (x, y, z, yaw) poses (planar rotation, z added)."""
    c, s = np.cos(a[3]), np.sin(a[3])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2],
                     wrap(a[3] + b[3])])


def inverse(a):
    c, s = np.cos(a[3]), np.sin(a[3])
    return np.array([-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), -a[2], wrap(-a[3])])


def between(a, b):
    """The anchor T with T * b = a."""
    return compose(a, inverse(b))


@dataclass
class MuxParams:
    tau: float = 1.0                    # s, offset decay after a switch to map
    anchor_xy_rate: float = 0.25        # m/s the anchor may move while tracking the map
    anchor_yaw_rate: float = np.deg2rad(10.0)
    anchor_z: bool = False              # also pull odom z to the map's
    map_timeout: float = 1.0            # s without a map pose -> back to odom
    rejoin_s: float = 0.5               # map pose fresh and healthy this long before (re)joining it


class PoseMux:

    def __init__(self, params=None, logger=None):
        self.p = params or MuxParams()
        self.T = np.zeros(4)                # nav <- odom
        self.mode = 'odom'
        self.want = 'odom'
        self.track = False                  # pull the anchor toward the map in odom mode
        self.odom = None                    # (t, pose)
        self.map = None                     # (t, pose, healthy)
        self.out = None                     # (t, pose)
        self.vel = np.zeros(3)
        self._offset = np.zeros(4)
        self._t_switch = None
        self._ok_since = None
        self.logger = logger
        self.events = []                    # (t, message)

    def say(self, t, msg):
        self.events.append((t, msg))
        if self.logger:
            self.logger(msg)

    # ── control ─────────────────────────────────────────────────────────────
    def set_anchor(self, nav_pose, odom_pose):
        """Declare that odom_pose is nav_pose right now."""
        self.T = between(np.asarray(nav_pose, float), np.asarray(odom_pose, float))

    def request(self, mode, t=None):
        if mode not in ('odom', 'map'):
            raise ValueError(mode)
        self.want = mode
        if t is not None:
            self._switch(t)

    def map_ok(self, t):
        return self.map is not None and self.map[2] and t - self.map[0] <= self.p.map_timeout

    # ── inputs ──────────────────────────────────────────────────────────────
    def on_odom(self, t, pose, vel_nav=None):
        """pose (x, y, z, yaw) in odom; vel_nav optional (3,) already in nav axes."""
        pose = np.asarray(pose, dtype=float)
        dt = t - self.odom[0] if self.odom is not None else 0.0
        self.odom = (t, pose)
        self._switch(t)
        if self.mode != 'odom':
            return
        if self.track and self.map_ok(t) and dt > 0:
            self._pull_anchor(dt, pose)
        self._emit(t, compose(self.T, pose), vel_nav)

    def on_map(self, t, pose, healthy=True):
        self.map = (t, np.asarray(pose, dtype=float), bool(healthy))
        if not healthy:
            self._ok_since = None
        elif self._ok_since is None:
            self._ok_since = t
        self._switch(t)
        if self.mode != 'map':
            return
        k = np.exp(-(t - self._t_switch) / self.p.tau) if self.p.tau > 0 else 0.0
        self._emit(t, self.map[1] + self._offset * k)

    # ── internals ───────────────────────────────────────────────────────────
    def _pull_anchor(self, dt, odom_pose):
        target = between(self.map[1], odom_pose)
        d = target - self.T
        d[3] = wrap(d[3])
        step_xy = self.p.anchor_xy_rate * dt
        n = np.hypot(d[0], d[1])
        if n > step_xy:
            d[:2] *= step_xy / n
        d[3] = np.clip(d[3], -self.p.anchor_yaw_rate * dt, self.p.anchor_yaw_rate * dt)
        d[2] = np.clip(d[2], -step_xy, step_xy) if self.p.anchor_z else 0.0
        self.T = self.T + d
        self.T[3] = wrap(self.T[3])

    def _switch(self, t):
        if self.mode == 'map' and (self.want == 'odom' or not self.map_ok(t)):
            why = 'requested' if self.want == 'odom' else (
                'map pose stale' if self.map is None or t - self.map[0] > self.p.map_timeout
                else 'map pose unhealthy')
            if self.odom is not None:
                # carry on from where the output is
                last = self.out[1] if self.out is not None else self.map[1]
                self.T = between(last, self.odom[1])
            self.mode = 'odom'
            if why != 'requested':
                self._ok_since = None
            self.say(t, f'pose source -> odom ({why})')
        elif self.mode == 'odom' and self.want == 'map' and self.map_ok(t) \
                and self._ok_since is not None and t - self._ok_since >= self.p.rejoin_s:
            last = self.out[1] if self.out is not None else self.map[1]
            self._offset = last - self.map[1]
            self._offset[3] = wrap(self._offset[3])
            self._t_switch = t
            self.mode = 'map'
            self.say(t, f'pose source -> map (offset {np.round(self._offset, 3).tolist()})')

    def _emit(self, t, pose, vel=None):
        pose = np.asarray(pose, dtype=float)
        pose[3] = wrap(pose[3])
        if vel is not None:
            self.vel = np.asarray(vel, dtype=float)
        elif self.out is not None and t > self.out[0]:
            # finite difference, smoothed: the map pose is noisier than LIO
            v = (pose[:3] - self.out[1][:3]) / (t - self.out[0])
            a = min(1.0, (t - self.out[0]) / 0.3)
            self.vel = (1 - a) * self.vel + a * v
        self.out = (t, pose)

    def pose(self):
        return None if self.out is None else self.out[1]
