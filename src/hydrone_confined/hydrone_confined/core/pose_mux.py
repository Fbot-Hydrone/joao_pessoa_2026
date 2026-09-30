"""The one pose that flies: odometry, held to a map by a correction that never jumps.

Output (frame "nav"): T * odom, always. odom (LIO) gives the smooth,
fast, locally right motion the flight controller's velocity loop needs; the
map pose (beam localizer) decides where that motion *is*, by pulling the
anchor T (the usual map -> odom correction) toward map * odom^-1.

How hard it pulls depends on what's asked for:
  'map'   fast (tau 0.3 s, capped at 1 m/s and 45 deg/s, z too): the walls rule.
          A LIO that slides along a corridor at s m/s is held to ~s * tau.
  'odom'  slow and planar (tau 2 s, capped 0.25 m/s, 10 deg/s), and only if
          `track`: LIO rules, the map only keeps it from wandering.
An unhealthy or stale map pose pulls nothing; the anchor just holds. So the
output has no steps, ever: the EKF behind it never sees a jump, which is
what made the drone swing in BiguaSim when the raw map pose was fed through.

The map pose is matched against odometry *at its own timestamp* (the scan's),
from a short odometry history, so processing latency doesn't turn into error.

When odometry fails, the map pose carries on alone ('map-only'):
  - silent for odom_timeout (FAST-LIO "No Effective Points" in a 1 m room,
    or the motion-prior gate holding it back): the output becomes the map
    pose, smoothed, emitted on each map pose
  - lying (a step or a velocity faster than max_odom_speed: a diverging
    FAST-LIO runs away at ~20 m/s): the same, and its velocity is not passed
    on (in BiguaSim the EKF failed safe on exactly that velocity)
  - back and sane for odom_sane_s: re-anchored so it continues from where
    the output is
In BiguaSim the EKF lost external nav when LIO went silent inside, failed
safe to LAND and the drone crashed; that's what this is for.

Poses are (x, y, z, yaw). Pure logic, no ROS.
"""
from collections import deque
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
    tau_map: float = 0.3                # s, correction time constant when the map is asked for
    map_xy_rate: float = 1.0            # m/s cap
    map_yaw_rate: float = np.deg2rad(45.0)
    map_z_rate: float = 0.5
    tau_odom: float = 2.0               # when odometry is asked for (and track is on)
    anchor_xy_rate: float = 0.25
    anchor_yaw_rate: float = np.deg2rad(10.0)
    map_timeout: float = 1.0            # s without a map pose: stop correcting
    rejoin_s: float = 0.5               # healthy this long before corrections resume
    history_s: float = 3.0              # odometry kept to match map stamps
    odom_timeout: float = 0.35          # s without odometry: fly on the map pose alone
    max_odom_speed: float = 1.2         # m/s; faster odometry (step or velocity) is lying
    map_only_tau: float = 0.2           # s, smoothing of the map pose when it flies alone
    odom_sane_s: float = 1.0            # odometry must look sane this long before it's trusted again


class PoseMux:

    def __init__(self, params=None, logger=None):
        self.p = params or MuxParams()
        self.T = np.zeros(4)                # nav <- odom
        self.want = 'odom'
        self.mode = 'odom'                  # what's actually steering the anchor
        self.track = False                  # slow map tracking while odom is asked for
        self.odom = None                    # (t, pose)
        self.map = None                     # (t_arrival, pose, healthy)
        self.out = None                     # (t, pose)
        self.vel = np.zeros(3)
        self._hist = deque()                # (stamp, pose)
        self._ok_since = None
        self._t_corr = None
        self._sane_since = None
        self.last_offset = 0.0              # |map - output| at the last correction, m
        self.logger = logger
        self.events = []

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
            self._update_mode(t)

    def map_ok(self, t):
        return (self.map is not None and self.map[2] and t - self.map[0] <= self.p.map_timeout
                and self._ok_since is not None and t - self._ok_since >= self.p.rejoin_s)

    # ── inputs ──────────────────────────────────────────────────────────────
    def on_odom(self, t, pose, vel_odom=None, stamp=None):
        """pose (x, y, z, yaw) in odom; vel_odom optional (3,) in odom axes.
        stamp: the pose's own time (defaults to t) for matching map poses."""
        pose = np.asarray(pose, dtype=float)
        st = t if stamp is None else stamp
        v_ok = vel_odom is None or float(np.linalg.norm(np.asarray(vel_odom)[:2])) <= self.p.max_odom_speed
        if self.odom is not None and self.out is not None:
            gap = t - self.odom[0]
            step = float(np.hypot(*(pose[:2] - self.odom[1][:2])))
            jump = step > self.p.max_odom_speed * max(gap, 0.05) + 0.05
            if jump or not v_ok:
                # odometry is lying (a diverging FAST-LIO runs away at ~20 m/s):
                # don't follow it, don't pass its velocity on; the map flies
                self._sane_since = None
                self.odom = (t, pose)
                if self.mode != 'map-only':
                    self.say(t, f'odometry jumped {step:.2f} m in {gap:.2f} s'
                             + ('' if v_ok else ' (velocity implausible)') + ': flying on the map pose alone')
                    self.mode = 'map-only'
                return
            if self.mode == 'map-only' or gap > self.p.odom_timeout:
                # back (or sane again): trust it after odom_sane_s of sane steps
                if self._sane_since is None or gap > self.p.odom_timeout:
                    self._sane_since = t
                self.odom = (t, pose)
                if t - self._sane_since < self.p.odom_sane_s and self.map_ok_any(t):
                    if self.mode != 'map-only':
                        self.mode = 'map-only'
                    return
                self.T = between(self.out[1], pose)
                self._hist.clear()
                self.say(t, f'odometry sane again, re-anchored')
                self.mode = 'odom'
        self._hist.append((st, pose))
        while self._hist and st - self._hist[0][0] > self.p.history_s:
            self._hist.popleft()
        self.odom = (t, pose)
        self._update_mode(t)
        out = compose(self.T, pose)
        c, s = np.cos(self.T[3]), np.sin(self.T[3])
        if vel_odom is not None and v_ok:
            v = np.asarray(vel_odom, dtype=float)
            self.vel = np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])
        elif self.out is not None and t > self.out[0]:
            v = (out[:3] - self.out[1][:3]) / (t - self.out[0])
            a = min(1.0, (t - self.out[0]) / 0.3)
            self.vel = (1 - a) * self.vel + a * v
        self.out = (t, out)

    def odom_alive(self, t):
        return (self.odom is not None and t - self.odom[0] <= self.p.odom_timeout
                and self.mode != 'map-only')

    def map_ok_any(self, t):
        """A healthy, fresh map pose to fly on alone (no rejoin wait)."""
        return self.map is not None and self.map[2] and t - self.map[0] <= self.p.map_timeout

    def on_map(self, t, pose, healthy=True, stamp=None):
        """t: arrival time (freshness); stamp: when the scan behind it was taken.
        Returns True when it moved the output by itself (map-only)."""
        pose = np.asarray(pose, dtype=float)
        self.map = (t, pose, bool(healthy))
        if not healthy:
            self._ok_since = None
        elif self._ok_since is None:
            self._ok_since = t
        if not self.odom_alive(t) and self.out is not None and healthy:
            # no odometry: the map pose flies on its own, smoothed
            if self.mode != 'map-only':
                self.say(t, 'odometry silent: flying on the map pose alone')
                self.mode = 'map-only'
            dt = float(np.clip(t - self.out[0], 1e-3, 0.5))
            k = min(1.0, dt / self.p.map_only_tau)
            d = pose - self.out[1]
            d[3] = wrap(d[3])
            new = self.out[1] + k * d
            new[3] = wrap(new[3])
            self.vel = (new[:3] - self.out[1][:3]) / dt
            self.out = (t, new)
            return True
        if self.mode == 'map-only':
            return False                    # wait for odometry to come back (it re-anchors)
        self._update_mode(t)
        fast = self.mode == 'map'
        if not (fast or (self.track and self.map_ok(t))):
            self._t_corr = None
            return False
        od = self._odom_at(t if stamp is None else stamp)
        if od is None:
            return False
        dt = 0.1 if self._t_corr is None else float(np.clip(t - self._t_corr, 1e-3, 0.5))
        self._t_corr = t
        target = between(pose, od)
        d = target - self.T
        d[3] = wrap(d[3])
        self.last_offset = float(np.hypot(d[0], d[1]))
        if fast:
            k = min(1.0, dt / self.p.tau_map)
            xy_cap, yaw_cap, z_cap = self.p.map_xy_rate * dt, self.p.map_yaw_rate * dt, self.p.map_z_rate * dt
        else:
            k = min(1.0, dt / self.p.tau_odom)
            xy_cap, yaw_cap, z_cap = self.p.anchor_xy_rate * dt, self.p.anchor_yaw_rate * dt, 0.0
        step = d * k
        n = np.hypot(step[0], step[1])
        if n > xy_cap:
            step[:2] *= xy_cap / n
        step[3] = np.clip(step[3], -yaw_cap, yaw_cap)
        step[2] = np.clip(step[2], -z_cap, z_cap)
        self.T = self.T + step
        self.T[3] = wrap(self.T[3])
        if self.odom is not None:
            self.out = (self.out[0] if self.out else t, compose(self.T, self.odom[1]))
        return False

    # ── internals ───────────────────────────────────────────────────────────
    def _odom_at(self, st):
        """Odometry pose at stamp st, interpolated from the history (or the latest)."""
        if not self._hist:
            return None
        h = self._hist
        if st >= h[-1][0]:
            return h[-1][1]
        if st <= h[0][0]:
            return h[0][1]
        for (t0, p0), (t1, p1) in zip(h, list(h)[1:]):
            if t0 <= st <= t1:
                a = (st - t0) / max(t1 - t0, 1e-9)
                p = p0 + a * (p1 - p0)
                p[3] = wrap(p0[3] + a * wrap(p1[3] - p0[3]))
                return p
        return h[-1][1]

    def _update_mode(self, t):
        if self.mode == 'map-only':
            return
        mode = 'map' if (self.want == 'map' and self.map_ok(t)) else 'odom'
        if mode != self.mode:
            if mode == 'map':
                why = 'map pose healthy'
            elif self.want == 'odom':
                why = 'requested'
            elif self.map is None or t - self.map[0] > self.p.map_timeout:
                why = 'map pose stale'
            else:
                why = 'map pose unhealthy'
            self.say(t, f'pose source -> {mode} ({why})')
            self.mode = mode

    def pose(self):
        return None if self.out is None else self.out[1]
