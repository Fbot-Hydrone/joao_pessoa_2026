#!/usr/bin/env python3
"""
motion_prior_node — does the LIO move the way the motors say it should?

The team's first hook on top of the LIO (docs/Motor Physics Prior.md).
Minimal on purpose: one check, one flag.

Physics, per ESC telemetry sample:
    thrust  = k_eta * sum(omega_i^2)             along body z
    a_pred  = R_tilt * [0, 0, thrust/mass] - g   (heading frame: yaw removed)
Over a window of `window_s`:
    d_pred  = v0 * T + double integral of a_pred    (v0 from the LIO itself)
    d_lio   = LIO displacement, rotated into the start heading frame
    ok      = |d_lio - d_pred| <= abs_margin + rel_margin * |d_pred|

Drag, wind and ground effect aren't modelled, which is what the margins pay
for. v0 comes from the LIO, so a slow smooth drift passes this check; a jump
or a pose sliding against the thrust doesn't. Catching slow drift is the
revisit-a-known-place layer's job, not this one.

Why the heading frame: the FCU's yaw and the LIO's yaw have different zeros,
but "forward of where the nose points now" is the same direction for both.

In:  esc telemetry (mavros_msgs/ESCTelemetry), attitude (sensor_msgs/Imu
     orientation, e.g. /mavros/imu/data), LIO odometry (nav_msgs/Odometry)
Out: /hydrone/lio/consistency (diagnostic_msgs/DiagnosticStatus), one per window
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from mavros_msgs.msg import ESCTelemetry
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from hydrone_lio.geometry import quat_to_matrix, rpy_to_matrix, yaw_of

G = 9.80665


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def tilt_only(r):
    """Rotation with its yaw removed: body -> heading frame."""
    y = yaw_of(r)
    return rpy_to_matrix(0.0, 0.0, -y) @ r


def predict_displacement(samples, v0, t0, t1, k_eta, mass):
    """Double-integrate thrust accel over [t0, t1].

    samples: list of (t, sum_omega_sq, R_tilt) sorted by t, heading frame.
    Zero-order hold between samples. Returns the displacement (3,).
    """
    d = np.zeros(3)
    v = np.array(v0, dtype=float)
    g = np.array([0.0, 0.0, G])
    usable = [s for s in samples if t0 <= s[0] <= t1]
    if not usable:
        return v * (t1 - t0)
    times = [t0] + [s[0] for s in usable[1:]] + [t1]
    for i, s in enumerate(usable):
        dt = times[i + 1] - times[i]
        if dt <= 0:
            continue
        a = s[2] @ np.array([0.0, 0.0, k_eta * s[1] / mass]) - g
        d += v * dt + 0.5 * a * dt * dt
        v += a * dt
    return d


def check(d_lio, d_pred, abs_margin, rel_margin):
    err = float(np.linalg.norm(np.asarray(d_lio) - np.asarray(d_pred)))
    allowed = abs_margin + rel_margin * float(np.linalg.norm(d_pred))
    return err <= allowed, err, allowed


def angle_deg(a, b):
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-6 or nb < 1e-6:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(a, b) / (na * nb))))))


class MotionPriorNode(Node):

    def __init__(self):
        super().__init__('motion_prior_node')
        dp = self.declare_parameter
        dp('in_esc', '/mavros/esc_telemetry/telemetry')
        dp('in_attitude', '/mavros/imu/data')
        dp('in_odom', '/hydrone/lio/odom_raw')
        dp('out_status', '/hydrone/lio/consistency')
        # sim KopisX8 (biguasim dynamics/agents.py); measure on the real drone
        dp('k_eta', 1.90e-6)      # N / (rad/s)^2 per rotor
        dp('mass', 2.0)           # kg
        dp('window_s', 0.5)
        dp('abs_margin', 0.10)    # m over a window
        dp('rel_margin', 0.5)     # fraction of the predicted displacement
        # below this total thrust (N) we're on the ground: nothing to check
        dp('min_thrust', 5.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.k_eta, self.mass = float(p('k_eta')), float(p('mass'))
        self.window = float(p('window_s'))
        self.abs_margin, self.rel_margin = float(p('abs_margin')), float(p('rel_margin'))
        self.min_thrust = float(p('min_thrust'))

        self._tilt = None           # latest FCU attitude, heading frame
        self._esc = deque(maxlen=2000)     # (t, sum_omega_sq, R_tilt)
        self._odom = deque(maxlen=400)     # (t, pos, R)
        self._window_start = None

        self.pub = self.create_publisher(DiagnosticStatus, p('out_status'), 10)
        self.create_subscription(ESCTelemetry, p('in_esc'), self._cb_esc, 50)
        self.create_subscription(Imu, p('in_attitude'), self._cb_att, qos_profile_sensor_data)
        self.create_subscription(Odometry, p('in_odom'), self._cb_odom, 50)
        self.get_logger().info(
            f"motion prior: {p('in_esc')} + {p('in_attitude')} vs {p('in_odom')}")

    def _cb_att(self, msg: Imu):
        q = msg.orientation
        self._tilt = tilt_only(quat_to_matrix(q.x, q.y, q.z, q.w))

    def _cb_esc(self, msg: ESCTelemetry):
        if self._tilt is None or not msg.esc_telemetry:
            return
        omega_sq = sum((it.rpm * 2 * math.pi / 60.0) ** 2 for it in msg.esc_telemetry)
        self._esc.append((stamp_s(msg.header.stamp), omega_sq, self._tilt))

    def _cb_odom(self, msg: Odometry):
        q, pp = msg.pose.pose.orientation, msg.pose.pose.position
        t = stamp_s(msg.header.stamp)
        self._odom.append((t, np.array([pp.x, pp.y, pp.z]), quat_to_matrix(q.x, q.y, q.z, q.w)))
        if self._window_start is None:
            self._window_start = t
        if t - self._window_start >= self.window:
            self._evaluate(self._window_start, t)
            self._window_start = t

    def _pose_at(self, t):
        return min(self._odom, key=lambda o: abs(o[0] - t))

    def _evaluate(self, t0, t1):
        if len(self._odom) < 3 or not self._esc:
            return
        start, end = self._pose_at(t0), self._pose_at(t1)
        prev = [o for o in self._odom if o[0] < start[0]]
        if not prev:
            return
        before = prev[-1]
        # heading frame at the start of the window
        to_heading = rpy_to_matrix(0.0, 0.0, -yaw_of(start[2]))
        v0 = to_heading @ ((start[1] - before[1]) / max(start[0] - before[0], 1e-3))
        d_lio = to_heading @ (end[1] - start[1])

        samples = [s for s in self._esc if t0 <= s[0] <= t1]
        status = DiagnosticStatus()
        status.name = 'lio_motion_prior'
        status.hardware_id = 'fast_lio'
        if not samples:
            status.level = DiagnosticStatus.STALE
            status.message = 'no ESC telemetry in window'
            self.pub.publish(status)
            return
        mean_thrust = self.k_eta * float(np.mean([s[1] for s in samples]))
        if mean_thrust < self.min_thrust:
            # on the ground, motors idle: a still LIO is the only right answer
            ok, err, allowed = check(d_lio, np.zeros(3), self.abs_margin, 0.0)
            d_pred = np.zeros(3)
        else:
            d_pred = predict_displacement(samples, v0, t0, t1, self.k_eta, self.mass)
            ok, err, allowed = check(d_lio, d_pred, self.abs_margin, self.rel_margin)

        status.level = DiagnosticStatus.OK if ok else DiagnosticStatus.WARN
        status.message = f'err {err:.3f} m / allowed {allowed:.3f} m'
        status.values = [
            KeyValue(key='err_m', value=f'{err:.4f}'),
            KeyValue(key='allowed_m', value=f'{allowed:.4f}'),
            KeyValue(key='angle_deg', value=f'{angle_deg(d_lio, d_pred):.1f}'),
            KeyValue(key='d_lio', value=np.array2string(d_lio, precision=3)),
            KeyValue(key='d_pred', value=np.array2string(d_pred, precision=3)),
            KeyValue(key='thrust_n', value=f'{mean_thrust:.2f}'),
        ]
        self.pub.publish(status)
        if not ok:
            self.get_logger().warn(f'LIO vs physics: {status.message}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = MotionPriorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
