#!/usr/bin/env python3
"""
livox_mimic_node — the Livox Mid-360's ROS 2 topics, from BiguaSim.

SIM ONLY. On the drone `livox_ros_driver2` (xfer_format 1) publishes the same
topics and this node isn't launched.

In:   <agent>/Mid360      PointCloud2, base_link, one engine tick of raycasts
      <agent>/IMUSensor   Imu, body frame, m/s^2
Out:  /livox/lidar        livox_ros_driver2/CustomMsg, livox_frame, 10 Hz
      /livox/lidar_pc     PointCloud2 copy for RViz (optional)
      /livox/imu          Imu in livox_frame, accel in g like the real driver
      TF base_link -> livox_frame (static, the mount)

The engine lidar is a real raycaster whose rings would repeat every sweep;
ardubridge wobbles it a few degrees so they don't (see
docs/Livox Mid-360 Sim.md). Here we only do what the device does on top:
cut 100 ms windows (by stamp, which is sim time, so the window doesn't depend
on how fast the sim runs), add range noise and dropouts, and pack.
"""

import array
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster
from livox_ros_driver2.msg import CustomMsg, CustomPoint

G = 9.80665
FRAME_BASE = 'base_link'


def _rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


class LivoxMimicNode(Node):

    def __init__(self):
        super().__init__('livox_mimic')
        dp = self.declare_parameter
        dp('in_cloud', '/biguasim/uav0_id0/Mid360')
        dp('in_imu', '/biguasim/uav0_id0/IMUSensor')
        dp('out_cloud', '/livox/lidar')
        dp('out_cloud_pc', '/livox/lidar_pc')   # empty disables it
        dp('out_imu', '/livox/imu')
        dp('frame_id', 'livox_frame')
        dp('publish_rate_hz', 10.0)
        # Mid-360 datasheet: 0.1 m blind zone, range error <= 2 cm (1 sigma)
        dp('min_range', 0.1)
        dp('max_range', 70.0)
        dp('range_noise_std', 0.02)
        # fraction of returns lost at random (the real unit misses some)
        dp('dropout', 0.02)
        # the unit sends ~20k pts per message; keep at most this many
        dp('max_points', 20000)
        dp('reflectivity', 100)
        dp('mount_xyz', [0.0, 0.0, 0.1])
        dp('mount_rpy_deg', [0.0, 0.0, 0.0])
        # BiguaSim spawns the drone above the ground and it falls for ~0.2 s.
        # FAST-LIO takes gravity from its first ~10 IMU samples, so a start in
        # free fall tilts the whole map by degrees. A real unit powers up on a
        # still drone; stay silent until the sim has settled.
        dp('startup_skip_s', 3.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.frame_id = p('frame_id')
        self.window_ns = int(1e9 / float(p('publish_rate_hz')))
        self.min_range, self.max_range = float(p('min_range')), float(p('max_range'))
        self.noise = float(p('range_noise_std'))
        self.dropout = float(p('dropout'))
        self.max_points = int(p('max_points'))
        self.reflectivity = int(p('reflectivity'))
        self.rng = np.random.default_rng()
        self.skip_ns = int(float(p('startup_skip_s')) * 1e9)
        self._first_ns = None

        # base_link -> livox_frame; points arrive in base_link
        self.t_mount = np.array(p('mount_xyz'), dtype=np.float64)
        self.r_mount = _rpy_matrix(*(math.radians(v) for v in p('mount_rpy_deg')))
        self._publish_mount_tf()

        self.pub_cloud = self.create_publisher(CustomMsg, p('out_cloud'), 10)
        self.pub_pc = (self.create_publisher(PointCloud2, p('out_cloud_pc'), 5)
                       if p('out_cloud_pc') else None)
        self.pub_imu = self.create_publisher(Imu, p('out_imu'), 200)
        # deep queues, the bridge publishes both every tick
        self.create_subscription(PointCloud2, p('in_cloud'), self._cb_cloud, 400)
        self.create_subscription(Imu, p('in_imu'), self._cb_imu, 400)

        self._gyro_hist = deque(maxlen=5)   # ~20 ms at 200 Hz
        self._window_start = None
        self._chunks = []       # (stamp_ns, Nx3 in livox_frame)
        self._published = 0
        self.get_logger().info(
            f"livox_mimic: {p('in_cloud')} -> {p('out_cloud')} ({self.frame_id})")

    def _publish_mount_tf(self):
        self._tf_static = StaticTransformBroadcaster(self)
        r = self.r_mount
        # rotation matrix -> quaternion (mount is near identity, no edge cases)
        w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = FRAME_BASE
        t.child_frame_id = self.frame_id
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            (float(v) for v in self.t_mount)
        t.transform.rotation.w = w
        t.transform.rotation.x = (r[2, 1] - r[1, 2]) / (4 * w)
        t.transform.rotation.y = (r[0, 2] - r[2, 0]) / (4 * w)
        t.transform.rotation.z = (r[1, 0] - r[0, 1]) / (4 * w)
        self._tf_static.sendTransform(t)

    @staticmethod
    def _ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _settling(self, stamp_ns):
        if self._first_ns is None:
            self._first_ns = stamp_ns
        return stamp_ns - self._first_ns < self.skip_ns

    def _cb_imu(self, msg: Imu):
        if self._settling(self._ns(msg.header.stamp)):
            return
        # The real Mid-360's IMU sits inside the unit, so move the body IMU's
        # reading to the mount point (full lever arm: alpha x r + w x (w x r))
        # before rotating it into the lidar's axes. Angular acceleration is a
        # backward difference over ~20 ms: a 5 ms one turned sim jitter into
        # 0.5 g of noise, and dropping the term entirely cost 50 m/s^2 when the
        # airframe oscillated. The real driver reports accel in g.
        w_b = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        a_b = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                        msg.linear_acceleration.z])
        t = self._ns(msg.header.stamp)
        self._gyro_hist.append((t, w_b))
        alpha = np.zeros(3)
        t0, w0 = self._gyro_hist[0]
        if t > t0:
            alpha = (w_b - w0) / ((t - t0) * 1e-9)
        r = self.t_mount
        a_b = a_b + np.cross(alpha, r) + np.cross(w_b, np.cross(w_b, r))
        a = self.r_mount.T @ a_b
        w = self.r_mount.T @ w_b
        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.orientation_covariance[0] = -1.0
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = \
            (float(v / G) for v in a)
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = \
            (float(v) for v in w)
        self.pub_imu.publish(out)

    def _cb_cloud(self, msg: PointCloud2):
        stamp = self._ns(msg.header.stamp)
        if self._settling(stamp):
            return
        if self._window_start is None:
            self._window_start = stamp
        # a tick past the window closes it; this tick opens the next one
        if stamp - self._window_start >= self.window_ns:
            self._flush()
            self._window_start = stamp

        if msg.width == 0:
            return
        pts = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :3]
        pts = (pts.astype(np.float64) - self.t_mount) @ self.r_mount
        rng = np.linalg.norm(pts, axis=1)
        keep = (rng > self.min_range) & (rng < self.max_range)
        if self.dropout > 0:
            keep &= self.rng.random(len(pts)) >= self.dropout
        pts, rng = pts[keep], rng[keep]
        if self.noise > 0 and len(pts):
            # noise along the beam, like a real time-of-flight error
            pts *= ((rng + self.rng.normal(0.0, self.noise, len(rng))) / rng)[:, None]
        self._chunks.append((stamp, pts.astype(np.float32)))

    def _flush(self):
        chunks, self._chunks = self._chunks, []
        if not chunks:
            return
        base = self._window_start
        pts = np.concatenate([c[1] for c in chunks])
        offs = np.concatenate([np.full(len(c[1]), c[0] - base, dtype=np.int64) for c in chunks])
        if len(pts) > self.max_points:
            idx = np.sort(self.rng.choice(len(pts), self.max_points, replace=False))
            pts, offs = pts[idx], offs[idx]
        if not len(pts):
            return
        # Mid-360 reports 4 laser lines; band by elevation so line is sane
        el = np.arctan2(pts[:, 2], np.hypot(pts[:, 0], pts[:, 1]))
        lines = np.clip(((el + math.radians(7)) / math.radians(59) * 4).astype(int), 0, 3)

        msg = CustomMsg()
        msg.header.stamp = rclpy.time.Time(nanoseconds=base).to_msg()
        msg.header.frame_id = self.frame_id
        msg.timebase = base
        msg.point_num = len(pts)
        msg.lidar_id = 0
        points = []
        for (x, y, z), off, line in zip(pts.tolist(), offs.tolist(), lines.tolist()):
            cp = CustomPoint()
            cp.offset_time = off
            cp.x, cp.y, cp.z = x, y, z
            cp.reflectivity = self.reflectivity
            cp.tag = 0
            cp.line = line
            points.append(cp)
        msg.points = points
        self.pub_cloud.publish(msg)

        if self.pub_pc is not None and self.pub_pc.get_subscription_count():
            pc = PointCloud2()
            pc.header = msg.header
            pc.height, pc.width = 1, len(pts)
            pc.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                         for i, n in enumerate('xyz')]
            pc.point_step, pc.row_step = 12, 12 * len(pts)
            pc.is_dense = True
            pc.data = array.array('B', pts.tobytes())
            self.pub_pc.publish(pc)

        self._published += 1
        if self._published == 1:
            self.get_logger().info(f"first scan out: {len(pts)} points from {len(chunks)} ticks")


def main(args=None):
    rclpy.init(args=args)
    node = LivoxMimicNode()
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
