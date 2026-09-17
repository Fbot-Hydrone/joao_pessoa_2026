#!/usr/bin/env python3
"""
lio_odom_adapter — FAST-LIO's odometry, as the rest of the stack wants it.

FAST-LIO publishes the pose of the lidar/IMU body ("body") in the frame the
lidar had at start ("camera_init"). Everything else here talks about
base_link in `odom`, where odom is base_link's pose at start. So:

    T_odom_base = T_base_livox * T_camerainit_body * T_livox_base

Out:
  <out_odom_raw>   every LIO pose, always (motion_prior_node checks this one)
  <out_odom>       the same, unless the consistency gate is closed (EKF eats this)
  TF odom -> base_link

Gate: motion_prior_node publishes a DiagnosticStatus per window. After
`gate_after` non-OK windows in a row we stop forwarding to the EKF until an OK
one arrives. ArduPilot then dead-reckons on its IMU for a while, which beats
flying on a pose the physics says is wrong. That's the whole fallback for now.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from hydrone_lio.geometry import (inv_tf, make_tf, matrix_to_quat,
                                  quat_to_matrix, rpy_to_matrix)


class LioOdomAdapter(Node):

    def __init__(self):
        super().__init__('lio_odom_adapter')
        dp = self.declare_parameter
        dp('in_odom', '/Odometry')
        dp('out_odom', '/hydrone/lio/odom')
        dp('out_odom_raw', '/hydrone/lio/odom_raw')
        dp('in_consistency', '/hydrone/lio/consistency')
        dp('odom_frame', 'odom')
        dp('base_frame', 'base_link')
        # base_link -> livox_frame, same numbers livox_mimic / the real mount use
        dp('mount_xyz', [0.0, 0.0, 0.1])
        dp('mount_rpy_deg', [0.0, 0.0, 0.0])
        dp('publish_tf', True)
        dp('gate_enabled', True)
        dp('gate_after', 3)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        r = rpy_to_matrix(*(math.radians(v) for v in p('mount_rpy_deg')))
        self.t_base_livox = make_tf(r, np.array(p('mount_xyz'), dtype=float))
        self.t_livox_base = inv_tf(self.t_base_livox)
        self.odom_frame, self.base_frame = p('odom_frame'), p('base_frame')
        self.gate_enabled = bool(p('gate_enabled'))
        self.gate_after = int(p('gate_after'))

        self.pub = self.create_publisher(Odometry, p('out_odom'), 10)
        self.pub_raw = self.create_publisher(Odometry, p('out_odom_raw'), 10)
        self.tf = TransformBroadcaster(self) if p('publish_tf') else None
        # odom -> camera_init is just the mount, so FAST-LIO's own outputs
        # (/Odometry, /cloud_registered, its camera_init -> body TF) sit in the
        # same tree as everything else and RViz can show them in odom
        self._tf_static = StaticTransformBroadcaster(self)
        mount = TransformStamped()
        mount.header.stamp = self.get_clock().now().to_msg()
        mount.header.frame_id = self.odom_frame
        mount.child_frame_id = 'camera_init'
        mqx, mqy, mqz, mqw = matrix_to_quat(r)
        mt = self.t_base_livox[:3, 3]
        mount.transform.translation.x, mount.transform.translation.y, mount.transform.translation.z = \
            (float(v) for v in mt)
        mount.transform.rotation.x, mount.transform.rotation.y = float(mqx), float(mqy)
        mount.transform.rotation.z, mount.transform.rotation.w = float(mqz), float(mqw)
        self._tf_static.sendTransform(mount)
        self.create_subscription(Odometry, p('in_odom'), self._cb_odom, 10)
        self.create_subscription(DiagnosticStatus, p('in_consistency'), self._cb_check, 10)

        self._bad_windows = 0
        self._gated = False
        self.get_logger().info(f"lio_odom_adapter: {p('in_odom')} -> {p('out_odom')}")

    def _cb_check(self, msg: DiagnosticStatus):
        if msg.level == DiagnosticStatus.OK:
            if self._gated:
                self.get_logger().warn('consistency back, forwarding LIO to the EKF again')
            self._bad_windows, self._gated = 0, False
            return
        self._bad_windows += 1
        if self.gate_enabled and not self._gated and self._bad_windows >= self.gate_after:
            self._gated = True
            self.get_logger().error(
                f'LIO disagrees with the motor physics for {self._bad_windows} windows '
                f'({msg.message}); holding LIO poses back from the EKF')

    def _cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        pp = msg.pose.pose.position
        t_ci_body = make_tf(quat_to_matrix(q.x, q.y, q.z, q.w), np.array([pp.x, pp.y, pp.z]))
        t_odom_base = self.t_base_livox @ t_ci_body @ self.t_livox_base
        rot, pos = t_odom_base[:3, :3], t_odom_base[:3, 3]
        qx, qy, qz, qw = matrix_to_quat(rot)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = \
            (float(v) for v in pos)
        out.pose.pose.orientation.x, out.pose.pose.orientation.y = float(qx), float(qy)
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = float(qz), float(qw)
        out.pose.covariance = msg.pose.covariance

        # Vendored FAST-LIO puts its filter velocity in twist, in camera_init
        # axes (src/fast_lio/VENDORED.md). The lidar's velocity is base_link's
        # plus w x lever arm; the lever term is left out (0.5 m, slow turns).
        v = msg.twist.twist.linear
        v_odom = self.t_base_livox[:3, :3] @ np.array([v.x, v.y, v.z])
        v_body = rot.T @ v_odom
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = \
            (float(c) for c in v_body)

        self.pub_raw.publish(out)
        if not self._gated:
            self.pub.publish(out)

        if self.tf is not None:
            tf = TransformStamped()
            tf.header = out.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = out.pose.pose.position.x
            tf.transform.translation.y = out.pose.pose.position.y
            tf.transform.translation.z = out.pose.pose.position.z
            tf.transform.rotation = out.pose.pose.orientation
            self.tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = LioOdomAdapter()
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
