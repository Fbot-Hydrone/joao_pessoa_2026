#!/usr/bin/env python3
"""
superlio_bridge — Super-LIO's outputs in FAST-LIO's frames, so everything
downstream (lio_odom_adapter, lio_map_node, RViz) takes either LIO unchanged.

FAST-LIO reports the IMU ("body") in the IMU's own pose at start
("camera_init"). Super-LIO reports the same IMU in "world": gravity-levelled,
yaw zeroed at start, so world -> imu at start is the start tilt, not the
identity. With T0 = the first /lio/odom pose:

    T_camerainit_body = T0^-1 * T_world_imu
    v_camerainit      = R0^T  * v_world
    p_camerainit      = T0^-1 * p_world        (every cloud point)

In:   /lio/odom          Odometry, world
      /lio/cloud_world   PointCloud2, world (Super-LIO lio.output.map: true)
Out:  /Odometry          camera_init -> body, as FAST-LIO publishes it
      /cloud_registered  camera_init
      TF camera_init -> body

T0 comes from the first pose, so start with the drone still, as FAST-LIO needs
anyway. A restarted Super-LIO starts a new world: restart this node with it.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster

from hydrone_lio.geometry import inv_tf, make_tf, matrix_to_quat, quat_to_matrix


class SuperLioBridge(Node):

    def __init__(self):
        super().__init__('superlio_bridge')
        dp = self.declare_parameter
        dp('in_odom', '/lio/odom')
        dp('in_cloud', '/lio/cloud_world')
        dp('out_odom', '/Odometry')
        dp('out_cloud', '/cloud_registered')
        dp('frame', 'camera_init')
        dp('child_frame', 'body')
        dp('publish_tf', True)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.frame, self.child = p('frame'), p('child_frame')
        self.t0_inv = None
        self.pub_odom = self.create_publisher(Odometry, p('out_odom'), 20)
        self.pub_cloud = self.create_publisher(PointCloud2, p('out_cloud'), 20)
        self.tf = TransformBroadcaster(self) if p('publish_tf') else None
        self.create_subscription(Odometry, p('in_odom'), self._cb_odom, 20)
        self.create_subscription(PointCloud2, p('in_cloud'), self._cb_cloud, 20)
        self.get_logger().info(f"superlio_bridge: {p('in_odom')}, {p('in_cloud')} -> "
                               f"{p('out_odom')}, {p('out_cloud')} in {self.frame}")

    def _cb_odom(self, msg: Odometry):
        q, pp = msg.pose.pose.orientation, msg.pose.pose.position
        t_w_imu = make_tf(quat_to_matrix(q.x, q.y, q.z, q.w), np.array([pp.x, pp.y, pp.z]))
        if self.t0_inv is None:
            self.t0_inv = inv_tf(t_w_imu)
            self.get_logger().info('first Super-LIO pose taken as camera_init')
        t = self.t0_inv @ t_w_imu
        qx, qy, qz, qw = matrix_to_quat(t[:3, :3])
        v = msg.twist.twist.linear
        vc = self.t0_inv[:3, :3] @ np.array([v.x, v.y, v.z])

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame
        out.child_frame_id = self.child
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = \
            (float(c) for c in t[:3, 3])
        out.pose.pose.orientation.x, out.pose.pose.orientation.y = float(qx), float(qy)
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = float(qz), float(qw)
        out.pose.covariance = msg.pose.covariance
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = \
            (float(c) for c in vc)
        self.pub_odom.publish(out)

        if self.tf is not None:
            tf = TransformStamped()
            tf.header = out.header
            tf.child_frame_id = self.child
            tf.transform.translation.x = out.pose.pose.position.x
            tf.transform.translation.y = out.pose.pose.position.y
            tf.transform.translation.z = out.pose.pose.position.z
            tf.transform.rotation = out.pose.pose.orientation
            self.tf.sendTransform(tf)

    def _cb_cloud(self, msg: PointCloud2):
        if self.t0_inv is None:
            return
        transform_cloud(msg, self.t0_inv)
        msg.header.frame_id = self.frame
        self.pub_cloud.publish(msg)


def transform_cloud(msg: PointCloud2, t):
    """Move every point of `msg` by the 4x4 `t`, in place (x, y, z float32)."""
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [off['x'], off['y'], off['z']], 'itemsize': msg.point_step})
    n = msg.width * msg.height
    pts = np.frombuffer(msg.data, dtype=dt, count=n)
    if not pts.flags.writeable:
        msg.data = bytearray(msg.data)
        pts = np.frombuffer(msg.data, dtype=dt, count=n)
    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=1) @ t[:3, :3].T + t[:3, 3]
    pts['x'], pts['y'], pts['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    return msg


def main(args=None):
    rclpy.init(args=args)
    node = SuperLioBridge()
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
