#!/usr/bin/env python3
"""
pose_mux_node — the one pose that flies: odometry held to a map by a smooth correction.

Thin wrapper around core.pose_mux (docs/Confined Navigation.md).

in   <odom>          odometry (LIO, gated), odom frame
     <map_pose>      beam_localizer's pose in the map frame, + <map_status>
     <source>        std_msgs/String 'odom' | 'map', from the mission
out  <out>           nav_msgs/Odometry in the map frame (feed vision_odom_bridge)
     <mode>          std_msgs/String, the source actually in use
     TF map -> odom  the anchor, so odom-frame things line up in RViz

Freshness is judged on arrival time (map poses must keep coming); the
correction matches each map pose against odometry at its scan's stamp, so
the localizer's processing time doesn't become error. The output is always
odometry through the anchor: published at the odometry rate, never a step.

Before the first odometry nothing is published. The anchor starts from the
map's takeoff pose (or start_xy_yaw) and then tracks the map pose at a capped
rate while it's healthy, so the output is in the map frame from the first
message and the EKF never sees a frame change.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from hydrone_confined.core.known_map import KnownMap
from hydrone_confined.core.pose_mux import MuxParams, PoseMux
from hydrone_confined.nodes.rosutil import quat_to_rpy, stamp_s, yaw_to_quat


class PoseMuxNode(Node):

    def __init__(self):
        super().__init__('pose_mux')
        p = {q.name: q.value for q in self.declare_parameters('', [
            ('map_yaml', ''),
            ('odom', '/hydrone/lio/odom'),
            ('map_pose', '/beam_localizer/map_pose'),
            ('map_status', '/beam_localizer/status'),
            ('source', '/hydrone/nav/source'),
            ('out', '/hydrone/nav/odom'),
            ('mode', '/hydrone/nav/mode'),
            ('start_xy_yaw', [0.0]),
            ('start_z', -1.0),
            ('track', True),
            ('tau_map', 0.3),
            ('map_timeout', 1.0),
        ])}
        self.par = p
        self.mux = PoseMux(MuxParams(tau_map=p['tau_map'], map_timeout=p['map_timeout']),
                           logger=lambda m: self.get_logger().info(m))
        self.mux.track = bool(p['track'])
        self.prior = None
        if p['map_yaml']:
            km = KnownMap.from_yaml(p['map_yaml'])
            to = km.takeoff
            z0 = p['start_z'] if p['start_z'] >= 0 else float(to.get('top', 0.0)) + 0.08
            self.prior = np.array([to['center'][0], to['center'][1], z0, to.get('yaw', 0.0)])
        if len(p['start_xy_yaw']) == 3:
            x, y, yd = p['start_xy_yaw']
            z0 = p['start_z'] if p['start_z'] >= 0 else 0.0
            self.prior = np.array([x, y, z0, math.radians(yd)])
        self.healthy = False
        self.pub = self.create_publisher(Odometry, p['out'], 10)
        self.pub_mode = self.create_publisher(String, p['mode'], 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, p['odom'], self._odom, qos_profile_sensor_data)
        self.create_subscription(Odometry, p['map_pose'], self._map, qos_profile_sensor_data)
        self.create_subscription(DiagnosticStatus, p['map_status'], self._status, 10)
        self.create_subscription(String, p['source'], self._source, 10)
        self._anchored = False

    @staticmethod
    def _pose(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        return np.array([p.x, p.y, p.z, quat_to_rpy(q.x, q.y, q.z, q.w)[2]])

    def _status(self, m):
        self.healthy = m.level == DiagnosticStatus.OK

    def _source(self, m):
        if m.data in ('odom', 'map') and m.data != self.mux.want:
            self.mux.request(m.data, self._now())

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom(self, m):
        pose = self._pose(m)
        t = self._now()
        if not self._anchored:
            if self.prior is not None:
                self.mux.set_anchor(self.prior, pose)
            self._anchored = True
        # body-frame twist -> odom axes (the mux turns it on into the map frame)
        v = m.twist.twist.linear
        c, s = math.cos(pose[3]), math.sin(pose[3])
        v_odom = np.array([c * v.x - s * v.y, s * v.x + c * v.y, v.z])
        self.mux.on_odom(t, pose, v_odom, stamp=stamp_s(m.header))
        self._publish(m.header.stamp)

    def _map(self, m):
        # arrival time for freshness, the scan's stamp to match it against odometry
        if self.mux.on_map(self._now(), self._pose(m), self.healthy, stamp=stamp_s(m.header)):
            self._publish(m.header.stamp)       # odometry is silent: the map pose flies alone

    def _publish(self, stamp):
        pose = self.mux.pose()
        if pose is None:
            return
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'map'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = \
            (float(v) for v in pose[:3])
        qx, qy, qz, qw = yaw_to_quat(pose[3])
        o.pose.pose.orientation.x, o.pose.pose.orientation.y = qx, qy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        # the bridge expects body-frame twist
        c, s = math.cos(pose[3]), math.sin(pose[3])
        v = self.mux.vel
        o.twist.twist.linear.x = float(c * v[0] + s * v[1])
        o.twist.twist.linear.y = float(-s * v[0] + c * v[1])
        o.twist.twist.linear.z = float(v[2])
        self.pub.publish(o)
        self.pub_mode.publish(String(data=self.mux.mode))
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        T = self.mux.T
        tf.transform.translation.x, tf.transform.translation.y = float(T[0]), float(T[1])
        tf.transform.translation.z = float(T[2])
        qx, qy, qz, qw = yaw_to_quat(T[3])
        tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = PoseMuxNode()
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
