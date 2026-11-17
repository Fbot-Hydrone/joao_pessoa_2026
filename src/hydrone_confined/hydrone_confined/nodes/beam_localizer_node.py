#!/usr/bin/env python3
"""
beam_localizer_node — where the drone is in a known map, from lidar beams.

Thin wrapper around core.map_localization (docs/Confined Navigation.md).

in   <cloud>        the lidar in its own frame: PointCloud2 (cloud_type pc2)
                    or livox_ros_driver2/CustomMsg read raw (cloud_type livox)
     <imu>          FCU attitude, only roll and pitch are used (levelling)
     <odom>         any odometry (LIO); only its step between scans is used,
                    and only when it's believable
out  ~/map_pose     nav_msgs/Odometry, base_link in the map frame, covariance
     ~/status       DiagnosticStatus: OK = healthy enough to fly on
     ~/beams        LaserScan, levelled body-frame beams (for the mission, RViz)
     ~/sectors      LaserScan, the guard's sectors
     ~/particles    PoseArray, every particles_every-th scan

The prior (where the drone starts) comes from the map's takeoff block unless
start_xy_yaw is given.
"""
import math
import struct

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2

from hydrone_confined.core.beams import BeamParams
from hydrone_confined.core.known_map import KnownMap
from hydrone_confined.core.map_localization import MapLocalization, MapLocParams
from hydrone_confined.nodes.rosutil import cloud_xyz, livox_custom_xyz, quat_to_rpy, yaw_to_quat


def scan_msg(beams, stamp, frame, max_range):
    m = LaserScan()
    m.header.stamp = stamp
    m.header.frame_id = frame
    n = len(beams.angles)
    m.angle_min = float(beams.angles[0])
    m.angle_increment = float(2 * math.pi / n)
    m.angle_max = float(beams.angles[0] + m.angle_increment * (n - 1))
    m.range_min = 0.1
    m.range_max = float(max_range)
    m.ranges = [float(r) if np.isfinite(r) else float('inf') for r in beams.ranges]
    return m


class BeamLocalizerNode(Node):

    def __init__(self):
        super().__init__('beam_localizer')
        p = {q.name: q.value for q in self.declare_parameters('', [
            ('map_yaml', ''),
            ('cloud', '/livox/lidar_pc'),
            ('cloud_type', 'pc2'),              # pc2 / livox
            ('imu', '/mavros/imu/data'),
            ('odom', '/hydrone/lio/odom_raw'),
            ('mount_xyz', [0.0, 0.0, 0.1]),     # sensor over base_link
            ('n_beams', 24),
            ('n_sectors', 16),
            ('start_xy_yaw', [0.0]),            # [x, y, yaw_deg] overrides the map's takeoff
            ('start_z', -1.0),
            ('max_points', 30000),
            ('particles_every', 5),
        ])}
        self.par = p
        if not p['map_yaml']:
            raise RuntimeError('beam_localizer: map_yaml is required')
        self.km = KnownMap.from_yaml(p['map_yaml'])
        lp = MapLocParams(beams=BeamParams(n=int(p['n_beams'])), sectors=int(p['n_sectors']),
                          mount=tuple(float(v) for v in p['mount_xyz']))
        self.loc = MapLocalization(self.km, lp)
        to = self.km.takeoff
        if len(p['start_xy_yaw']) == 3:
            x, y, yd = p['start_xy_yaw']
            start = (np.array([x, y]), math.radians(yd))
        else:
            start = (to['center'], to.get('yaw', 0.0))
        z0 = p['start_z'] if p['start_z'] >= 0 else float(to.get('top', 0.0)) + 0.08
        self.loc.start(start[0], start[1], z0)
        self.get_logger().info(f'map {self.km.name}: start at {np.round(start[0], 2)}, '
                               f'yaw {math.degrees(start[1]):.0f} deg, z {z0:.2f}')

        self.att = (0.0, 0.0)
        self.odom = None
        self.odom_prev = None
        self.pub_pose = self.create_publisher(Odometry, '~/map_pose', 10)
        self.pub_status = self.create_publisher(DiagnosticStatus, '~/status', 10)
        self.pub_beams = self.create_publisher(LaserScan, '~/beams', 10)
        self.pub_sectors = self.create_publisher(LaserScan, '~/sectors', 10)
        self.pub_particles = self.create_publisher(PoseArray, '~/particles', 1)
        self._n = 0
        self.create_subscription(Imu, p['imu'], self._imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, p['odom'], self._odom, qos_profile_sensor_data)
        if p['cloud_type'] == 'livox':
            from livox_ros_driver2.msg import CustomMsg
            self.create_subscription(CustomMsg, p['cloud'], self._cloud_raw, qos_profile_sensor_data,
                                     raw=True)
        else:
            self.create_subscription(PointCloud2, p['cloud'], self._cloud_pc2, qos_profile_sensor_data)
        self._t_last = None

    def _imu(self, m):
        q = m.orientation
        r, pch, _ = quat_to_rpy(q.x, q.y, q.z, q.w)
        self.att = (r, pch)

    def _odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.odom = np.array([p.x, p.y, p.z, quat_to_rpy(q.x, q.y, q.z, q.w)[2]])

    def _cloud_pc2(self, m):
        n = m.width * m.height
        stride = max(1, int(math.ceil(n / max(self.par['max_points'], 1000))))
        self._scan(cloud_xyz(m, stride), m.header.stamp)

    def _cloud_raw(self, data):
        # raw=True hands us bytes; the stamp is the first 8 bytes after the CDR header
        sec, nsec = struct.unpack_from('<iI', data, 4)
        self._scan(livox_custom_xyz(data), Time(sec=sec, nanosec=nsec))

    def _scan(self, pts, stamp):
        t = stamp.sec + stamp.nanosec * 1e-9
        step = None
        if self.odom is not None and self.odom_prev is not None:
            d = self.odom - self.odom_prev
            c, s = math.cos(self.odom_prev[3]), math.sin(self.odom_prev[3])
            dyaw = (d[3] + math.pi) % (2 * math.pi) - math.pi
            step = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2], dyaw])
        self.odom_prev = None if self.odom is None else self.odom.copy()
        pts = pts[np.isfinite(pts).all(axis=1)]
        fix = self.loc.step(t, pts, self.att, step)
        self._publish(fix, stamp)

    def _publish(self, fix, stamp):
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'map'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = \
            (float(v) for v in fix.pose[:3])
        qx, qy, qz, qw = yaw_to_quat(fix.pose[3])
        o.pose.pose.orientation.x, o.pose.pose.orientation.y = qx, qy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        cov = np.zeros((6, 6))
        cov[0, 0], cov[0, 1], cov[1, 0], cov[1, 1] = fix.cov[0, 0], fix.cov[0, 1], fix.cov[1, 0], fix.cov[1, 1]
        cov[5, 5] = fix.cov[2, 2]
        cov[2, 2] = 0.02 ** 2 if fix.z_source == 'ceiling' else 0.1 ** 2
        o.pose.covariance = [float(v) for v in cov.ravel()]
        self.pub_pose.publish(o)
        st = DiagnosticStatus()
        st.name = 'beam_localizer'
        st.level = DiagnosticStatus.OK if fix.healthy else DiagnosticStatus.WARN
        st.message = 'healthy' if fix.healthy else (fix.note or 'not flyable')
        st.values = [KeyValue(key='consistency', value=f'{fix.consistency:.2f}'),
                     KeyValue(key='spread', value=f'{math.sqrt(fix.cov[0, 0] + fix.cov[1, 1]):.3f}'),
                     KeyValue(key='z_source', value=fix.z_source),
                     KeyValue(key='ms', value=f'{self.loc.ms[-1]:.1f}')]
        self.pub_status.publish(st)
        self.pub_beams.publish(scan_msg(fix.beams, stamp, 'base_level', self.loc.p.beams.max_range))
        self.pub_sectors.publish(scan_msg(fix.sectors, stamp, 'base_level', self.loc.p.beams.max_range))
        self._n += 1
        if self.par['particles_every'] > 0 and self._n % self.par['particles_every'] == 0:
            pa = PoseArray()
            pa.header.stamp, pa.header.frame_id = stamp, 'map'
            xy, yaw = self.loc.loc.xy, self.loc.loc.yaw
            for k in range(0, len(xy), max(1, len(xy) // 150)):
                ps = Pose()
                ps.position.x, ps.position.y, ps.position.z = float(xy[k, 0]), float(xy[k, 1]), float(fix.pose[2])
                qx, qy, qz, qw = yaw_to_quat(yaw[k])
                ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = qx, qy, qz, qw
                pa.poses.append(ps)
            self.pub_particles.publish(pa)


def main():
    rclpy.init()
    node = BeamLocalizerNode()
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
