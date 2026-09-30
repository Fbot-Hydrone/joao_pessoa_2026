#!/usr/bin/env python3
"""
phase4_known_node — thin ROS wrapper around the known-map Phase 4 mission
(hydrone_mission/phase4/known_mission.py): fly the rulebook maze by its walls.

in   /hydrone/nav/odom            the pose that flies (pose_mux), map frame
     /hydrone/nav/mode            which source the mux is using
     /beam_localizer/beams        levelled beams (passage checks)
     /beam_localizer/sectors      guard sectors
     /beam_localizer/status       healthy or not
     /hydrone/qr                  std_msgs/String, a QR letter each time one is read
     <cloud> + /mavros/imu/data   only for the explorer fallback's map
out  /hydrone/nav/source          'odom' | 'map', what the mission wants the mux on
     MAVROS velocity / arm / GUIDED / takeoff / land (mavros_glue)
dbg  /hydrone/maze/{phase,path,markers}

The core is stepped on each beams message (one per scan).
"""
import math
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from hydrone_confined.core.beams import Beams, level
from hydrone_confined.core.known_map import KnownMap
from hydrone_confined.nodes.rosutil import cloud_xyz, quat_to_rpy, stamp_s
from hydrone_mission.mavros_glue import MavrosMissionNode
from hydrone_mission.phase4.explore_mission import Mission, MissionParams
from hydrone_mission.phase4.known_mission import HANDOVER_BANDS, KnownMission, KnownParams


def beams_of(m):
    n = len(m.ranges)
    ang = m.angle_min + np.arange(n) * m.angle_increment
    r = np.array(m.ranges, dtype=float)
    return Beams(ang, r, np.isfinite(r).astype(int))


class KnownNode(MavrosMissionNode):

    def __init__(self):
        super().__init__('phase4_known_node', [
            ('map_yaml', ''),
            ('out_z', 1.1),
            ('cruise_z', 0.75),
            ('v_in', 0.25),
            ('v_out', 0.5),
            ('v_crawl', 0.15),
            ('spin', True),
            ('qr_target', 5),
            ('explore_fallback', True),
            ('cloud', '/livox/lidar_pc'),
            ('lidar_mount', [0.0, 0.0, 0.1]),
            ('max_points', 15000),
            ('debug_hz', 2.0),
        ])
        p = self.par
        self.km = KnownMap.from_yaml(p['map_yaml'])
        kp = KnownParams(out_z=p['out_z'], cruise_z=p['cruise_z'], v_in=p['v_in'], v_out=p['v_out'],
                         v_crawl=p['v_crawl'], spin=bool(p['spin']), qr_target=int(p['qr_target']),
                         explore_fallback=bool(p['explore_fallback']))
        ex = Mission(MissionParams(flight_z=kp.out_z, inside_bands=HANDOVER_BANDS)) if kp.explore_fallback else None
        self.mission = KnownMission(self.km, kp, explore=ex,
                                    logger=lambda m: self.get_logger().info(f'[known] {m}'))
        self.mount = np.array(p['lidar_mount'], dtype=float)
        self.sectors = None
        self.healthy = False
        self.mode = 'odom'
        self.qr_new = []
        self.att = (0.0, 0.0)
        self.scan_body = None
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=20)
        sq = qos_profile_sensor_data
        self.create_subscription(Odometry, '/hydrone/nav/odom', self._pose_cb, sq)
        self.create_subscription(String, '/hydrone/nav/mode', lambda m: setattr(self, 'mode', m.data), 10)
        self.create_subscription(LaserScan, '/beam_localizer/beams', self._beams_cb, sq)
        self.create_subscription(LaserScan, '/beam_localizer/sectors', self._sectors_cb, sq)
        self.create_subscription(DiagnosticStatus, '/beam_localizer/status', self._status_cb, 10)
        self.create_subscription(String, '/hydrone/qr', lambda m: self.qr_new.append(m.data), rel)
        if kp.explore_fallback:
            self.create_subscription(PointCloud2, p['cloud'], self._cloud_cb, sq)
            self.create_subscription(Imu, '/mavros/imu/data', self._imu_cb, sq)
        self.pub_source = self.create_publisher(String, '/hydrone/nav/source', 10)
        self.pub_phase = self.create_publisher(String, '/hydrone/maze/phase', 10)
        self.pub_path = self.create_publisher(Path, '/hydrone/maze/path', 1)
        self.pub_mark = self.create_publisher(MarkerArray, '/hydrone/maze/markers', 1)
        self.create_timer(1.0 / max(p['debug_hz'], 0.1), self._publish_debug)
        self.get_logger().info(f'phase4_known_node up on {self.km.name}: waiting for the nav pose')

    # ── inputs ──────────────────────────────────────────────────────────────
    def _pose_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = quat_to_rpy(q.x, q.y, q.z, q.w)[2]
        # the glue wants (xyz, yaw); the core gets (x, y, z, yaw)
        self.set_pose(stamp_s(m.header), (np.array([p.x, p.y, p.z]), yaw))

    def _status_cb(self, m):
        self.healthy = m.level == DiagnosticStatus.OK

    def _sectors_cb(self, m):
        self.sectors = beams_of(m)

    def _imu_cb(self, m):
        q = m.orientation
        r, pch, _ = quat_to_rpy(q.x, q.y, q.z, q.w)
        self.att = (r, pch)

    def _cloud_cb(self, m):
        n = m.width * m.height
        stride = max(1, int(math.ceil(n / max(self.par['max_points'], 1000))))
        self.scan_body = cloud_xyz(m, stride)

    def _scan_nav(self):
        """The latest scan, levelled and put in the map frame through the nav pose."""
        if self.scan_body is None or self.pose is None:
            return None
        lvl = level(self.scan_body, *self.att)
        lvl = lvl[np.isfinite(lvl).all(axis=1)]
        xyz, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        origin = np.r_[xyz[:2] + R @ self.mount[:2], xyz[2] + self.mount[2]]
        pts = np.column_stack([lvl[:, :2] @ R.T + origin[:2], lvl[:, 2] + origin[2]])
        self.scan_body = None
        return pts, origin

    def _beams_cb(self, m):
        if self.pose is None or self.finished:
            return
        t = stamp_s(m.header)
        fix = SimpleNamespace(beams=beams_of(m), sectors=self.sectors, healthy=self.healthy)
        xyz, yaw = self.pose
        pose = np.array([xyz[0], xyz[1], xyz[2], yaw])
        qr, self.qr_new = self.qr_new, []
        cmd = self.mission.step(t, pose, fix, qr, self._scan_nav(), self.mode)
        self.pub_source.publish(String(data=cmd.source))
        self.act(cmd, t)

    # ── debug ───────────────────────────────────────────────────────────────
    def _publish_debug(self):
        if self.cmd is None:
            return
        d = self.cmd.debug
        self.pub_phase.publish(String(data=f"{d['phase']} src={d['source']} qr={''.join(d['qr'])} "
                                           f"{'|'.join(d['fallback_used'])}"))
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        z = float(self.pose[0][2]) if self.pose is not None else 0.0
        for q in (d['path'] if d.get('path') is not None else []):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(q[0]), float(q[1]), z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_path.publish(path)
        arr = MarkerArray()
        for k, pr in enumerate(self.km.passages()):
            mk = Marker()
            mk.header = path.header
            mk.ns, mk.id, mk.type, mk.action = 'passages', k, Marker.CUBE, Marker.ADD
            mk.pose.position.x, mk.pose.position.y = float(pr.center[0]), float(pr.center[1])
            mk.pose.position.z = float(pr.opening.pass_z)
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = 0.12
            mk.scale.z = float(pr.opening.z[1] - pr.opening.z[0])
            col = {'entry': (0.2, 1.0, 0.3), 'exit': (1.0, 0.3, 1.0), 'forbidden': (1.0, 0.2, 0.2)}.get(
                pr.opening.role, (1.0, 0.6, 0.1))
            mk.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=0.7)
            arr.markers.append(mk)
        if d.get('goal') is not None:
            mk = Marker()
            mk.header = path.header
            mk.ns, mk.id, mk.type, mk.action = 'goal', 0, Marker.SPHERE, Marker.ADD
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(d['goal'][0]), float(d['goal'][1]), z
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = 0.15
            mk.color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=1.0)
            arr.markers.append(mk)
        self.pub_mark.publish(arr)


def main():
    rclpy.init()
    node = KnownNode()
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
