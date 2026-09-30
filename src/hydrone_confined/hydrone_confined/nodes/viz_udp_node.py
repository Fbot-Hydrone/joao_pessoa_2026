#!/usr/bin/env python3
"""
viz_udp_node — ROS topics -> tools/maze_viewer (JSON over UDP, see core/viz_udp.py).

Works with any backend (BiguaSim, the kinematic sim, a bag): whatever of these
topics exist gets drawn.

in   <map_yaml>                    the known map (param)
     /hydrone/nav/odom             estimated pose (the one flying)
     /hydrone/nav/mode             pose source in use
     /beam_localizer/beams|sectors|particles
     /hydrone/maze/phase, /hydrone/maze/path
     /hydrone/qr                   letters read
     truth (param)                 ground truth, sim only: /hydrone/sim/truth, or
                                   BiguaSim's DynamicsSensor/Odom with truth_offset
     /hydrone/sim/qr_world         where the QR codes are (sim only, latched)
"""
import json
import math
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from hydrone_confined.core.beams import Beams
from hydrone_confined.core.known_map import KnownMap
from hydrone_confined.core.viz_udp import VizUdp
from hydrone_confined.nodes.rosutil import quat_to_rpy


def pose_of(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return np.array([p.x, p.y, p.z, quat_to_rpy(q.x, q.y, q.z, q.w)[2]])


def beams_of(m):
    ang = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
    return Beams(ang, np.array(m.ranges, dtype=float), np.zeros(len(m.ranges), dtype=int))


class VizUdpNode(Node):

    def __init__(self):
        super().__init__('viz_udp')
        p = {q.name: q.value for q in self.declare_parameters('', [
            ('map_yaml', ''),
            ('host', '127.0.0.1'),
            ('port', 47800),
            ('hz', 10.0),
            ('truth', '/hydrone/sim/truth'),
            ('truth_offset', [0.0, 0.0, 0.0, 0.0]),   # map <- truth frame (x, y, z, yaw_deg)
            ('mount_xyz', [0.0, 0.0, 0.1]),
        ])}
        self.par = p
        self.viz = VizUdp(p['host'], p['port'], hz=p['hz'])
        if p['map_yaml']:
            self.viz.set_map(KnownMap.from_yaml(p['map_yaml']))
        self.viz.reset()
        self.est = self.truth = None
        self.mode, self.phase = '', ''
        self.beams = self.sectors = None
        self.particles = None
        self.path = None
        self.qr_seen = []
        self.qr_world = None
        sq = qos_profile_sensor_data
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=20)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/hydrone/nav/odom', lambda m: setattr(self, 'est', pose_of(m)), sq)
        self.create_subscription(Odometry, p['truth'], self._truth, sq)
        self.create_subscription(String, '/hydrone/nav/mode', lambda m: setattr(self, 'mode', m.data), 10)
        self.create_subscription(String, '/hydrone/maze/phase', lambda m: setattr(self, 'phase', m.data), 10)
        self.create_subscription(LaserScan, '/beam_localizer/beams',
                                 lambda m: setattr(self, 'beams', beams_of(m)), sq)
        self.create_subscription(LaserScan, '/beam_localizer/sectors',
                                 lambda m: setattr(self, 'sectors', beams_of(m)), sq)
        self.create_subscription(PoseArray, '/beam_localizer/particles', self._particles, sq)
        self.create_subscription(Path, '/hydrone/maze/path', self._path, 10)
        self.create_subscription(String, '/hydrone/qr', self._qr, rel)
        self.create_subscription(String, '/hydrone/sim/qr_world', self._qr_world, latched)
        self.create_timer(1.0 / max(p['hz'], 1.0), self._send)

    def _truth(self, m):
        tp = pose_of(m)
        ox, oy, oz, oyaw = self.par['truth_offset']
        a = math.radians(oyaw)
        c, s = math.cos(a), math.sin(a)
        self.truth = np.array([ox + c * tp[0] - s * tp[1], oy + s * tp[0] + c * tp[1], oz + tp[2], tp[3] + a])

    def _particles(self, m):
        self.particles = np.array([[q.position.x, q.position.y] for q in m.poses]) if m.poses else None

    def _path(self, m):
        self.path = np.array([[q.pose.position.x, q.pose.position.y] for q in m.poses]) if m.poses else None

    def _qr(self, m):
        if m.data not in self.qr_seen:
            self.qr_seen.append(m.data)

    def _qr_world(self, m):
        self.qr_world = {k: np.array(v) for k, v in json.loads(m.data).items()}

    def _send(self):
        if self.est is None:
            return
        fix = SimpleNamespace(beams=self.beams, sectors=self.sectors)
        self.viz.frame(self.get_clock().now().nanoseconds * 1e-9, self.est, phase=self.phase, mode=self.mode,
                       truth=self.truth, fix=fix, particles=self.particles, path=self.path,
                       qr_world=self.qr_world, qr_seen=self.qr_seen, mount=self.par['mount_xyz'][:2])


def main():
    rclpy.init()
    node = VizUdpNode()
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
