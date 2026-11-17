#!/usr/bin/env python3
"""
kinematic_sim_node — the simple backend: a kinematic drone in a box world,
speaking the same topics as BiguaSim + ArduPilot SITL + MAVROS + FAST-LIO.

World: a KnownMap YAML, built into boxes (sim/world.py), optionally with
passages moved or walled up and QR codes scattered on inner walls.

out  <cloud>                       PointCloud2 in the sensor frame, tilted with the body
     /mavros/imu/data              attitude (roll/pitch from the acceleration)
     /mavros/state                 connected / armed / mode
     /mavros/home_position/home    once a second (the mock's EKF is always ready)
     /mavros/local_position/pose   what the FCU believes (echo of vision_pose)
     <odom_raw>, <odom>            drifting LIO-like odometry in 'odom' (optionally
                                   sliding along a corridor while inside)
     /hydrone/qr                   std_msgs/String, a letter each time one is in view
     /hydrone/sim/truth            nav_msgs/Odometry, ground truth in the map frame
     /hydrone/sim/qr_world         std_msgs/String (JSON, latched): where the QR codes are
in   /mavros/setpoint_velocity/cmd_vel_unstamped   ENU velocity, run in the FCU's
                                   own frame estimate (from /mavros/vision_pose/pose)
     /mavros/set_mode, /mavros/cmd/arming, /mavros/cmd/takeoff, /mavros/cmd/land

Wall-clock time, real time. A 0.1 s step, lidar every step.
"""
import json
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import HomePosition, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header, String

from hydrone_confined.core.beams import rot_rp
from hydrone_confined.core.known_map import KnownMap
from hydrone_confined.nodes.rosutil import enu_to_odom, quat_to_rpy
from hydrone_confined.sim import lidar
from hydrone_confined.sim.runner import BODY_DOWN, LIDAR_UP, SimParams, clearance, inside, qr_in_view, rz
from hydrone_confined.sim.world import from_known_map


def euler_quat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def pc2(points, stamp, frame):
    m = PointCloud2()
    m.header = Header(stamp=stamp, frame_id=frame)
    m.height, m.width = 1, len(points)
    m.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                for k, n in enumerate('xyz')]
    m.is_bigendian, m.point_step = False, 12
    m.row_step = 12 * len(points)
    m.is_dense = True
    m.data = np.asarray(points, dtype=np.float32).tobytes()
    return m


class KinematicSimNode(Node):

    def __init__(self):
        super().__init__('kinematic_sim')
        p = {q.name: q.value for q in self.declare_parameters('', [
            ('map_yaml', ''),
            ('cloud', '/livox/lidar_pc'),
            ('odom_raw', '/hydrone/lio/odom_raw'),
            ('odom', '/hydrone/lio/odom'),
            ('n_rays', 9000),
            ('dt', 0.1),
            ('qr', 5),                      # QR codes scattered on inner walls
            ('seed', 1),
            ('yaw_error_deg', 0.0),         # the drone isn't quite where the map says it starts
            ('xy_error', [0.0, 0.0]),
            ('closed', ['']),               # passages walled up in the world (not in the map)
            ('degen_rate', 0.0),            # m/s of LIO sliding while inside
            ('pose_drift', 0.002),
        ])}
        self.par = p
        km = KnownMap.from_yaml(p['map_yaml'])
        perturb = {n: None for n in p['closed'] if n}
        self.world = from_known_map(km, yaw_error=math.radians(p['yaw_error_deg']),
                                    xy_error=tuple(p['xy_error']), perturb=perturb, qr=int(p['qr']),
                                    seed=int(p['seed']))
        self.sp = SimParams(n_rays=int(p['n_rays']), dt=float(p['dt']))
        self.rng = np.random.default_rng(int(p['seed']))
        w = self.world
        self.p = w.takeoff.astype(float).copy()
        self.v = np.zeros(3)
        self.v_prev = np.zeros(3)
        self.psi, self.wz = 0.0, 0.0
        self.bias = np.zeros(3)
        self.degen = np.zeros(3)
        self.degen_sign = 1.0
        self.armed, self.mode = False, 'STABILIZE'
        self.cmd = np.zeros(3)
        self.cmd_wz = 0.0
        self.t_cmd = -1e9
        self.target_z = None
        self.landing = False
        self.est_yaw_enu = None
        self.est_pose = None
        self.qr_sent = set()
        self.crashed = False

        sq = qos_profile_sensor_data
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.pub_cloud = self.create_publisher(PointCloud2, p['cloud'], sq)
        self.pub_imu = self.create_publisher(Imu, '/mavros/imu/data', sq)
        self.pub_state = self.create_publisher(State, '/mavros/state', rel)
        self.pub_local = self.create_publisher(PoseStamped, '/mavros/local_position/pose', sq)
        self.pub_odom_raw = self.create_publisher(Odometry, p['odom_raw'], sq)
        self.pub_odom = self.create_publisher(Odometry, p['odom'], sq)
        self.pub_qr = self.create_publisher(String, '/hydrone/qr', rel)
        self.pub_home = self.create_publisher(HomePosition, '/mavros/home_position/home', sq)
        self.pub_truth = self.create_publisher(Odometry, '/hydrone/sim/truth', sq)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_qr_world = self.create_publisher(String, '/hydrone/sim/qr_world', latched)
        self.pub_qr_world.publish(String(data=json.dumps(
            {k: [float(v[0][0]), float(v[0][1])] for k, v in w.meta['qr'].items()})))
        self.create_subscription(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', self._vel, 10)
        self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self._vision, rel)
        self.create_service(SetMode, '/mavros/set_mode', self._set_mode)
        self.create_service(CommandBool, '/mavros/cmd/arming', self._arming)
        self.create_service(CommandTOL, '/mavros/cmd/takeoff', self._takeoff)
        self.create_service(CommandTOL, '/mavros/cmd/land', self._land)
        self.create_timer(self.sp.dt, self._step)
        self.create_timer(1.0, self._state)
        self.get_logger().info(f'kinematic sim: {km.name}, QR {sorted(w.meta["qr"])}, closed {list(perturb)}')

    # ── FCU mock ────────────────────────────────────────────────────────────
    def _state(self):
        self.pub_home.publish(HomePosition())       # the mock's EKF is always up
        self.pub_state.publish(State(connected=True, armed=self.armed, guided=self.mode == 'GUIDED',
                                     mode=self.mode))

    def _set_mode(self, req, res):
        self.mode = req.custom_mode
        res.mode_sent = True
        self._state()
        return res

    def _arming(self, req, res):
        self.armed = bool(req.value)
        res.success, res.result = True, 0
        self._state()
        return res

    def _takeoff(self, req, res):
        ok = self.armed and self.mode == 'GUIDED'
        if ok:
            self.target_z = self.p[2] + float(req.altitude)
        res.success, res.result = ok, 0 if ok else 4
        return res

    def _land(self, req, res):
        self.landing = True
        self.mode = 'LAND'
        res.success, res.result = True, 0
        return res

    def _vel(self, m):
        self.cmd = np.array([m.linear.x, m.linear.y, m.linear.z])
        self.cmd_wz = m.angular.z
        self.t_cmd = self.get_clock().now().nanoseconds * 1e-9
        self.target_z = None

    def _vision(self, m):
        q = m.pose.orientation
        self.est_yaw_enu = quat_to_rpy(q.x, q.y, q.z, q.w)[2]
        self.est_pose = m
        self.pub_local.publish(m)

    # ── physics ─────────────────────────────────────────────────────────────
    def _step(self):
        sp, w = self.sp, self.world
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        yaw_true = w.yaw0 + self.psi
        want = np.zeros(3)
        w_want = 0.0
        if self.armed and self.landing:
            want = np.array([0.0, 0.0, -0.3])
        elif self.armed and self.target_z is not None:
            want = np.array([0.0, 0.0, float(np.clip(self.target_z - self.p[2], -0.5, 0.5))])
        elif self.armed and self.mode == 'GUIDED' and t - self.t_cmd < 1.0:
            # ENU -> the FCU's own nav frame -> body (its yaw estimate) -> world (true yaw)
            v_nav = enu_to_odom(self.cmd)
            est = (self.est_yaw_enu - math.pi / 2) if self.est_yaw_enu is not None else yaw_true
            want = rz(yaw_true) @ rz(-est) @ v_nav
            w_want = self.cmd_wz
        self.v_prev = self.v.copy()
        a = min(sp.dt / sp.tau, 1.0)
        self.v += (want - self.v) * a + self.rng.normal(0, sp.accel_noise * math.sqrt(sp.dt), 3)
        self.wz += (w_want - self.wz) * min(sp.dt / sp.yaw_tau, 1.0)
        self.psi += self.wz * sp.dt
        if not self.armed:
            self.v[:] = 0.0
        self.p = self.p + self.v * sp.dt
        support = w.support_z(self.p[:2])
        if self.p[2] - BODY_DOWN < support:
            self.p[2] = support + BODY_DOWN
            self.v[:] = 0.0
            if self.landing and self.armed:
                self.armed, self.landing = False, False
                self.get_logger().info(f'landed at {np.round(self.p[:2], 2)}')
        if self.armed and clearance(w, self.p) < 0 and not self.crashed:
            self.crashed = True
            self.get_logger().error(f'CONTACT at {np.round(self.p, 2)}')
        stamp = now.to_msg()
        yaw_true = w.yaw0 + self.psi
        # attitude from the acceleration
        acc = rz(-yaw_true) @ ((self.v - self.v_prev) / sp.dt)
        pitch, roll = math.atan2(acc[0], 9.81), -math.atan2(acc[1], 9.81)
        imu = Imu()
        imu.header = Header(stamp=stamp, frame_id='base_link')
        # ENU orientation: map yaw + 90 deg, the bridge's convention
        qx, qy, qz, qw = euler_quat(roll, pitch, yaw_true + math.pi / 2)
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = qx, qy, qz, qw
        self.pub_imu.publish(imu)
        # LIO-like odometry
        self.bias += self.rng.normal(0, float(self.par['pose_drift']) * math.sqrt(sp.dt), 3) * np.array([1, 1, 0.3])
        if self.par['degen_rate'] > 0 and inside(w, self.p[:2]):
            if self.rng.random() < sp.dt / 8.0:
                self.degen_sign = -self.degen_sign
            self.degen += np.array([1.0, 0.0, 0.0]) * self.degen_sign * self.par['degen_rate'] * sp.dt
        est = w.to_odom(self.p + self.bias + self.degen)
        o = Odometry()
        o.header = Header(stamp=stamp, frame_id='odom')
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = (float(v) for v in est)
        qx, qy, qz, qw = euler_quat(0.0, 0.0, self.psi)
        o.pose.pose.orientation.x, o.pose.pose.orientation.y = qx, qy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        vb = rz(-yaw_true) @ self.v
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = (float(v) for v in vb)
        self.pub_odom_raw.publish(o)
        self.pub_odom.publish(o)
        tr = Odometry()
        tr.header = Header(stamp=stamp, frame_id='map')
        tr.child_frame_id = 'base_link'
        tr.pose.pose.position.x, tr.pose.pose.position.y, tr.pose.pose.position.z = (float(v) for v in self.p)
        qx, qy, qz, qw = euler_quat(0.0, 0.0, yaw_true)
        tr.pose.pose.orientation.x, tr.pose.pose.orientation.y = qx, qy
        tr.pose.pose.orientation.z, tr.pose.pose.orientation.w = qz, qw
        self.pub_truth.publish(tr)
        # the lidar, in its own tilted frame
        origin = self.p + np.array([0, 0, LIDAR_UP])
        pts = lidar.scan(origin, w.lo, w.hi, sp.n_rays, self.rng, sp.range_noise)
        body = ((pts - origin) @ rz(yaw_true)) @ rot_rp(roll, pitch)
        self.pub_cloud.publish(pc2(body, stamp, 'livox_frame'))
        for q in qr_in_view(w, self.p + np.array([0, 0, 0.05]), yaw_true, sp):
            self.pub_qr.publish(String(data=q))


def main():
    rclpy.init()
    node = KinematicSimNode()
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
