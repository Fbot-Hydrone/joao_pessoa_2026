#!/usr/bin/env python3
"""
phase3_gesture_node — thin ROS wrapper around the Phase 3 gesture mission core.

All the deciding lives in `gesture.mission.Mission` (pure Python, tested
headless in test/test_gesture_mission.py). This node only:

  in   /hydrone/lio/odom_raw          (odom -> base_link, the LIO pose)
       /cloud_registered              (FAST-LIO scan in camera_init = odom - lidar_mount)
       /hydrone/vision/human_gesture  (gesture_detector_node, one per frame)
       /hydrone/gesture/inject        (std_msgs/String, allow_inject only: a
                                       gesture name typed by hand stands in for
                                       the camera — the sim has no person in it)
       /mavros/state
  out  /mavros/setpoint_velocity/cmd_vel_unstamped at cmd_hz (ArduPilot drops
       GUIDED velocity control after ~1 s of silence)
       GUIDED / arm / takeoff / land over the MAVROS services
  dbg  /hydrone/gesture/state         (String: state, active gesture, blocks)
  srv  /hydrone/phase3/start          (std_srvs/Trigger) arms and takes off when
                                      auto_start is false — the real drone's way in

Frames as in phase4_maze_node: the core flies in `odom`, and vision_odom_bridge
feeds the EKF that pose rotated +90 deg about z, so a velocity goes out as
odom_to_enu(v).

Clock: gestures are timed on this node's clock (wall time in the sim too): how
long an arm is held is a human's time, not the simulator's.

Pilot override: once the mission has been flying in GUIDED, a mode change it
did not ask for (the safety pilot flipping to LOITER/STABILIZE/LAND) makes the
node go silent for good — no more setpoints, no more mode requests.
"""

import dataclasses
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

from hydrone_msgs.msg import HumanGesture
from hydrone_mission.gesture.mission import Mission, MissionParams, Observation
from hydrone_mission.phase4_maze_node import cloud_xyz, odom_to_enu, yaw_of

AIRBORNE = ("FORWARD", "TURN", "FIND", "GESTURE")


def body_obstacles(pts_odom, pos, yaw, band, r_min, r_max):
    """Points near flight level, in the body frame's xy: (N,2)."""
    rel = pts_odom - pos
    rel = rel[np.abs(rel[:, 2]) < band]
    c, s = math.cos(yaw), math.sin(yaw)
    xy = np.stack([c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]], axis=1)
    r = np.hypot(xy[:, 0], xy[:, 1])
    return xy[(r > r_min) & (r < r_max)]


class GestureMissionNode(Node):

    def __init__(self):
        super().__init__('phase3_gesture_node')
        own = [
            ('cmd_hz', 20.0),
            ('start_delay', 5.0),           # s of odometry before arming
            ('auto_start', True),
            ('odom_timeout', 0.5),
            ('lidar_mount', [0.0, 0.0, 0.1]),
            ('max_points', 20000),
            ('obstacle_band', 0.35),        # +- m around flight level
            ('obstacle_min_range', 0.35),   # the airframe itself
            ('obstacle_max_range', 5.0),
            ('allow_inject', False),
            ('inject_timeout', 1.0),
        ]
        mission_defaults = [(f.name, f.default) for f in dataclasses.fields(MissionParams)]
        p = self.declare_parameters('', own + mission_defaults)
        self.par = {q.name: q.value for q in p}
        self.mount = np.array(self.par['lidar_mount'], dtype=float)

        mp = MissionParams(**{k: type(d)(self.par[k]) for k, d in mission_defaults})
        self.mission = Mission(mp, logger=lambda m: self.get_logger().info(f'[PROVA3] {m}'))

        self.state = State()
        self.pose = None
        self.t_pose = None
        self.t_first = None
        self.obs = None
        self.inject = None
        self.obstacles, self.t_obstacles = None, None
        self.cmd = None
        self.started = False
        self.takeoff_done_seq = -1
        self.takeoff_z0 = None
        self.land_called = False
        self.flying_guided = False
        self.override = False
        self._last_call = {}

        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.create_subscription(Odometry, '/hydrone/lio/odom_raw', self._odom_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/cloud_registered', self._cloud_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(HumanGesture, '/hydrone/vision/human_gesture', self._gesture_cb, 10)
        if self.par['allow_inject']:
            self.create_subscription(String, '/hydrone/gesture/inject', self._inject_cb, 10)
            self.get_logger().warn('allow_inject: /hydrone/gesture/inject overrides the camera')

        self.pub_vel = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.pub_state = self.create_publisher(String, '/hydrone/gesture/state', 10)

        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.cli_land = self.create_client(CommandTOL, '/mavros/cmd/land')
        self._pending = []

        self.create_service(Trigger, '/hydrone/phase3/start', self._start_srv)

        self.create_timer(1.0 / max(self.par['cmd_hz'], 1.0), self._control)
        self.get_logger().info('phase3_gesture_node up: waiting for LIO odometry'
                               + ('' if self.par['auto_start'] else
                                  '; start with: ros2 service call /hydrone/phase3/start '
                                  'std_srvs/srv/Trigger'))

    def _start_srv(self, req, res):
        if self.pose is None or self.t_pose is None or self._now() - self.t_pose > self.par['odom_timeout']:
            res.success, res.message = False, 'no fresh LIO odometry yet'
        elif self.started:
            res.success, res.message = False, 'already started'
        else:
            self._begin()
            res.success, res.message = True, 'mission started'
        return res

    def _begin(self):
        self.started = True
        self.mission.start()
        self.get_logger().info('[PROVA3] starting the mission')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ── inputs ──────────────────────────────────────────────────────────────
    def _state_cb(self, m):
        if (self.flying_guided and not self.override and m.mode != 'GUIDED'
                and not self.land_called):
            self.override = True
            self.get_logger().error(f'[PROVA3] mode {m.mode} not ours: pilot override, '
                                    'mission stops commanding')
        self.state = m

    def _odom_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pose = (np.array([p.x, p.y, p.z]), yaw_of(q))
        self.t_pose = self._now()
        if self.t_first is None:
            self.t_first = self.t_pose

    def _cloud_cb(self, m):
        if self.pose is None:
            return
        n = m.width * m.height
        stride = max(1, int(math.ceil(n / max(self.par['max_points'], 1000))))
        pts = cloud_xyz(m, stride) + self.mount
        self.obstacles = body_obstacles(pts, self.pose[0], self.pose[1],
                                        self.par['obstacle_band'],
                                        self.par['obstacle_min_range'],
                                        self.par['obstacle_max_range'])
        self.t_obstacles = self._now()

    def _gesture_cb(self, m):
        self.obs = Observation(self._now(), m.gesture_name, bool(m.person_found), float(m.image_x))

    def _inject_cb(self, m):
        g = m.data.strip().upper()
        self.inject = Observation(self._now(), g, g not in ('', 'SEM_PESSOA'), 0.0)

    def _observation(self, t):
        if self.inject is not None and t - self.inject.t <= self.par['inject_timeout']:
            return self.inject
        return self.obs

    # ── commands ────────────────────────────────────────────────────────────
    def _call(self, cli, req, what, every=1.0):
        t = self._now()
        if t - self._last_call.get(what, -1e9) < every:
            return
        self._last_call[what] = t
        if not cli.service_is_ready():
            self.get_logger().warn(f'{what}: service not ready')
            return
        fut = cli.call_async(req)
        fut.add_done_callback(lambda f, w=what: self.get_logger().info(f'{w}: {f.result()}'))
        self._pending = [f for f in self._pending if not f.done()] + [fut]

    def _takeoff(self, cmd):
        if self.takeoff_done_seq == cmd.takeoff_seq:
            return
        if self.state.mode != 'GUIDED':
            self._call(self.cli_mode, SetMode.Request(custom_mode='GUIDED'), 'GUIDED')
            return
        if not self.state.armed:
            self._call(self.cli_arm, CommandBool.Request(value=True), 'arm')
            return
        self.takeoff_z0 = float(self.pose[0][2])
        alt = float(cmd.z) - self.takeoff_z0
        self.takeoff_done_seq = cmd.takeoff_seq
        self.land_called = False
        self.flying_guided = True
        self.get_logger().info(f'[PROVA3] takeoff to {alt:.2f} m')
        self._call(self.cli_takeoff, CommandTOL.Request(altitude=alt), 'takeoff', every=0.0)

    def _control(self):
        if self.pose is None or self.override:
            return
        t = self._now()
        if not self.started:
            if self.par['auto_start'] and t - self.t_first >= self.par['start_delay']:
                self._begin()
            return
        if t - self.t_pose > self.par['odom_timeout']:
            self.get_logger().warn('no LIO odometry — holding still', throttle_duration_sec=1.0)
            if self.mission.state in AIRBORNE:
                self.pub_vel.publish(Twist())
            return

        cmd = self.mission.step(t, self.pose, self._observation(t),
                                self.obstacles, self.t_obstacles, armed=self.state.armed)
        self.cmd = cmd
        if cmd.kind == 'takeoff':
            self._takeoff(cmd)
        elif cmd.kind == 'land':
            if not self.land_called:
                self.land_called = True
                self.get_logger().info('[PROVA3] landing')
                self._call(self.cli_land, CommandTOL.Request(), 'land', every=0.0)
        elif cmd.kind == 'velocity':
            enu = odom_to_enu(np.asarray(cmd.velocity, dtype=float))
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.linear.z = (float(a) for a in enu)
            msg.angular.z = float(cmd.yaw_rate)
            self.pub_vel.publish(msg)
        self.pub_state.publish(String(
            data=f'{cmd.state} {cmd.gesture}' + (f' blocked:{cmd.blocked}' if cmd.blocked else '')))


def main():
    rclpy.init()
    node = GestureMissionNode()
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
