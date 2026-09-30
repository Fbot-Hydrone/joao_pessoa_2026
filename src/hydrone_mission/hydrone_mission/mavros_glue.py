"""The MAVROS side every mission core needs, once.

A mission core returns Commands (velocity / hold / takeoff / land) in its
planning frame. This base node turns them into MAVROS:
  - arm, GUIDED and takeoff over the services, one step per command tick
  - land once
  - a velocity setpoint every cmd_hz tick, whatever else is happening:
    ArduPilot drops GUIDED velocity control after ~1 s of silence
  - hold still when the pose or the sensor stream goes stale

Frames: the planning frame is whatever vision_odom_bridge feeds the EKF (the
LIO odom, or the map frame through pose_mux). The bridge turns it +90 deg into
MAVROS' local ENU, so velocities go out the same way (odom_to_enu).
"""
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from hydrone_confined.nodes.rosutil import odom_to_enu


class MavrosMissionNode(Node):

    def __init__(self, name, extra_params=()):
        super().__init__(name)
        p = self.declare_parameters('', [
            ('cmd_hz', 20.0),
            ('start_delay', 5.0),               # s of pose before arming
            ('stale_s', 2.0),                   # no sensor tick for this long -> hold
            ('auto_start', True),
        ] + list(extra_params))
        self.par = {q.name: q.value for q in p}
        self.state = State()
        self.local = None
        self.pose = None                        # latest pose in the planning frame
        self.t_pose = None
        self.t_first = None
        self.t_tick = None                      # last time the core was stepped
        self.cmd = None
        self.armed_z = None
        self.started = False
        self.takeoff_sent = False
        self.land_sent = False
        self.finished = False
        self._pending = []
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._local_cb,
                                 qos_profile_sensor_data)
        self.pub_vel = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.cli_land = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.create_timer(1.0 / max(self.par['cmd_hz'], 1.0), self._control)

    def _state_cb(self, m):
        self.state = m

    def _local_cb(self, m):
        self.local = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])

    def set_pose(self, t, pose):
        self.pose, self.t_pose = pose, t
        if self.t_first is None:
            self.t_first = t

    def _call(self, cli, req, what):
        if not cli.service_is_ready():
            self.get_logger().warn(f'{what}: service not ready')
            return
        fut = cli.call_async(req)
        fut.add_done_callback(lambda f, w=what: self.get_logger().info(f'{w}: {f.result()}'))
        self._pending.append(fut)

    def act(self, cmd, t):
        """Take a core command: MAVROS calls now, the velocity on the timer."""
        self.cmd, self.t_tick = cmd, t
        if cmd.kind == 'takeoff':
            if not self.started:
                return
            if self.state.mode != 'GUIDED':
                self._call(self.cli_mode, SetMode.Request(custom_mode='GUIDED'), 'GUIDED')
                return
            if not self.state.armed:
                self._call(self.cli_arm, CommandBool.Request(value=True), 'arm')
                return
            if self.armed_z is None:
                self.armed_z = float(self.pose[0][2])
            if not self.takeoff_sent:
                alt = float(cmd.z) - self.armed_z
                self.takeoff_sent = True
                self.get_logger().info(f'takeoff to z {cmd.z:.2f} ({alt:.2f} m above here)')
                self._call(self.cli_takeoff, CommandTOL.Request(altitude=alt), 'takeoff')
        elif cmd.kind == 'land':
            if not self.land_sent:
                self.land_sent = True
                self.finished = True
                self.get_logger().info('landing')
                self._call(self.cli_land, CommandTOL.Request(), 'land')

    def _control(self):
        """Publish a velocity setpoint every tick, whatever else is going on."""
        if self.pose is None:
            return
        if not self.started:
            if self.par['auto_start'] and self.t_pose - self.t_first >= self.par['start_delay']:
                self.started = True
                self.get_logger().info('starting the mission')
            return
        if self.land_sent:
            return
        # takeoff climbs on the FCU's own ramp; don't fight it with velocities
        if self.cmd is not None and self.cmd.kind == 'takeoff':
            return
        v = np.zeros(3)
        wz = 0.0
        stale = self.t_tick is None or (self.t_pose - self.t_tick) > self.par['stale_s']
        if stale:
            if self.t_tick is not None:
                self.get_logger().warn('no sensor tick for too long — holding still',
                                       throttle_duration_sec=2.0)
        elif self.cmd is not None and self.cmd.kind == 'velocity':
            v = np.asarray(self.cmd.velocity, dtype=float)
            wz = float(self.cmd.yaw_rate or 0.0)
        enu = odom_to_enu(v)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = (float(a) for a in enu)
        msg.angular.z = wz
        self.pub_vel.publish(msg)
