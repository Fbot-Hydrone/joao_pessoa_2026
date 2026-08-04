#!/usr/bin/env python3
"""
hydrone_controller — High-level drone control node.

Bridges between mission/nav commands and the ArduPilot flight controller
via MAVROS.  Exposes simple action-like services so the rest of the stack
never has to know about MAVLink details.

Subscribed topics:
  /mavros/state                     (mavros_msgs/State)
  /mavros/local_position/pose       (geometry_msgs/PoseStamped)
  /mavros/imu/data                  (sensor_msgs/Imu)
  /hydrone/controller/cmd_pose      (geometry_msgs/PoseStamped)  — setpoint
  /hydrone/controller/cmd_vel       (geometry_msgs/Twist)        — velocity override

Published topics:
  /mavros/setpoint_position/local   (geometry_msgs/PoseStamped)
  /mavros/setpoint_velocity/cmd_vel (geometry_msgs/TwistStamped)
  /hydrone/controller/drone_pose    (geometry_msgs/PoseStamped)  — current pose
  /hydrone/controller/status        (std_msgs/String)            — state string

Services exposed:
  /hydrone/controller/arm           (std_srvs/Trigger)
  /hydrone/controller/disarm        (std_srvs/Trigger)
  /hydrone/controller/takeoff       (std_srvs/SetBool)  — data = True → guided
  /hydrone/controller/land          (std_srvs/Trigger)
  /hydrone/controller/set_mode      (mavros_msgs/SetMode)  — forwarded
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Vector3
from sensor_msgs.msg    import Imu
from std_msgs.msg       import String
from std_srvs.srv       import Trigger, SetBool

# MAVROS messages
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


# ── Constants ────────────────────────────────────────────────────────────────

TAKEOFF_HEIGHT_M   = 1.2    # default hover altitude (metres)
LAND_DESCENT_SPEED = 0.3    # m/s descent when landing
POSITION_TOLERANCE = 0.12   # metres — "arrived at setpoint"
YAW_TOLERANCE_RAD  = 0.08   # radians

SETPOINT_HZ = 20            # Hz — MAVROS requires continuous setpoints

# Service/command timeout defaults, in seconds. These are monotonic-clock
# budgets, so they do NOT stretch when the simulator runs below real-time —
# hydrone_bringup/config/timeouts.yaml overrides them with the sim values.
SERVICE_WAIT_TIMEOUT = 3.0   # waiting for a MAVROS service to appear
ARM_TIMEOUT          = 5.0   # waiting for the arm/disarm COMMAND_ACK
LAND_TIMEOUT         = 10.0  # waiting for the land COMMAND_ACK


class HydroneControllerNode(Node):

    def __init__(self):
        super().__init__("hydrone_controller")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("takeoff_height", TAKEOFF_HEIGHT_M)
        self.declare_parameter("position_tolerance", POSITION_TOLERANCE)
        self.declare_parameter("setpoint_hz", SETPOINT_HZ)
        self.declare_parameter("service_wait_timeout", SERVICE_WAIT_TIMEOUT)
        self.declare_parameter("arm_timeout", ARM_TIMEOUT)
        self.declare_parameter("land_timeout", LAND_TIMEOUT)

        self.takeoff_height  = self.get_parameter("takeoff_height").value
        self.pos_tol         = self.get_parameter("position_tolerance").value
        hz                   = self.get_parameter("setpoint_hz").value
        self.svc_wait_tmo    = self.get_parameter("service_wait_timeout").value
        self.arm_tmo         = self.get_parameter("arm_timeout").value
        self.land_tmo        = self.get_parameter("land_timeout").value

        # ── Internal state ──────────────────────────────────────────────────
        self.mav_state       = State()
        self.current_pose    = PoseStamped()
        self.target_pose     = PoseStamped()
        self.vel_override    = None          # Twist or None
        self._setpoint_mode  = "position"    # "position" | "velocity"
        self._is_landed      = True
        self._reach_cb       = None          # optional callback when setpoint reached

        # ── QoS ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ─────────────────────────────────────────────────────
        self.sub_state = self.create_subscription(
            State, "/mavros/state", self._cb_mav_state, 10)
        self.sub_pose = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._cb_pose, sensor_qos)
        self.sub_cmd_pose = self.create_subscription(
            PoseStamped, "/hydrone/controller/cmd_pose",
            self._cb_cmd_pose, 10)
        self.sub_cmd_vel = self.create_subscription(
            Twist, "/hydrone/controller/cmd_vel",
            self._cb_cmd_vel, 10)

        # ── Publishers ──────────────────────────────────────────────────────
        self.pub_setpoint = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self.pub_vel = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        self.pub_drone_pose = self.create_publisher(
            PoseStamped, "/hydrone/controller/drone_pose", sensor_qos)
        self.pub_status = self.create_publisher(
            String, "/hydrone/controller/status", 10)

        # ── Service clients (MAVROS) ─────────────────────────────────────────
        self.cli_arm   = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_mode  = self.create_client(SetMode,     "/mavros/set_mode")
        self.cli_land  = self.create_client(CommandTOL,  "/mavros/cmd/land")

        # ── Services exposed ─────────────────────────────────────────────────
        self.srv_arm      = self.create_service(
            Trigger, "/hydrone/controller/arm",     self._svc_arm)
        self.srv_disarm   = self.create_service(
            Trigger, "/hydrone/controller/disarm",  self._svc_disarm)
        self.srv_takeoff  = self.create_service(
            SetBool, "/hydrone/controller/takeoff", self._svc_takeoff)
        self.srv_land     = self.create_service(
            Trigger, "/hydrone/controller/land",    self._svc_land)

        # ── Timer — continuously publish setpoint so MAVROS stays happy ─────
        self.timer = self.create_timer(1.0 / hz, self._publish_setpoint)

        self.get_logger().info("HydroneControllerNode ready.")

    # ────────────────────────────────────────────────────────────────────────
    # Subscriber callbacks
    # ────────────────────────────────────────────────────────────────────────

    def _cb_mav_state(self, msg: State):
        self.mav_state = msg

    def _cb_pose(self, msg: PoseStamped):
        self.current_pose = msg
        self.pub_drone_pose.publish(msg)
        self._check_setpoint_reached()

    def _cb_cmd_pose(self, msg: PoseStamped):
        """External setpoint from nav or mission nodes."""
        self.target_pose    = msg
        self._setpoint_mode = "position"
        self.vel_override   = None

    def _cb_cmd_vel(self, msg: Twist):
        """Velocity override (e.g. slow landing descent)."""
        self.vel_override   = msg
        self._setpoint_mode = "velocity"

    # ────────────────────────────────────────────────────────────────────────
    # Setpoint publisher (timer)
    # ────────────────────────────────────────────────────────────────────────

    def _publish_setpoint(self):
        now = self.get_clock().now().to_msg()

        if self._setpoint_mode == "velocity" and self.vel_override is not None:
            ts            = TwistStamped()
            ts.header.stamp    = now
            ts.header.frame_id = "base_link"
            ts.twist           = self.vel_override
            self.pub_vel.publish(ts)
        else:
            self.target_pose.header.stamp = now
            self.pub_setpoint.publish(self.target_pose)

        # Status string
        status = (f"mode={self.mav_state.mode} "
                  f"armed={self.mav_state.armed} "
                  f"landed={self._is_landed}")
        self.pub_status.publish(String(data=status))

    # ────────────────────────────────────────────────────────────────────────
    # Setpoint reached detection
    # ────────────────────────────────────────────────────────────────────────

    def _check_setpoint_reached(self):
        if self._reach_cb is None:
            return
        dx = self.current_pose.pose.position.x - self.target_pose.pose.position.x
        dy = self.current_pose.pose.position.y - self.target_pose.pose.position.y
        dz = self.current_pose.pose.position.z - self.target_pose.pose.position.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < self.pos_tol:
            cb = self._reach_cb
            self._reach_cb = None   # one-shot
            cb()

    def go_to(self, x: float, y: float, z: float,
              yaw: float = 0.0, on_reached=None):
        """
        Command the drone to move to (x, y, z) in local frame.
        Optional callback fired when position is reached.
        """
        sp                       = PoseStamped()
        sp.header.frame_id       = "map"
        sp.pose.position.x       = x
        sp.pose.position.y       = y
        sp.pose.position.z       = z
        sp.pose.orientation.z    = math.sin(yaw / 2.0)
        sp.pose.orientation.w    = math.cos(yaw / 2.0)
        self.target_pose         = sp
        self._setpoint_mode      = "position"
        self.vel_override        = None
        self._reach_cb           = on_reached

    # ────────────────────────────────────────────────────────────────────────
    # Service handlers
    # ────────────────────────────────────────────────────────────────────────

    def _svc_arm(self, request, response):
        if not self.cli_arm.wait_for_service(timeout_sec=self.svc_wait_tmo):
            response.success = False
            response.message = "ARM service unavailable"
            return response

        # Switch to GUIDED first
        mode_req          = SetMode.Request()
        mode_req.custom_mode = "GUIDED"
        self.cli_mode.call_async(mode_req)

        arm_req           = CommandBool.Request()
        arm_req.value     = True
        future            = self.cli_arm.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.arm_tmo)

        if future.result() and future.result().success:
            self._is_landed      = False
            response.success     = True
            response.message     = "Armed successfully"
            self.get_logger().info("Drone ARMED.")
        else:
            response.success = False
            response.message = "Arming failed"
        return response

    def _svc_disarm(self, request, response):
        arm_req       = CommandBool.Request()
        arm_req.value = False
        future        = self.cli_arm.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.arm_tmo)

        response.success = bool(future.result() and future.result().success)
        response.message = "Disarmed" if response.success else "Disarm failed"
        if response.success:
            self._is_landed = True
            self.get_logger().info("Drone DISARMED.")
        return response

    def _svc_takeoff(self, request: SetBool.Request, response):
        """
        data=True  → arm + switch GUIDED + ascend to takeoff_height
        data=False → just ascend (already armed)
        """
        if request.data:
            arm_resp = Trigger.Response()
            self._svc_arm(Trigger.Request(), arm_resp)
            if not arm_resp.success:
                response.success = False
                response.message = arm_resp.message
                return response

        self.go_to(
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.takeoff_height,
        )
        self._is_landed  = False
        response.success = True
        response.message = f"Ascending to {self.takeoff_height:.1f} m"
        self.get_logger().info(response.message)
        return response

    def _svc_land(self, request, response):
        """
        Use MAVROS land command for a clean touchdown.
        """
        if not self.cli_land.wait_for_service(timeout_sec=self.svc_wait_tmo):
            response.success = False
            response.message = "LAND service unavailable"
            return response

        land_req               = CommandTOL.Request()
        land_req.altitude      = 0.0
        land_req.latitude      = 0.0
        land_req.longitude     = 0.0
        land_req.min_pitch     = 0.0
        land_req.yaw           = 0.0
        future = self.cli_land.call_async(land_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.land_tmo)

        if future.result() and future.result().success:
            self._is_landed  = True
            response.success = True
            response.message = "Landing initiated"
            self.get_logger().info("LANDING.")
        else:
            response.success = False
            response.message = "Land command failed"
        return response


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = HydroneControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
