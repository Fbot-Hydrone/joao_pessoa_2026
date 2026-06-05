#!/usr/bin/env python3
"""
hydrone_biguasim_bridge — Adapter between BiguaSim and the Hydrone stack.

BiguaSim publishes/subscribes with its own topic naming and message formats.
This bridge translates everything so the rest of our stack (controller, nav,
mission) sees exactly the same interface it would see with real hardware.

┌─────────────────────────────────────────────────────────────────────────────┐
│  BIGUASIM side                   │  HYDRONE STACK side                      │
│                                   │                                          │
│  drone_id0/odom/Odom  (Odometry) ──▶ /mavros/local_position/pose (Pose)    │
│  drone_id0/odom/IMU   (Imu)      ──▶ /mavros/imu/data            (Imu)     │
│  drone_id0/camera     (Image)    ──▶ /zed2/zed_node/rgb/...      (Image)   │
│  drone_id0/depth      (Image)    ──▶ /zed2/zed_node/depth/...    (Image)   │
│                                   │                                          │
│  drone_id0/command_control        ◀── /mavros/setpoint_position/local       │
│  (Float64MultiArray [x,y,z,yaw])      (PoseStamped)                         │
└─────────────────────────────────────────────────────────────────────────────┘

Also publishes a fake /mavros/state so the controller thinks it is ARMED and
in GUIDED mode from the start — no real MAVLink needed in simulation.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg        import Float64MultiArray, Header
from geometry_msgs.msg   import PoseStamped, TransformStamped
from nav_msgs.msg        import Odometry
from sensor_msgs.msg     import Imu, Image
from tf2_ros             import TransformBroadcaster

# MAVROS message (only the State is needed to fake arming/mode)
try:
    from mavros_msgs.msg import State as MavState
    MAVROS_AVAILABLE = True
except ImportError:
    MAVROS_AVAILABLE = False

# ── Constants ────────────────────────────────────────────────────────────────

# BiguaSim agent name (must match config yaml, dashes→underscores)
AGENT = "drone_id0"

# How frequently to re-publish the fake MAVROS state (Hz)
FAKE_STATE_HZ = 2


class BiguaSimBridge(Node):

    def __init__(self):
        super().__init__("hydrone_biguasim_bridge")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("agent_name",   AGENT)
        self.declare_parameter("publish_tf",   True)

        agent      = self.get_parameter("agent_name").value
        self.pub_tf = self.get_parameter("publish_tf").value

        # ── QoS ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── TF broadcaster ──────────────────────────────────────────────────
        if self.pub_tf:
            self.tf_br = TransformBroadcaster(self)

        # ────────────────────────────────────────────────────────────────────
        # SUBSCRIBERS — from BiguaSim
        # ────────────────────────────────────────────────────────────────────

        # Odometry (position + velocity + orientation quaternion)
        self.sub_odom = self.create_subscription(
            Odometry,
            f"{agent}/odom/Odom",
            self._cb_odom,
            sensor_qos,
        )

        # IMU
        self.sub_imu = self.create_subscription(
            Imu,
            f"{agent}/odom/IMU",
            self._cb_imu,
            sensor_qos,
        )

        # RGB camera
        self.sub_rgb = self.create_subscription(
            Image,
            f"{agent}/camera",
            self._cb_rgb,
            sensor_qos,
        )

        # Depth camera
        self.sub_depth = self.create_subscription(
            Image,
            f"{agent}/depth",
            self._cb_depth,
            sensor_qos,
        )

        # ────────────────────────────────────────────────────────────────────
        # PUBLISHERS — toward Hydrone stack (mimicking real hardware topics)
        # ────────────────────────────────────────────────────────────────────

        self.pub_pose = self.create_publisher(
            PoseStamped,
            "/mavros/local_position/pose",
            sensor_qos,
        )

        self.pub_imu = self.create_publisher(
            Imu,
            "/mavros/imu/data",
            sensor_qos,
        )

        self.pub_rgb = self.create_publisher(
            Image,
            "/zed2/zed_node/rgb/image_rect_color",
            sensor_qos,
        )

        self.pub_depth = self.create_publisher(
            Image,
            "/zed2/zed_node/depth/depth_registered",
            sensor_qos,
        )

        # Also re-publish Odometry for anything that wants raw odom
        self.pub_odom = self.create_publisher(
            Odometry,
            "/hydrone/odom",
            sensor_qos,
        )

        # ── Fake MAVROS state (GUIDED + ARMED) ───────────────────────────────
        if MAVROS_AVAILABLE:
            self.pub_mav_state = self.create_publisher(
                MavState, "/mavros/state", reliable_qos)
            self.create_timer(1.0 / FAKE_STATE_HZ, self._publish_fake_state)
        else:
            self.get_logger().warn(
                "mavros_msgs not found — /mavros/state will not be published. "
                "Install with: sudo apt install ros-humble-mavros-msgs")

        # ────────────────────────────────────────────────────────────────────
        # SUBSCRIBER — setpoint from Hydrone stack → BiguaSim command
        # ────────────────────────────────────────────────────────────────────

        self.sub_setpoint = self.create_subscription(
            PoseStamped,
            "/mavros/setpoint_position/local",
            self._cb_setpoint,
            reliable_qos,
        )

        # Command publisher toward BiguaSim
        self.pub_cmd = self.create_publisher(
            Float64MultiArray,
            f"{agent}/command_control",
            reliable_qos,
        )

        # Cache the last known yaw so we can hold it when no new setpoint arrives
        self._last_yaw = 0.0

        self.get_logger().info(
            f"BiguaSimBridge ready — bridging agent '{agent}'")

    # ────────────────────────────────────────────────────────────────────────
    # BiguaSim → Stack
    # ────────────────────────────────────────────────────────────────────────

    def _cb_odom(self, msg: Odometry):
        """
        Convert BiguaSim Odometry → PoseStamped for /mavros/local_position/pose.

        BiguaSim uses NED (North-East-Down) internally but publishes in the
        standard ROS ENU frame via the DynamicsSensor encoder, so we just
        re-stamp and forward.
        """
        ps                    = PoseStamped()
        ps.header             = msg.header
        ps.header.frame_id    = "map"
        ps.pose               = msg.pose.pose
        self.pub_pose.publish(ps)
        self.pub_odom.publish(msg)

        # Broadcast map→base_link TF
        if self.pub_tf:
            t                         = TransformStamped()
            t.header                  = msg.header
            t.header.frame_id         = "map"
            t.child_frame_id          = "base_link"
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation      = msg.pose.pose.orientation
            self.tf_br.sendTransform(t)

        # Keep yaw cache updated for command forwarding
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._last_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _cb_imu(self, msg: Imu):
        """Forward IMU directly — already in ROS format."""
        msg.header.frame_id = "base_link"
        self.pub_imu.publish(msg)

    def _cb_rgb(self, msg: Image):
        """Forward RGB camera image — re-stamp with ROS time."""
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        self.pub_rgb.publish(msg)

    def _cb_depth(self, msg: Image):
        """
        Forward depth image.
        BiguaSim DepthCamera publishes float32 meters — same encoding expected
        by hydrone_vision (_get_depth_at uses 32FC1).
        Re-encode if needed.
        """
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        # Fix encoding for cv_bridge (BiguaSim DepthMapEncoder uses bgr8 by mistake)
        msg.encoding        = "32FC1"
        msg.step            = msg.width * 4
        self.pub_depth.publish(msg)

    # ────────────────────────────────────────────────────────────────────────
    # Stack → BiguaSim
    # ────────────────────────────────────────────────────────────────────────

    def _cb_setpoint(self, msg: PoseStamped):
        """
        Convert PoseStamped setpoint → Float64MultiArray [x, y, z, yaw]
        for BiguaSim cmd_pos_yaw control abstraction.
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # Extract yaw from quaternion
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        cmd          = Float64MultiArray()
        cmd.data     = [x, y, z, yaw]
        self.pub_cmd.publish(cmd)

    # ────────────────────────────────────────────────────────────────────────
    # Fake MAVROS state
    # ────────────────────────────────────────────────────────────────────────

    def _publish_fake_state(self):
        """
        Publish a permanent GUIDED+ARMED state so the controller node
        never blocks waiting for arming handshake.
        """
        if not MAVROS_AVAILABLE:
            return
        state             = MavState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.connected   = True
        state.armed       = True
        state.guided      = True
        state.mode        = "GUIDED"
        self.pub_mav_state.publish(state)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = BiguaSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
