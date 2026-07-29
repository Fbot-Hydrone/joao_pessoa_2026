#!/usr/bin/env python3
"""
vision_odom_bridge — feed ZED visual odometry into ArduPilot as external nav.

CBR rules ban GPS, so the drone must localise from the ZED's visual odometry.
This node takes /zed/zed_node/odom (nav_msgs/Odometry, ENU odom->base_link) and
republishes the pose on /mavros/vision_pose/pose (geometry_msgs/PoseStamped),
which MAVROS turns into VISION_POSITION_ESTIMATE for the FCU. With EKF3 sources
set to ExternalNav (see holybro_sitl.parm), ArduPilot then flies on this instead
of GPS.

MAVROS's vision_pose plugin does the ENU->NED conversion, so we forward the pose
as-is. On the real drone the exact same topic is produced by the real ZED VO —
no code change.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped


class VisionOdomBridge(Node):
    def __init__(self):
        super().__init__("vision_odom_bridge")
        self.declare_parameter("in_odom", "/zed/zed_node/odom")
        self.declare_parameter("out_pose", "/mavros/vision_pose/pose")
        # GPS-denied flight needs a global origin, or ArduPilot never sets home
        # and GUIDED takeoff's altitude-frame conversion silently fails
        # ("NAV_TAKEOFF: FAILED"). Send an arbitrary but fixed origin (matches the
        # ardubridge gps_origin) so home/altitude references resolve.
        self.declare_parameter("origin_lat", 33.810313)
        self.declare_parameter("origin_lon", -118.393867)
        self.declare_parameter("origin_alt", 0.0)
        in_odom = self.get_parameter("in_odom").value
        out_pose = self.get_parameter("out_pose").value

        # MAVROS's vision_pose plugin subscribes RELIABLE — a BEST_EFFORT
        # publisher is dropped ("incompatible QoS, no messages sent"). Publish
        # RELIABLE to it. Subscribe to the ZED odom BEST_EFFORT so we still match
        # the sensor-style publisher (a best-effort sub accepts a reliable pub).
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub = self.create_publisher(PoseStamped, out_pose, pub_qos)
        self.create_subscription(Odometry, in_odom, self._cb, sub_qos)

        # Set the global origin at 1 Hz for the first 30 s (MAVROS/FCU must be up
        # first, and home must be established before any takeoff). Sending it
        # repeatedly is idempotent and robust against startup timing.
        self.origin_pub = self.create_publisher(
            GeoPointStamped, "/mavros/global_position/set_gp_origin", pub_qos)
        self._origin_sends = 0
        self.create_timer(1.0, self._send_origin)

        self.get_logger().info(
            f"vision_odom_bridge: {in_odom} (Odometry) -> {out_pose} (PoseStamped)")

    def _send_origin(self):
        if self._origin_sends >= 30:
            return
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position.latitude = self.get_parameter("origin_lat").value
        msg.position.longitude = self.get_parameter("origin_lon").value
        msg.position.altitude = self.get_parameter("origin_alt").value
        self.origin_pub.publish(msg)
        self._origin_sends += 1
        if self._origin_sends == 1:
            self.get_logger().info("sent GPS global origin (GPS-denied home reference)")

    def _cb(self, msg: Odometry):
        # BiguaSim's odom is in NWU (world); MAVROS vision_pose expects ENU and
        # converts ENU->NED for the FCU. Without this the position is rotated 90
        # deg, so position-hold corrections go sideways -> the drone circles and
        # spins. Rotate +90 deg about Z: position (E,N,U) = (-W, N, U) = (-y,x,z);
        # orientation q_enu = Rz(90) (x) q_nwu (world reframe; body is already FLU).
        # This makes MAVROS' NED output match the working JSON/GPS path's (n,-w,-u).
        o = msg.pose.pose
        p = PoseStamped()
        p.header = msg.header
        p.pose.position.x = -o.position.y
        p.pose.position.y = o.position.x
        p.pose.position.z = o.position.z
        w, x, y, z = o.orientation.w, o.orientation.x, o.orientation.y, o.orientation.z
        s = 0.7071067811865476  # cos/sin(45 deg)
        p.pose.orientation.w = s * (w - z)
        p.pose.orientation.x = s * (x - y)
        p.pose.orientation.y = s * (x + y)
        p.pose.orientation.z = s * (w + z)
        self.pub.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = VisionOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
