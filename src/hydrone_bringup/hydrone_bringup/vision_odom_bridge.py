#!/usr/bin/env python3
"""
vision_odom_bridge — feed ZED visual odometry into ArduPilot as external nav.

CBR rules ban GPS, so the drone must localise from the ZED's visual odometry.
This node takes /zed/zed_node/odom (nav_msgs/Odometry, NWU odom->base_link as
produced by the sim VO) and republishes the pose on /mavros/vision_pose/pose
(geometry_msgs/PoseStamped), which MAVROS turns into VISION_POSITION_ESTIMATE for
the FCU. With EKF3 sources set to ExternalNav (see holybro_sitl.parm), ArduPilot
then flies on this instead of GPS.

Frame: the input is NWU, so we rotate +90deg about Z to ENU (see _cb); MAVROS's
vision_pose plugin then does ENU->NED for the FCU. NOTE sim->real contract risk:
the real zed_wrapper publishes odom already in ENU (REP-103), so that +90deg must
be dropped when running on hardware — see docs/SENSOR-CONFIG.md contract checklist.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import HomePosition


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
        # Keep resending the origin until home is CONFIRMED, not for a fixed
        # wall-clock window — see _send_origin for why the old 30 s budget was a
        # race the simulator loses on a slow machine. This cap is only a backstop
        # against publishing forever if home never arrives.
        self.declare_parameter("origin_max_sends", 600)
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

        # Set the global origin at 1 Hz until the FCU confirms home. Sending it
        # repeatedly is idempotent; the confirmation is what stops us.
        self.origin_pub = self.create_publisher(
            GeoPointStamped, "/mavros/global_position/set_gp_origin", pub_qos)
        self._origin_sends = 0
        self._home_set = False
        self._max_sends = self.get_parameter("origin_max_sends").value
        self.create_subscription(
            HomePosition, "/mavros/home_position/home", self._home_cb, sub_qos)
        self.create_timer(1.0, self._send_origin)

        self.get_logger().info(
            f"vision_odom_bridge: {in_odom} (Odometry) -> {out_pose} (PoseStamped)")

    def _home_cb(self, msg: HomePosition):
        """FCU published a home position — the origin took, so stop resending."""
        if not self._home_set:
            self._home_set = True
            self.get_logger().info(
                f"home confirmed after {self._origin_sends} origin send(s) "
                f"(lat {msg.geo.latitude:.6f}, lon {msg.geo.longitude:.6f})")

    def _send_origin(self):
        # ArduPilot SILENTLY DROPS SET_GPS_GLOBAL_ORIGIN until EKF3 is the active
        # AHRS backend: AP_AHRS::set_origin() switches on active_EKF_type(), and
        # the EKFType::DCM case just breaks, leaving success=false. On a slow
        # machine EKF3 init runs on the synthetic clock and takes far longer in
        # wall time — measured here, the origin was sent t+0..t+29 s while AHRS
        # was still DCM, and EKF3 only went active at t+59 s. The origin was
        # therefore never accepted, home was never set, and every takeoff failed
        # (MAVROS logs "CMD: Unexpected command 410, result 4" — ArduPilot's
        # GET_HOME_POSITION returns MAV_RESULT_FAILED when !home_is_set()).
        # That race is why it works on a fast machine and not on a slow one.
        # So: resend until home is CONFIRMED, never on a fixed time budget.
        if self._home_set or self._origin_sends >= self._max_sends:
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
        elif self._origin_sends == self._max_sends:
            self.get_logger().error(
                f"home STILL not set after {self._max_sends} origin sends — "
                "giving up. Takeoff will fail; check that EKF3 went active "
                "(FCU: 'AHRS: EKF3 active') and is accepting external nav.")

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
