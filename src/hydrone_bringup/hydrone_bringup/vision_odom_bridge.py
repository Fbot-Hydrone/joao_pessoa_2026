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


class VisionOdomBridge(Node):
    def __init__(self):
        super().__init__("vision_odom_bridge")
        self.declare_parameter("in_odom", "/zed/zed_node/odom")
        self.declare_parameter("out_pose", "/mavros/vision_pose/pose")
        in_odom = self.get_parameter("in_odom").value
        out_pose = self.get_parameter("out_pose").value

        # Best-effort matches the ZED/odometry publishers; MAVROS vision_pose is
        # tolerant. Keep depth small — only the latest pose matters.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub = self.create_publisher(PoseStamped, out_pose, qos)
        self.create_subscription(Odometry, in_odom, self._cb, qos)
        self.get_logger().info(
            f"vision_odom_bridge: {in_odom} (Odometry) -> {out_pose} (PoseStamped)")

    def _cb(self, msg: Odometry):
        p = PoseStamped()
        p.header = msg.header          # same stamp + odom frame
        p.pose = msg.pose.pose
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
