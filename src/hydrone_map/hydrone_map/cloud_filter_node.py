#!/usr/bin/env python3
"""cloud_filter_node — the ZED's cloud, per frame, with the liars removed.

    /zed/zed_node/point_cloud/cloud_registered  ──> filter ──> /hydrone/map/cloud_filtered
                                                                      │
                                                                octomap_server

feature_map_node ACCUMULATES: it folds every frame into one persistent voxel
hash and publishes that. An occupancy map needs the opposite — one frame at a
time, still stamped, still in the sensor's frame — because ray casting has to
know where the camera was when it saw those points. So this node does the
filtering and nothing else: same cloud in, same cloud out, minus the points
that would lie to a ray.

Why the cloud must NOT be transformed here
------------------------------------------
octomap_server casts a ray from the sensor origin to every point, and it finds
that origin by looking up the cloud's frame_id in TF. Hand it a cloud already
folded into `odom` and every ray is traced from the world origin instead of
from the camera — the map fills with free space swept from a point the drone
has never been. The header is therefore passed through untouched, and the
existing map->odom->base_link->camera chain does the work. That chain is
map_odom_node's job and already runs in the Phase 1 stack.

Why not just point octomap_server at the raw cloud
--------------------------------------------------
Flying pixels. A pixel straddling a silhouette returns a foreground/background
blend: MEASURED on the arena, a wall at 4.86 m produced edge samples out to
18.07 m. In a voxel grid that is one wrong cell. In an occupancy grid the ray
to that phantom carves everything it crosses as FREE — including the real wall
at 4.86 m — and the planner reads a doorway where there is masonry. The
rejection lives in hydrone_map.cloud_filter and is shared with feature_map_node
so both maps reject the same points.

`reject_hole_borders` is ON here and off there: the sky/wall border is exactly
where a ray does the most damage, and it is worth losing the outline to keep
the wall solid. See cloud_filter.edge_mask.
"""

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField

from hydrone_map import cloud_filter


class CloudFilterNode(Node):

    def __init__(self, **kwargs):
        super().__init__("cloud_filter", **kwargs)

        self.declare_parameter("in_cloud",
                               "/zed/zed_node/point_cloud/cloud_registered")
        self.declare_parameter("out_cloud", "/hydrone/map/cloud_filtered")

        # Thinning. 4 (one pixel in 16) is what feature_map_node has run with;
        # for occupancy it is also the CPU dial. The cameras are already capped
        # at 10 Hz by the render budget (see biguasim config.yaml), so a frame
        # here is 640x480/16 ~ 19k points before the depth window.
        self.declare_parameter("stride", 4)
        self.declare_parameter("min_depth", 0.4)
        # Tighter than feature_map's 20 m on purpose: the arena is 8x8 m, so a
        # return past its diagonal (11.3 m) is either the sky or a reflection,
        # and a long ray is also the most expensive thing octomap can be given.
        self.declare_parameter("max_depth", 12.0)
        self.declare_parameter("max_edge_step", 0.30)
        # ON here, off in feature_map_node — see the module docstring.
        self.declare_parameter("reject_hole_borders", True)
        # Throttle. The camera runs at 10 Hz and octomap does not need every
        # frame: the drone moves centimetres between them, so consecutive
        # clouds carry almost the same rays. It IS the CPU dial (ray casting
        # is per point) and the bandwidth dial — octomap republishes its whole
        # visualisation on every insert, and MEASURED on a 6x6 m floor plus one
        # wall that is 111 KB of occupied cells and 172 KB of free cells per
        # update. At 10 Hz that is 2.8 MB/s of markers for a scene far smaller
        # than the arena; at 2 Hz it is 0.6. feature_map_node throttles the
        # same way and for the same reason (its process_hz is 4.0).
        self.declare_parameter("process_hz", 2.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._in = p("in_cloud")
        self.stride = max(1, int(p("stride")))
        self.min_depth = float(p("min_depth"))
        self.max_depth = float(p("max_depth"))
        self.max_edge_step = float(p("max_edge_step"))
        self.reject_hole_borders = bool(p("reject_hole_borders"))
        hz = float(p("process_hz"))
        self._min_period_ns = int(1e9 / hz) if hz > 0 else 0
        self._last_ns = 0

        # Best effort, depth 1: a cloud is worth publishing only while it is
        # the current one. Matches how the camera publishes it.
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(PointCloud2, p("out_cloud"), qos)
        self.create_subscription(PointCloud2, self._in, self._cb, qos)

        self._unorganized_warned = False
        self.get_logger().info(
            f"filtering {self._in} -> {p('out_cloud')} "
            f"(stride {self.stride}, {self.min_depth}-{self.max_depth} m, "
            f"{hz:g} Hz, "
            f"hole borders {'rejected' if self.reject_hole_borders else 'kept'})")

    def _cb(self, msg: PointCloud2):
        # Drop before doing any work: the filtering and the ray casting it
        # feeds are the expensive parts, not the subscription.
        now = self.get_clock().now().nanoseconds
        if self._min_period_ns and now - self._last_ns < self._min_period_ns:
            return
        self._last_ns = now

        problem = cloud_filter.layout_problem(msg)
        if problem is not None:
            self.get_logger().error(f"{self._in} {problem}; not filtering it",
                                    throttle_duration_sec=30.0)
            return

        points, unorganized = cloud_filter.sample(
            cloud_filter.xyz_view(msg),
            min_depth=self.min_depth,
            max_depth=self.max_depth,
            max_edge_step=self.max_edge_step,
            stride=self.stride,
            reject_hole_borders=self.reject_hole_borders,
        )
        if unorganized and not self._unorganized_warned:
            self.get_logger().warn(
                f"{self._in} is unorganized (height 1); flying-pixel "
                "rejection needs pixel neighbours and is off. The occupancy "
                "map will carve free space through walls.")
            self._unorganized_warned = True
        if points is None:
            return

        self.pub.publish(self._cloud(msg.header, points))

    def _cloud(self, header, points: np.ndarray) -> PointCloud2:
        """Mx3 -> an unorganized float32 cloud, header passed through.

        The header is the whole point: same stamp, same frame_id, so
        octomap_server resolves the sensor origin for these exact points.
        """
        xyz = np.asarray(points, dtype=np.float32)
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(xyz)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = xyz.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = CloudFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
