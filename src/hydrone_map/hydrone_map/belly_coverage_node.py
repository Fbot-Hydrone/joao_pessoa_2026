#!/usr/bin/env python3
"""
belly_coverage_node — what the DOWN camera has actually looked at, painted on
the floor, plus the path the vehicle actually flew.

Three topics, all for RViz, all pure observation — nothing in the stack reads
them and turning this node off changes no behaviour:

  /hydrone/belly/coverage    nav_msgs/OccupancyGrid    the painted floor
  /hydrone/belly/footprint   visualization_msgs/Marker the CURRENT view quad
  /hydrone/belly/trajectory  nav_msgs/Path             where it has been

WHY THIS AND NOT feature_map's coverage
---------------------------------------
`feature_map_node` already publishes a coverage grid, built from the ZED's
point cloud: it answers "which parts of the arena has the forward camera's
depth reached". That is a different question from the one that matters to a
lawnmower, which is "which parts of the FLOOR has the camera that finds pads
actually passed over". In map_sweep the belly camera is the only detector, so
its footprint IS the search coverage, and a gap in this grid is a strip where a
base could sit unseen.

THE SIZE OF THE PATCH COMES FROM THE RANGEFINDER
------------------------------------------------
A nadir camera's ground footprint is `height * width_px / fx`, and the height
that matters is the height above WHATEVER IS UNDERNEATH — not above the arena
floor. Over the house roof at 1.5 m the vehicle is 1.5 m closer to the surface
than it is over open floor, and the patch it sees is that much smaller.

Deriving the height from the vehicle's altitude and an assumed floor would
paint a patch 88% too wide over the house, which is exactly the mistake that
cost the lane spacing its coverage (docs/SEED-SWEEP-2026-09-02.md 3). The
rangefinder measures the real distance to whatever is below, so the painted
patch shrinks over a raised structure on its own — and the thin band that the
lanes then fail to cover shows up as unpainted floor, which is the whole point
of having this to look at.

`ground_z` is only the fallback, for a frame with no usable range reading.

WHAT THE SHADING MEANS
----------------------
A cell darkens with the number of times it has been seen, so a single pass is
faint and the overlap between two lanes is solid. Reading the lane pitch off
the picture is then possible: if the faint bands do not touch, the sweep has
holes.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import CameraInfo, Range
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

import tf2_ros


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Quaternion -> 3x3 rotation matrix. Hand-rolled: transforms3d is not in
    the stack's container and this is the only piece of it needed here."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class BellyCoverageNode(Node):

    def __init__(self, **kwargs):
        super().__init__("belly_coverage", **kwargs)

        self.declare_parameter("camera_info_topic", "/down_cam/camera_info")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("range_topic",
                               "/mavros/distance_sensor/rangefinder")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("optical_frame", "down_cam_optical_frame")
        # Fallback only. See the module docstring: the rangefinder is the
        # measurement, this is what to do when it is missing or implausible.
        self.declare_parameter("ground_z", -0.7)
        self.declare_parameter("range_min_m", 0.15)
        self.declare_parameter("range_max_m", 12.0)
        self.declare_parameter("max_range_age_s", 0.5)
        # Grid geometry, m. 12 m square at 0.10 m is 120x120 = 14 kB per
        # publish, which is nothing next to an image topic and fine at 2 Hz.
        self.declare_parameter("grid_size_m", 12.0)
        self.declare_parameter("grid_resolution_m", 0.10)
        self.declare_parameter("publish_hz", 2.0)
        # Cosine of the largest tilt at which the footprint is still worth
        # painting. Banked hard, the quad on the ground is a long smear and the
        # camera is not looking where the vehicle is. 0.87 is about 30 deg.
        self.declare_parameter("min_nadir_cos", 0.87)
        # How far the vehicle must move before another point is appended to the
        # trajectory. Without it a hover adds thousands of identical points.
        self.declare_parameter("path_step_m", 0.05)
        self.declare_parameter("path_max_points", 6000)

        p = lambda name: self.get_parameter(name).value
        self.base_frame = str(p("base_frame"))
        self.optical_frame = str(p("optical_frame"))
        self.ground_z = float(p("ground_z"))
        self.range_min = float(p("range_min_m"))
        self.range_max = float(p("range_max_m"))
        self.max_range_age = float(p("max_range_age_s"))
        self.res = float(p("grid_resolution_m"))
        self.size_m = float(p("grid_size_m"))
        self.min_nadir_cos = float(p("min_nadir_cos"))
        self.path_step = float(p("path_step_m"))
        self.path_max = int(p("path_max_points"))

        self.cells = max(4, int(round(self.size_m / self.res)))
        self.origin = -0.5 * self.cells * self.res
        # How many times each cell has been inside the footprint.
        self.seen = np.zeros((self.cells, self.cells), dtype=np.int32)

        self.K = None
        self.pose = None
        self.range_m = None
        self.range_t = 0.0
        self.R_base_opt = None
        self.t_base_opt = None
        self.path = Path()
        self._warned_tf = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        # The grid and the path are LATCHED: RViz is usually started after the
        # flight has begun, and a volatile publisher would leave it blank until
        # the next update.
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(CameraInfo, p("camera_info_topic"),
                                 self._cb_info, sensor_qos)
        self.create_subscription(PoseStamped, p("pose_topic"),
                                 self._cb_pose, sensor_qos)
        self.create_subscription(Range, p("range_topic"),
                                 self._cb_range, sensor_qos)

        self.pub_grid = self.create_publisher(
            OccupancyGrid, "/hydrone/belly/coverage", latched)
        self.pub_foot = self.create_publisher(
            Marker, "/hydrone/belly/footprint", 10)
        self.pub_path = self.create_publisher(
            Path, "/hydrone/belly/trajectory", latched)

        hz = max(0.2, float(p("publish_hz")))
        self.create_timer(1.0 / hz, self._publish)
        self.get_logger().info(
            f"belly_coverage up — painting {self.size_m:.0f} x {self.size_m:.0f} m "
            f"at {self.res:.2f} m from {p('camera_info_topic')}, height from "
            f"{p('range_topic')} (fallback: ground_z {self.ground_z:+.2f} m)")

    # ── inputs ──────────────────────────────────────────────────────────────

    def _cb_info(self, msg):
        self.K = (float(msg.k[0]), float(msg.k[4]),
                  float(msg.k[2]), float(msg.k[5]), msg.width, msg.height)

    def _cb_pose(self, msg):
        self.pose = msg
        self._append_path(msg)

    def _cb_range(self, msg):
        self.range_m = float(msg.range)
        self.range_t = self.get_clock().now().nanoseconds * 1e-9

    def _ensure_mount(self):
        if self.R_base_opt is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.optical_frame, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            if not self._warned_tf:
                self._warned_tf = True
                self.get_logger().warn(
                    f"no TF {self.base_frame} -> {self.optical_frame} yet "
                    f"({exc}); nothing painted until it appears")
            return False
        t, q = tf.transform.translation, tf.transform.rotation
        self.t_base_opt = np.array([t.x, t.y, t.z])
        self.R_base_opt = quat_to_matrix(q.x, q.y, q.z, q.w)
        return True

    # ── the footprint ───────────────────────────────────────────────────────

    def _height_below(self, p_cam):
        """Distance from the camera to whatever is under it, m, or None.

        The rangefinder first, because it MEASURES it. `ground_z` assumes a
        flat floor, which is exactly the assumption a raised structure breaks.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        if (self.range_m is not None
                and now - self.range_t <= self.max_range_age
                and self.range_min < self.range_m < self.range_max):
            return self.range_m
        drop = p_cam[2] - self.ground_z
        return drop if drop > 0.1 else None

    def _footprint(self):
        """The four ground corners of the current view, or None.

        Each image corner is a ray; they are intersected with the horizontal
        plane sitting `h` below the camera, where `h` is what the rangefinder
        measured. That plane is a local approximation of the surface — right
        under the vehicle, and increasingly wrong towards the corners if the
        surface is stepped. For a coverage picture that is the right trade:
        the alternative is a raycast per corner into the octomap, which costs a
        tree decode per frame to sharpen a debug overlay.
        """
        if self.K is None or self.pose is None or not self._ensure_mount():
            return None
        fx, fy, cx, cy, w, h_px = self.K
        if fx <= 0.0 or fy <= 0.0:
            return None

        q = self.pose.pose.orientation
        R_world_base = quat_to_matrix(q.x, q.y, q.z, q.w)
        p_base = np.array([self.pose.pose.position.x,
                           self.pose.pose.position.y,
                           self.pose.pose.position.z])
        R_world_opt = R_world_base @ self.R_base_opt
        p_cam = p_base + R_world_base @ self.t_base_opt

        # Refuse a banked frame rather than paint a smear.
        if R_world_opt[2, 2] > -self.min_nadir_cos:
            return None

        height = self._height_below(p_cam)
        if height is None:
            return None
        plane_z = p_cam[2] - height

        corners = []
        for u, v in ((0.0, 0.0), (w, 0.0), (w, h_px), (0.0, h_px)):
            ray = R_world_opt @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
            if ray[2] > -1e-3:            # this corner never meets the plane
                return None
            t = (plane_z - p_cam[2]) / ray[2]
            corners.append(p_cam + t * ray)
        return corners

    def _paint(self, corners):
        """Mark every cell inside the quad as seen once more.

        Convex quad, so a cell is inside when it is on the same side of all
        four edges. Only the quad's bounding box is tested — a few dozen cells
        a side — which is why this needs no polygon library.
        """
        xy = np.array([[c[0], c[1]] for c in corners])
        i0, j0 = self._cell(xy[:, 0].min(), xy[:, 1].min())
        i1, j1 = self._cell(xy[:, 0].max(), xy[:, 1].max())
        i0, j0 = max(i0, 0), max(j0, 0)
        i1, j1 = min(i1, self.cells - 1), min(j1, self.cells - 1)
        if i1 < i0 or j1 < j0:
            return

        # Cell centres in the box.
        xs = self.origin + (np.arange(i0, i1 + 1) + 0.5) * self.res
        ys = self.origin + (np.arange(j0, j1 + 1) + 0.5) * self.res
        gx, gy = np.meshgrid(xs, ys, indexing="ij")

        inside = np.ones(gx.shape, dtype=bool)
        sign = None
        for k in range(4):
            ax, ay = xy[k]
            bx, by = xy[(k + 1) % 4]
            cross = (bx - ax) * (gy - ay) - (by - ay) * (gx - ax)
            if sign is None:
                # The quad's winding is not fixed — it flips with the camera's
                # yaw — so take it from the first edge and test the rest
                # against that rather than assuming counter-clockwise.
                centre = ((bx - ax) * (xy[:, 1].mean() - ay)
                          - (by - ay) * (xy[:, 0].mean() - ax))
                sign = 1.0 if centre >= 0 else -1.0
            inside &= (cross * sign) >= 0
        self.seen[i0:i1 + 1, j0:j1 + 1][inside] += 1

    def _cell(self, x, y):
        return (int(math.floor((x - self.origin) / self.res)),
                int(math.floor((y - self.origin) / self.res)))

    # ── outputs ─────────────────────────────────────────────────────────────

    def _append_path(self, msg):
        if self.path.poses:
            last = self.path.poses[-1].pose.position
            if math.dist((last.x, last.y, last.z),
                         (msg.pose.position.x, msg.pose.position.y,
                          msg.pose.position.z)) < self.path_step:
                return
        self.path.poses.append(msg)
        del self.path.poses[:-self.path_max]

    def _publish(self):
        corners = self._footprint()
        if corners is not None:
            self._paint(corners)
            self.pub_foot.publish(self._footprint_marker(corners))

        if self.pose is None:
            return
        frame = self.pose.header.frame_id or "map"
        now = self.get_clock().now().to_msg()

        grid = OccupancyGrid()
        grid.header.frame_id = frame
        grid.header.stamp = now
        grid.info.resolution = self.res
        grid.info.width = self.cells
        grid.info.height = self.cells
        grid.info.origin.position.x = self.origin
        grid.info.origin.position.y = self.origin
        grid.info.origin.position.z = self.ground_z
        grid.info.origin.orientation.w = 1.0
        # -1 where nothing was ever seen, so unswept floor renders as unknown
        # rather than as "seen zero times". A cell darkens with each pass, and
        # five passes saturate — enough to read overlap off the picture.
        value = np.where(self.seen > 0,
                         np.clip(self.seen * 20, 1, 100), -1).astype(np.int8)
        # OccupancyGrid is row-major in y, so the (x, y) array transposes.
        grid.data = value.T.reshape(-1).tolist()
        self.pub_grid.publish(grid)

        self.path.header.frame_id = frame
        self.path.header.stamp = now
        self.pub_path.publish(self.path)

    def _footprint_marker(self, corners):
        m = Marker()
        m.header.frame_id = self.pose.header.frame_id or "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "belly_footprint"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.03
        m.color = ColorRGBA(r=0.1, g=0.9, b=1.0, a=0.9)
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2]))
                    for c in corners + [corners[0]]]
        return m


def main(args=None):
    rclpy.init(args=args)
    node = BellyCoverageNode()
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
