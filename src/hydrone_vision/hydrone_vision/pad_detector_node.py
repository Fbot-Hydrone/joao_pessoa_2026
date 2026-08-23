#!/usr/bin/env python3
"""
pad_detector_node — runs :mod:`hydrone_vision.pad_detector` on one camera and
turns each hit into a world-frame landing-pad observation.

One node instance per camera. The mission runs two:

  forward  /zed/zed_node/rgb/image_rect_color  + depth   sees pads AHEAD, far off
  down     /down_cam/image_raw                 no depth  sees the pad BELOW, and
                                                          only says THAT it does

Both publish the same message type, tagged with `camera`. Whether they share a
topic is the launch file's call: phase1 gives the down camera its own, so that
only positions the map should trust ever reach pad_map_node.

Pixel -> world
--------------
Three routes, in order of preference:

  DEPTH         if a registered depth image is available and finite at the pad
                centroid, back-project the pixel with the intrinsics. Metric and
                assumption-free. This is what the forward ZED normally uses.

  GROUND PLANE  otherwise, cast the pixel's ray and intersect it with a
                horizontal plane at `ground_z`. Accurate only where the floor
                really is at `ground_z`.

  NONE          `project_position: False` — publish the pixel, the radius and
                the confidence, and set `position_valid` false. The detection
                still says a pad is in view; it makes no claim about where.

That last route is not a degraded mode, it is the right one for a camera whose
job is confirmation. The ground-plane route assumes a FLAT FLOOR AT `ground_z`,
and the competition's bases are RAISED: a ray cast from overhead crosses the
assumed plane beyond the pad it actually hit. Worse, pad_map_node weights every
projection by `confidence / max(range, 1)`, so one belly-camera look from 2 m
outweighs a ZED look from 10 m four to one, and a confirmation hover delivers
hundreds of them — a wrong position arriving with the heaviest vote in the map,
overwriting the ZED estimate the drone flew there on. The ZED is the position
estimate. The belly camera is a yes/no.

Which world frame
-----------------
The frame of `/mavros/local_position/pose`, i.e. the FCU's local ENU. NOT the VO
`odom` frame, even though the two are near-identical here.

That is a deliberate choice: the mission's position setpoints go to
`/mavros/setpoint_position/local`, which is interpreted in exactly this frame. A
pad mapped in `odom` would have to survive an odom->EKF-local correction before
it could be flown to, and any error there would show up as a landing that misses
the pad. Composing the vehicle pose from the same source the controller uses
removes that class of error entirely.

So the only thing TF is consulted for is the CONSTANT `base_link -> <optical
frame>` mount transform, looked up once and cached.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image

import tf2_ros

from hydrone_msgs.msg import PadDetection

from hydrone_vision.image_convert import (
    bgr_image_to_numpy, depth_image_to_numpy, numpy_to_image)
from hydrone_vision.pad_detector import PadDetector, draw_detections


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Quaternion -> 3x3 rotation matrix.

    Hand-rolled on purpose: tf_transformations / transforms3d are not in the
    stack's container, and this is the only piece of them the node needs.
    """
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class PadDetectorNode(Node):

    def __init__(self, **kwargs):
        # **kwargs is forwarded to rclpy's Node so tests can pass
        # parameter_overrides; the parameters below are read once and cached, so
        # they have to be in place before construction.
        super().__init__("pad_detector", **kwargs)

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("camera", "forward")
        self.declare_parameter("image_topic",
                               "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("camera_info_topic",
                               "/zed/zed_node/rgb/camera_info")
        # Empty string = this camera has no depth; projection falls back to the
        # ground plane. That is the normal case for the down camera.
        self.declare_parameter("depth_topic",
                               "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("optical_frame",
                               "zed_left_camera_optical_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("out_topic", "/hydrone/pads/detections")
        # Height of the arena floor in the vehicle's local frame. 0.0 = the
        # plane the drone took off from, which is what the FCU zeroes its local
        # altitude to. Elevated pads sit above it; pad_map_node measures their
        # real height with the rangefinder rather than guessing it here.
        self.declare_parameter("ground_z", 0.0)
        # False = this camera reports WHAT it sees, never WHERE: no depth
        # back-projection, no ground-plane cast, position_valid always false.
        # It then needs neither camera_info nor the vehicle pose, so it goes on
        # answering while the position estimate is bad — which is precisely when
        # a confirmation is worth most. See the module docstring.
        self.declare_parameter("project_position", True)
        self.declare_parameter("publish_debug", True)
        # 0 = process every frame. Set it to cap CPU when the camera outruns the
        # detector (the sim's 20 Hz is comfortably below that).
        self.declare_parameter("max_rate_hz", 0.0)
        # Reject detections whose pose would come from a stale vehicle pose.
        self.declare_parameter("max_pose_age_s", 1.0)

        # Detector thresholds, exposed so the arena's real lighting can be
        # dialled in from a launch file / YAML without touching the algorithm.
        self.declare_parameter("blue_hsv_low", [95, 110, 50])
        self.declare_parameter("blue_hsv_high", [135, 255, 255])
        # WHICH PAD is in front of the camera, which decides how it is found
        # at all -- not a threshold. "blue" is BiguaSim's pad: a saturated blue
        # field on brown ground, found by its colour. "dark_blue" is the real
        # one, which lies on foam of its OWN hue and whose paint the ZED
        # renders green and washed out; it is found by local contrast instead,
        # and the HSV bands below do not apply to it at all.
        # docs/LANDING-SITES.md 3.
        self.declare_parameter("field_mode", "blue")
        self.declare_parameter("yellow_hsv_low", [18, 110, 90])
        self.declare_parameter("yellow_hsv_high", [38, 255, 255])
        # dark_blue knobs. These are contrasts, sizes and counts, not colours.
        # mark_delta is the one to reach for first if the arena's light changes
        # and the pad goes quiet: lower it until the markings come back in
        # /hydrone/pads/<camera>/debug_image, no further.
        self.declare_parameter("mark_delta", 8.0)
        self.declare_parameter("mark_window_frac", 0.06)
        # How far real paint has to clear mark_delta, as a multiple of it.
        # Raise if stains and mat seams start being reported; lower if a real
        # pad is being thrown out at range.
        self.declare_parameter("mark_contrast_mult", 1.5)
        self.declare_parameter("min_axis_px", 18.0)
        # THE ONE THAT DIFFERS BETWEEN THE TWO CAMERAS. How much of the polar
        # sweep has to stay inside the image. The belly camera at landing
        # height sees a pad whose centre is off-frame and needs this low; the
        # forward camera, whose answer becomes a world position, should never
        # be reading a fragment at the frame edge and wants it high.
        self.declare_parameter("min_seen", 0.30)
        # Parts of the frame the drone itself occupies, as x0,y0,x1,y1
        # fractions, repeated. The belly camera sees its own landing legs, and
        # a dark object with a bright edge on blue foam passes every test the
        # detector has -- one scored 0.95. Nothing in a single frame separates
        # them, so they are excluded by position. MEASURE THESE ON THE ACTUAL
        # AIRFRAME; the defaults are empty.
        self.declare_parameter("ignore_regions", [0.0])
        self.declare_parameter("min_area_px", 150.0)
        self.declare_parameter("min_confidence", 0.50)

        p = lambda name: self.get_parameter(name).value
        self.camera = p("camera")
        self.optical_frame = p("optical_frame")
        self.base_frame = p("base_frame")
        self.ground_z = float(p("ground_z"))
        self.project_position = bool(p("project_position"))
        self.publish_debug = bool(p("publish_debug"))
        self.max_pose_age = float(p("max_pose_age_s"))
        rate = float(p("max_rate_hz"))
        self.min_period_ns = int(1e9 / rate) if rate > 0.0 else 0

        # A ROS double array cannot be declared empty, so the default is a
        # single 0.0 and anything shorter than one rectangle means "none".
        ignore = [float(v) for v in p("ignore_regions")]
        if len(ignore) < 4:
            ignore = []

        self.detector = PadDetector(
            blue_hsv_low=tuple(int(v) for v in p("blue_hsv_low")),
            blue_hsv_high=tuple(int(v) for v in p("blue_hsv_high")),
            field_mode=str(p("field_mode")),
            mark_delta=float(p("mark_delta")),
            mark_window_frac=float(p("mark_window_frac")),
            mark_contrast_mult=float(p("mark_contrast_mult")),
            min_axis_px=float(p("min_axis_px")),
            min_seen=float(p("min_seen")),
            ignore_regions=ignore,
            yellow_hsv_low=tuple(int(v) for v in p("yellow_hsv_low")),
            yellow_hsv_high=tuple(int(v) for v in p("yellow_hsv_high")),
            min_area_px=float(p("min_area_px")),
            min_confidence=float(p("min_confidence")),
        )

        # ── State ───────────────────────────────────────────────────────────
        self.K: np.ndarray | None = None
        self.depth: np.ndarray | None = None
        self.pose: PoseStamped | None = None
        # base_link -> optical, resolved once from TF (it is a static mount).
        self.R_base_opt: np.ndarray | None = None
        self.t_base_opt: np.ndarray | None = None
        self._last_stamp_ns = 0
        self._warned_tf = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── I/O ─────────────────────────────────────────────────────────────
        self.pub_det = self.create_publisher(PadDetection, p("out_topic"), 10)
        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image, f"/hydrone/pads/{self.camera}/debug_image", sensor_qos)

        # Intrinsics, vehicle pose and depth exist only to place a pixel in the
        # world. A camera that does not project subscribes to none of them.
        depth_topic = p("depth_topic") if self.project_position else ""
        if self.project_position:
            self.create_subscription(CameraInfo, p("camera_info_topic"),
                                     self._cb_info, sensor_qos)
            # MAVROS publishes local_position/pose BEST_EFFORT; a RELIABLE
            # subscriber would never receive it.
            self.create_subscription(PoseStamped, p("pose_topic"),
                                     self._cb_pose, sensor_qos)
            if depth_topic:
                self.create_subscription(Image, depth_topic,
                                         self._cb_depth, sensor_qos)
        self.create_subscription(Image, p("image_topic"),
                                 self._cb_image, sensor_qos)

        how = ("depth=" + depth_topic if depth_topic
               else "ground-plane projection" if self.project_position
               else "DETECTION ONLY — publishes no position")
        self.get_logger().info(
            f"pad_detector[{self.camera}] up — image={p('image_topic')} "
            f"{how} -> {p('out_topic')}")

    # ────────────────────────────────────────────────────────────────────────
    # Inputs
    # ────────────────────────────────────────────────────────────────────────

    def _cb_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _cb_depth(self, msg: Image):
        self.depth = depth_image_to_numpy(msg)

    def _cb_pose(self, msg: PoseStamped):
        self.pose = msg

    # ────────────────────────────────────────────────────────────────────────
    # Main loop
    # ────────────────────────────────────────────────────────────────────────

    def _cb_image(self, msg: Image):
        if self.min_period_ns:
            now = self.get_clock().now().nanoseconds
            if now - self._last_stamp_ns < self.min_period_ns:
                return
            self._last_stamp_ns = now

        frame = bgr_image_to_numpy(msg)
        dets = self.detector.detect(frame)

        for det in dets:
            self.pub_det.publish(self._to_msg(msg, det))

        if self.publish_debug:
            label = f"{self.camera}  {len(dets)} pad(s)"
            out = numpy_to_image(draw_detections(frame, dets, label))
            out.header = msg.header
            self.pub_debug.publish(out)

    def _to_msg(self, img_msg: Image, det) -> PadDetection:
        msg = PadDetection()
        msg.header.stamp = img_msg.header.stamp
        msg.camera = self.camera
        msg.u = float(det.u)
        msg.v = float(det.v)
        msg.radius_px = float(det.radius_px)
        msg.confidence = float(det.confidence)

        if not self.project_position:
            # Deliberate, not a failure. u/v/radius/confidence above are the
            # whole message: this camera says a pad is in view and stops there.
            msg.position_valid = False
            msg.source = PadDetection.SOURCE_NONE
            return msg

        world = self._project(det.u, det.v)
        if world is None:
            msg.position_valid = False
            msg.source = PadDetection.SOURCE_NONE
            return msg

        point, source, range_m, frame_id = world
        msg.header.frame_id = frame_id
        msg.position.x = float(point[0])
        msg.position.y = float(point[1])
        msg.position.z = float(point[2])
        msg.position_valid = True
        msg.range_m = float(range_m)
        msg.source = source
        return msg

    # ────────────────────────────────────────────────────────────────────────
    # Geometry
    # ────────────────────────────────────────────────────────────────────────

    def _project(self, u: float, v: float):
        """Pixel -> (world point, source, range, frame_id), or None."""
        if self.K is None or self.pose is None:
            return None
        if not self._ensure_mount_tf():
            return None

        age = abs(self.get_clock().now().nanoseconds
                  - rclpy.time.Time.from_msg(self.pose.header.stamp).nanoseconds)
        if age > self.max_pose_age * 1e9:
            self._throttled_warn("vehicle pose is stale; skipping projection")
            return None

        # Pose of base_link in the world (the FCU's local ENU).
        q = self.pose.pose.orientation
        R_world_base = quat_to_matrix(q.x, q.y, q.z, q.w)
        p_base = np.array([self.pose.pose.position.x,
                           self.pose.pose.position.y,
                           self.pose.pose.position.z])

        # Compose the camera pose: world <- base <- optical.
        R_world_opt = R_world_base @ self.R_base_opt
        p_cam = p_base + R_world_base @ self.t_base_opt

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        if fx <= 0.0 or fy <= 0.0:
            return None
        # Ray in the optical frame (Z forward, X right, Y down).
        ray_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])

        depth = self._depth_at(u, v)
        if depth is not None:
            # Depth is the distance ALONG the optical Z axis, not along the ray.
            point_opt = ray_opt * depth
            point = p_cam + R_world_opt @ point_opt
            return point, PadDetection.SOURCE_DEPTH, float(
                np.linalg.norm(point_opt)), self.pose.header.frame_id

        # Ground-plane fallback: where does the ray cross z = ground_z?
        ray_world = R_world_opt @ ray_opt
        if ray_world[2] > -1e-3:
            # Pointing level or upward — it never meets the floor ahead of us.
            return None
        t = (self.ground_z - p_cam[2]) / ray_world[2]
        if t <= 0.0:
            return None
        point = p_cam + t * ray_world
        return (point, PadDetection.SOURCE_GROUND_PLANE,
                float(t * np.linalg.norm(ray_world)), self.pose.header.frame_id)

    def _depth_at(self, u: float, v: float, window: int = 4) -> float | None:
        """Median finite depth in a small window, or None if unusable.

        A median over a patch, not the single pixel: the pad's edge pixels can
        carry the floor's depth instead of the pad's, and one such sample would
        throw the back-projection metres off.
        """
        if self.depth is None:
            return None
        h, w = self.depth.shape[:2]
        r, c = int(round(v)), int(round(u))
        patch = self.depth[max(0, r - window):min(h, r + window + 1),
                           max(0, c - window):min(w, c + window + 1)]
        valid = patch[np.isfinite(patch) & (patch > 0.05)]
        if valid.size < 4:
            return None
        return float(np.median(valid))

    def _ensure_mount_tf(self) -> bool:
        """Resolve and cache the static base_link -> optical-frame transform."""
        if self.R_base_opt is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.optical_frame, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            self._throttled_warn(
                f"no TF {self.base_frame} -> {self.optical_frame} yet ({exc}); "
                "detections will have no world position until it appears")
            return False

        t = tf.transform.translation
        q = tf.transform.rotation
        self.t_base_opt = np.array([t.x, t.y, t.z])
        self.R_base_opt = quat_to_matrix(q.x, q.y, q.z, q.w)
        self.get_logger().info(
            f"camera mount resolved: {self.base_frame} -> {self.optical_frame} "
            f"at ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")
        return True

    def _throttled_warn(self, text: str):
        self.get_logger().warn(text, throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = PadDetectorNode()
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
