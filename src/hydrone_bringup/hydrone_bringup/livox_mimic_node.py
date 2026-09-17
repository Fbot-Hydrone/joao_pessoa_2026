#!/usr/bin/env python3
"""
livox_mimic_node — publishes a Livox Mid-360's ROS 2 topics from a BiguaSim
DepthCamera that the bridge spins.

SIM-ONLY, and the Livox counterpart of nothing else in this workspace: the
Kopis carries ONE sensor, this lidar, and shares no topic, node or frame with
the Holybro/ZED stack. On the real drone `livox_ros_driver2` publishes these
topics and this node is not launched.

Outputs (the real driver's names, so a consumer cannot tell sim from real):
  /livox/lidar   sensor_msgs/PointCloud2   10 Hz, whatever was gathered since
                                           the last one
  /livox/imu     sensor_msgs/Imu           the lidar's built-in IMU, if wired
  TF: base_link -> livox_frame             static, the unit's mounting pose

HOW IT WORKS, AND WHY IT IS SHAPED LIKE THE DEVICE
--------------------------------------------------
A Mid-360 appends every return it measures to an internal buffer and, on a
fixed 10 Hz tick, ships the buffer and starts an empty one. It is not a
snapshot and it is not cumulative: each message is exactly the points collected
during one 100 ms window, and nothing is retained after it goes out.

This node does the same thing with a different source of points. BiguaSim has
no lidar sensor of any kind (`biguasim/sensors.py`: cameras, a conical
RangeFinderSensor, and inertial sensors — nothing that sweeps), so the
simulated unit is a depth camera that ardubridge rotates as the simulation
steps. Every depth frame is back-projected into points, rotated to the yaw the
camera was actually at, and appended to the buffer; the timer flushes it.

    depth frame ──project──> points ──> buffer
                                          │
       10 Hz timer ──> rotate each frame by its own yaw ──> publish + clear

THE YAW IS TOLD, NOT COUNTED. `in_spin` carries the sensor's orientation as the
bridge knows it, stamped on the same clock as the frames, and each frame is
paired with the last spin sample at or before its own stamp. Counting frames
and multiplying instead would look identical until a frame was dropped, at
which point every later point in the run would be rotated wrong with nothing
in the graph saying so. If NOTHING publishes `in_spin` — the `--world` path,
where the sensor lives in another process and cannot be rotated — every frame
is treated as yaw 0, and the cloud is an honest single wedge rather than a
guess.

WHAT THIS IS NOT
----------------
A Mid-360 sees 360 x 59 degrees and returns ~200,000 points/s from a
non-repetitive pattern; a rotated depth camera returns a dense, perfectly
regular grid per wedge, covers only the azimuths it was pointed at during the
window, and has a blind cone above and below. Ranges are exact — no dropout on
dark or specular surfaces. Anything tuned against this data (a LIO's feature
extraction in particular) is tuned against a friendlier sensor than the one on
the drone.

POINT LAYOUT
------------
Mirrors `livox_ros_driver2`'s PointCloud2 (its `LivoxPointXyzrtlt`), so
FAST-LIO/Point-LIO style consumers read it by the field names they expect:

    x,y,z (float32) | intensity (float32) | tag (u8) | line (u8) | timestamp (f64)
    offsets 0,4,8,      12,                  16,        17,         18   -> 26 bytes

`timestamp` is the absolute capture time of the point's frame in nanoseconds,
as the driver publishes it, so a deskewing consumer sees the frames spread
across the 100 ms the message covers — which is real, the vehicle moves during
a window. `line` carries the frame's index within the message; on the real unit
it is the laser id, here it answers the same debugging question ("which sweep
did this come from"). `intensity` is a CONSTANT: a depth camera measures no
reflectance, and inventing one from range would be a signal a consumer could
tune against that the drone will never reproduce.

If your consumer wants the aligned 32-byte variant of that struct, change
POINT_STEP and the timestamp offset below; nothing else depends on it.
"""

import array
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Vector3Stamped
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster


# The frame the real driver stamps its cloud with (livox_ros_driver2's
# `frame_id` default). Axes are the Livox convention — X forward, Y left,
# Z up — which is also ROS body convention, so no swap is needed downstream.
FRAME_LIVOX = "livox_frame"
FRAME_BASE = "base_link"

# One point, exactly as livox_ros_driver2 lays it out. Explicit offsets and
# itemsize because the packing IS the contract: a numpy default would align
# `timestamp` to 24 and every consumer would read the wrong bytes.
POINT_STEP = 26
LIVOX_POINT = np.dtype({
    "names": ["x", "y", "z", "intensity", "tag", "line", "timestamp"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"],
    "offsets": [0, 4, 8, 12, 16, 17, 18],
    "itemsize": POINT_STEP,
})

POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
    PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
    PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
]

# UE's depth buffer saturates at 655.04 m (65504 cm, float16) where there is no
# geometry. It is not a return and must never become a point 655 m away.
SKY_SENTINEL = 655.0


class LivoxMimicNode(Node):

    def __init__(self):
        super().__init__("livox_mimic")

        # ── Inputs: where ardubridge publishes the spinning depth camera ────
        # Overridden by phase4_sim.launch.py from the biguasim scenario file,
        # so the agent name and the sensor's mounting pose have ONE home. The
        # defaults below only matter for a standalone `ros2 run`.
        self.declare_parameter("in_depth", "/biguasim/uav0_id0/DepthCamera")
        self.declare_parameter("in_depth_info",
                               "/biguasim/uav0_id0/DepthCamera/camera_info")
        # Where the sensor was pointing, published by the bridge each step as
        # [roll, pitch, yaw] in DEGREES. See the module docstring: this is what
        # makes a dropped frame harmless.
        self.declare_parameter("in_spin", "/biguasim/uav0_id0/DepthCamera/spin")
        # The Mid-360 carries its own IMU and the driver publishes it. Empty
        # disables the republish, which is the default because the BiguaSim
        # agent has no IMUSensor yet — and its DynamicsSensor IMU is not one:
        # that reports GLOBAL-frame acceleration with no gravity term, and the
        # bridge's encoder reads angular ACCELERATION into the angular velocity
        # field. Feeding either to a LIO fails on the first gravity alignment.
        self.declare_parameter("in_imu", "")

        # ── Outputs: the real driver's topic names ──────────────────────────
        self.declare_parameter("out_cloud", "/livox/lidar")
        self.declare_parameter("out_imu", "/livox/imu")
        self.declare_parameter("frame_id", FRAME_LIVOX)

        # ── The buffer ──────────────────────────────────────────────────────
        # The real unit's publish rate. Wall clock, like the device's: when the
        # simulator runs below real time the window simply holds fewer frames,
        # which is exactly what a slow sensor would do.
        self.declare_parameter("publish_rate_hz", 10.0)
        # Mid-360 datasheet: 0.1 m blind zone, 40 m at 10% reflectivity and
        # 70 m at 80%. Points outside are dropped rather than clamped — a
        # clamped return is a wall that is not there.
        self.declare_parameter("min_range", 0.1)
        self.declare_parameter("max_range", 70.0)
        # Take every Nth pixel in each direction. A 182x182 frame is ~33k
        # points and the camera runs far faster than 10 Hz, so a window can
        # hold several hundred thousand; stride 3 lands nearer the real unit's
        # ~20k per message. Left at 1 because throwing away returns should be a
        # decision, not a default.
        self.declare_parameter("stride", 1)
        # No reflectance is measured; see the module docstring.
        self.declare_parameter("intensity", 100.0)

        # ── Where the unit sits on the airframe (base_link -> livox_frame) ──
        self.declare_parameter("mount_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("mount_rpy_deg", [0.0, 0.0, 0.0])

        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.frame_id = p("frame_id")
        self.min_range = float(p("min_range"))
        self.max_range = float(p("max_range"))
        self.stride = max(1, int(p("stride")))
        self.intensity = float(p("intensity"))

        self.pub_cloud = self.create_publisher(PointCloud2, p("out_cloud"), 10)

        # RELIABLE (the default) on purpose, both ways. The real driver's
        # subscribers — FAST-LIO and friends — subscribe with default QoS, and
        # a BEST_EFFORT publisher would simply never match them: no error, no
        # points, and nothing in the graph saying why. Nothing here is
        # transient_local either: a lidar message is a 100 ms window, and a
        # late joiner must not be handed a stale one as if it were current.
        self.create_subscription(Image, p("in_depth"), self._cb_depth, 10)
        self.create_subscription(CameraInfo, p("in_depth_info"),
                                 self._cb_info, 10)
        # Deep queue: the bridge publishes this every simulation step, which is
        # many times faster than frames arrive, and a frame must still find its
        # own sample after a scheduling hiccup.
        self.create_subscription(Vector3Stamped, p("in_spin"), self._cb_spin, 200)

        self.pub_imu = None
        in_imu = (p("in_imu") or "").strip()
        if in_imu:
            self.pub_imu = self.create_publisher(Imu, p("out_imu"), 10)
            self.create_subscription(Imu, in_imu, self._cb_imu, 20)

        # Static mounting transform, published once, exactly as a real driver's
        # launch would supply it.
        self._publish_mount_tf(p("mount_xyz"), p("mount_rpy_deg"))

        self._info = None            # latest CameraInfo (intrinsics)
        self._points = []            # the buffer: (N, 3) arrays, one per frame
        self._stamps = []            # each one's capture time, ns
        self._spin = deque(maxlen=400)   # (stamp_ns, yaw_rad) from the bridge
        self._grid = None            # cached pixel grid
        self._grid_shape = None
        self._warned_info = False
        self._warned_spin = False
        # A lidar that publishes nothing must say why: silence is the one
        # symptom that looks identical for "no frames", "no returns in them",
        # "no intrinsics" and "the node is not running at all".
        self._frames = 0
        self._empty_frames = 0
        self._published = 0

        rate = float(p("publish_rate_hz"))
        self.create_timer(1.0 / rate, self._flush)

        self.get_logger().info(
            f"livox_mimic ready — buffering {p('in_depth')} -> "
            f"{p('out_cloud')} in '{self.frame_id}' at {rate:g} Hz"
            + ("" if self.pub_imu else " (no IMU wired)"))

    # ────────────────────────────────────────────────────────────────────────
    # Mounting transform
    # ────────────────────────────────────────────────────────────────────────

    def _publish_mount_tf(self, xyz, rpy_deg):
        """base_link -> livox_frame, from the scenario file's sensor block.

        The unit's own spin lives INSIDE livox_frame (each frame is rotated as
        it is buffered), so this transform is static even though the camera
        standing in for the lidar is turning.
        """
        self._tf_static = StaticTransformBroadcaster(self)
        r, pch, y = (math.radians(float(v)) for v in rpy_deg)
        cr, sr = math.cos(r / 2), math.sin(r / 2)
        cp, sp = math.cos(pch / 2), math.sin(pch / 2)
        cy, sy = math.cos(y / 2), math.sin(y / 2)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = FRAME_BASE
        t.child_frame_id = self.frame_id
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy
        self._tf_static.sendTransform(t)

    # ────────────────────────────────────────────────────────────────────────
    # Filling the buffer
    # ────────────────────────────────────────────────────────────────────────

    def _cb_info(self, msg: CameraInfo):
        self._info = msg

    def _cb_spin(self, msg: Vector3Stamped):
        """Where the sensor was pointing, as the bridge knows it."""
        stamp = (msg.header.stamp.sec * 10**9) + msg.header.stamp.nanosec
        self._spin.append((stamp, math.radians(msg.vector.z)))

    def _yaw_at(self, stamp_ns):
        """The sensor's yaw when the frame stamped `stamp_ns` was rendered.

        The last sample at or before the frame — the pose that was in effect
        while it was being taken, not one commanded after it. The bridge
        publishes the sample BEFORE the frame of the same step for exactly
        this reason.

        Asked at FLUSH time rather than on arrival. Both topics come from the
        same process, but they are separate subscriptions with separate
        callbacks and ROS orders neither against the other: a frame whose own
        spin sample has been published but not yet delivered would silently
        take the previous step's angle, one whole step out. By the time the
        window is flushed, every sample covering it has long since arrived.
        """
        if not self._spin:
            if not self._warned_spin:
                self._warned_spin = True
                self.get_logger().warn(
                    "no spin data — treating every frame as yaw 0, so the "
                    "cloud is a single wedge. Expected when the world runs in "
                    "another process (--world), which cannot rotate the "
                    "sensor; otherwise check that ardubridge is publishing "
                    "the spin topic.")
            return 0.0
        yaw = self._spin[0][1]
        for sample_stamp, sample_yaw in self._spin:
            if sample_stamp > stamp_ns:
                break
            yaw = sample_yaw
        return yaw

    def _cb_depth(self, msg: Image):
        if self._info is None:
            if not self._warned_info:
                self._warned_info = True
                self.get_logger().warn(
                    "depth frames arriving with no CameraInfo yet — no "
                    "intrinsics, so no points. Check that the DepthCamera's "
                    "camera_info topic is being published.")
            return

        self._frames += 1
        points = self._project(msg)
        if points is None or not len(points):
            self._empty_frames += 1
            self.get_logger().warn(
                f"{self._empty_frames}/{self._frames} depth frames produced NO "
                f"returns: every pixel was NaN, the sky sentinel, or outside "
                f"[{self.min_range}, {self.max_range}] m. If the whole run "
                "looks like this, check that the camera is pointed at "
                "geometry and that the depth really is in metres.",
                throttle_duration_sec=10.0)
            return

        stamp = (msg.header.stamp.sec * 10**9) + msg.header.stamp.nanosec
        if stamp == 0:                      # a standalone publisher with no clock
            stamp = self.get_clock().now().nanoseconds

        # Buffered in the SENSOR's frame; the yaw is applied at flush, once
        # the spin samples covering this window are certainly all in. See
        # _yaw_at.
        self._points.append(points)
        self._stamps.append(stamp)

    def _project(self, msg: Image):
        """Back-project one depth frame into (N, 3) points in the sensor frame.

        Returns points in the LIVOX/ROS convention (X forward, Y left, Z up),
        not the optical one the depth image is written in, and drops everything
        that is not a real return — NaN, the sky sentinel, and anything outside
        the unit's range envelope.
        """
        h, w = msg.height, msg.width
        if h == 0 or w == 0:
            return None
        depth = np.frombuffer(msg.data, dtype=np.float32)
        if depth.size != h * w:
            self.get_logger().warn(
                f"depth {msg.encoding} {w}x{h} carries {depth.size} floats; "
                "refusing to guess its layout", throttle_duration_sec=10.0)
            return None
        depth = depth.reshape(h, w)

        k = self._info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warn("CameraInfo has no focal length; no points",
                                   throttle_duration_sec=10.0)
            return None

        if self._grid_shape != (h, w):
            vv, uu = np.mgrid[0:h, 0:w]
            self._grid = (uu.astype(np.float32), vv.astype(np.float32))
            self._grid_shape = (h, w)
        uu, vv = self._grid

        if self.stride > 1:
            depth = depth[::self.stride, ::self.stride]
            uu = uu[::self.stride, ::self.stride]
            vv = vv[::self.stride, ::self.stride]

        keep = (np.isfinite(depth) & (depth > self.min_range)
                & (depth < self.max_range) & (depth < SKY_SENTINEL))
        if not keep.any():
            return None

        d = depth[keep]
        x_opt = (uu[keep] - np.float32(cx)) / np.float32(fx) * d
        y_opt = (vv[keep] - np.float32(cy)) / np.float32(fy) * d

        # Optical (Z out of the lens, X right, Y down) -> Livox/ROS body axes.
        points = np.empty((d.size, 3), dtype=np.float32)
        points[:, 0] = d
        points[:, 1] = -x_opt
        points[:, 2] = -y_opt
        return points

    # ────────────────────────────────────────────────────────────────────────
    # Emptying it
    # ────────────────────────────────────────────────────────────────────────

    def _flush(self):
        """Ship the window and start an empty one — the device's 10 Hz tick.

        Nothing is published when nothing was gathered. An empty cloud is not
        what a silent sensor looks like to a consumer (several LIOs treat one
        as a frame with no features and step their state forward), and the
        honest signal for "no returns" is no message.
        """
        if not self._points:
            if self._frames == 0:
                self.get_logger().warn(
                    "no depth frames have arrived at all. Check the topic "
                    f"name and that ardubridge is running: "
                    f"ros2 topic hz {self.get_parameter('in_depth').value}",
                    throttle_duration_sec=10.0)
            return
        points, stamps = self._points, self._stamps
        self._points, self._stamps = [], []

        total = sum(len(p) for p in points)
        msg = PointCloud2()
        # Stamped with the FIRST frame in the window, so the per-point
        # timestamps run forward from the header across it — what a deskewing
        # consumer expects of a spinning lidar.
        msg.header.stamp = rclpy.time.Time(nanoseconds=stamps[0]).to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = total
        msg.fields = POINT_FIELDS
        msg.is_bigendian = False
        msg.point_step = POINT_STEP
        msg.row_step = POINT_STEP * total
        # Every point kept is a real return; nothing invalid was buffered.
        msg.is_dense = True

        buf = np.empty(total, dtype=LIVOX_POINT)
        at = 0
        for index, (pts, stamp) in enumerate(zip(points, stamps)):
            n = len(pts)
            # Where the sensor was actually pointing for this frame.
            yaw = self._yaw_at(stamp)
            if yaw:
                c, sn = math.cos(yaw), math.sin(yaw)
                x, y = pts[:, 0].copy(), pts[:, 1].copy()
                pts[:, 0] = c * x - sn * y
                pts[:, 1] = sn * x + c * y
            chunk = buf[at:at + n]
            chunk["x"] = pts[:, 0]
            chunk["y"] = pts[:, 1]
            chunk["z"] = pts[:, 2]
            chunk["intensity"] = self.intensity
            chunk["tag"] = 0                   # Livox tag 0: a confident return
            chunk["line"] = index % 256        # which frame in the window
            chunk["timestamp"] = float(stamp)  # absolute ns, as the driver does
            at += n

        # array.array, not bytes: rclpy's uint8[] setter walks a bytes object
        # element by element in Python, which at a few hundred thousand points
        # a second is the difference between keeping up and not.
        msg.data = array.array("B", buf.tobytes())
        self.pub_cloud.publish(msg)

        self._published += 1
        megabytes = (total * POINT_STEP) / 1e6
        if self._published == 1:
            self.get_logger().info(
                f"first cloud out: {total} points from {len(points)} frames, "
                f"{megabytes:.1f} MB")
        else:
            self.get_logger().info(
                f"{self._published} clouds; last {total} points "
                f"({megabytes:.1f} MB) from {len(points)} frames",
                throttle_duration_sec=10.0)
        # A window at full resolution is far larger than anything else on this
        # graph -- 182x182 is 33k points per FRAME, and a 10 Hz window holds
        # every frame the camera produced. Past a couple of megabytes the
        # default DDS configuration fragments the sample across hundreds of
        # datagrams and subscribers with ordinary socket buffers (RViz above
        # all) start dropping whole messages, which looks exactly like a
        # sensor that is not publishing.
        if megabytes > 2.0:
            self.get_logger().warn(
                f"cloud is {megabytes:.1f} MB. If RViz shows nothing while "
                "this node reports publishing, that is the reason: raise "
                "stride (3 lands near the real unit's ~20k points) or lower "
                "publish_rate_hz.", throttle_duration_sec=20.0)

    # ────────────────────────────────────────────────────────────────────────
    # The unit's own IMU
    # ────────────────────────────────────────────────────────────────────────

    def _cb_imu(self, msg: Imu):
        """Republish the agent's IMU as the lidar's own.

        Frame only — the samples are passed through untouched. That is correct
        while the unit is mounted square to the body (the scenario file gives
        the sensor no rotation); if it ever gains one, the accelerations and
        rates have to be rotated into livox_frame here, or a LIO will fight a
        constant misalignment between its two inputs.
        """
        msg.header.frame_id = self.frame_id
        self.pub_imu.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxMimicNode()
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
