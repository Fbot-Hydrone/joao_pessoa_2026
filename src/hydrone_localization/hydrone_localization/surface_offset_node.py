#!/usr/bin/env python3
"""surface_offset_node — give the EKF a rangefinder that always sees the FLOOR.

The real drone's EKF takes its height from the downward rangefinder
(EK3_SRC1_POSZ=2). Over a base the sensor jumps by the base's height and the
EKF, filtering the jump over ~1.2 s, makes the position controller climb (log
46, 2026-11-17: 1.62 -> 1.12 m in 50 ms, the vehicle rose to 2.3 m). This node
sits between the sensor and the EKF:

  DroneCAN rangefinder (RNGFND2) --MAVLink--> /mavros/distance_sensor/rangefinder
        (raw; every consumer on the Orin keeps reading it unchanged)
                     |
            surface_offset (hydrone_localization.surface_offset)
                     |
  /mavros/distance_sensor/rangefinder_floor --mavros--> DISTANCE_SENSOR -> RNGFND1
        (MAVLink type; the EKF's primary downward rangefinder)

so the EKF's z becomes height above the arena FLOOR everywhere, and the
mission's takeoff_alt means "metres above the floor" as it always did in sim.

Evidence that a raised surface is really under the sensor (else a step is
released after confirm_timeout_s, which makes the vehicle climb — safe):
  * the sensor's ground point lies in an arena zone marked raised, or
  * a belly-camera detection covers the sensor's spot in the image.

FCU parameters this needs (scripts/fcu_surface_offset.py applies/reverts them):
  RNGFND1_TYPE=10 (MAVLink, ORIENT=25), RNGFND2_TYPE=24 (DroneCAN, ORIENT=24:
  labelled UP so the EKF does not fuse it too), same POS_X/Y/Z and GNDCLR. On start this node reads them back and refuses to feed
  the FCU (loud error) if they do not match.

Topics out:
  /mavros/distance_sensor/rangefinder_floor  sensor_msgs/Range  (to the FCU)
  /hydrone/surface/offset                    std_msgs/Float32   s, metres
  /hydrone/surface/state                     std_msgs/String    JSON, 2 Hz +
                                                                every step
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import CameraInfo, Range
from std_msgs.msg import Float32, String

from hydrone_msgs.msg import PadDetection

from hydrone_localization.arena import (Arena, camera_vouches,
                                        rangefinder_in_image,
                                        sensor_ground_point)
from hydrone_localization.datum import DatumCal, DatumParams
from hydrone_localization.surface_offset import Params, SurfaceOffset

DISTANCE_SENSOR_ID = 132

# What the FCU must have for this node's output to be what the EKF uses.
REQUIRED_FCU = {"RNGFND1_TYPE": 10, "RNGFND1_ORIENT": 25,
                "RNGFND2_TYPE": 24,
                # NOT downward: EKF3 fuses every PITCH_270 rangefinder, so the
                # raw sensor must be labelled UP (see fcu_surface_offset.py).
                "RNGFND2_ORIENT": 24}


def rpy_of(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class SurfaceOffsetNode(Node):
    def __init__(self):
        super().__init__("surface_offset")
        d = self.declare_parameter
        d("raw_topic", "/mavros/distance_sensor/rangefinder")
        d("out_topic", "/mavros/distance_sensor/rangefinder_floor")
        d("pose_topic", "/mavros/local_position/pose")
        d("velocity_topic", "/mavros/local_position/velocity_local")
        d("state_topic", "/mavros/state")
        d("detections_topic", "/hydrone/pads/down/detections")
        d("camera_info_topic", "/down_cam/camera_info")
        # The drone always starts standing on the takeoff base.
        d("start_surface_m", 0.5)
        # Body-frame (ROS FLU) positions, metres. Must match RNGFND_POS_X/Y
        # and the belly camera's mount.
        d("rangefinder_xy", [0.14, 0.10])
        d("camera_xy", [0.065, 0.071])
        d("rate_hz", 20.0)
        # Arena (see hydrone_localization.arena). Empty zones = no zone
        # evidence: only the camera can vouch.
        d("arena_size_m", [0.0, 0.0])
        d("takeoff_base_arena", [0.0, 0.0, 0.0])
        d("zone_edges_x_m", [0.0])
        d("zone_raised", [False])
        d("zone_margin_m", 0.15)
        # Detector parameters (hydrone_localization.surface_offset.Params)
        d("jump_min_m", 0.10)
        d("noise_m", 0.03)
        d("max_tilt_deg", 25.0)
        d("confirm_timeout_s", 1.5)
        d("detection_max_age_s", 0.5)
        d("check_fcu_params", True)
        # Datum (hydrone_localization.datum): the range the sensor reads
        # standing on a flat surface. While disarmed and still, the feed is
        # biased until the FCU's z = height of the landing gear above the
        # floor; frozen at arming.
        d("ground_range_m", 0.17)
        d("calibrate_datum", True)

        p = lambda n: self.get_parameter(n).value
        self.so = SurfaceOffset(Params(
            jump_min_m=float(p("jump_min_m")), noise_m=float(p("noise_m")),
            max_tilt_deg=float(p("max_tilt_deg")),
            confirm_timeout_s=float(p("confirm_timeout_s"))))
        self.so.reset(float(p("start_surface_m")))
        self.datum = DatumCal(DatumParams(
            ground_range_m=float(p("ground_range_m"))))
        self.calibrate = bool(p("calibrate_datum"))
        self._datum_disabled_logged = False
        self.rng_xy = tuple(float(v) for v in p("rangefinder_xy"))
        self.cam_xy = tuple(float(v) for v in p("camera_xy"))
        self.det_age = float(p("detection_max_age_s"))
        self.rate_hz = float(p("rate_hz"))

        edges = [float(v) for v in p("zone_edges_x_m")]
        raised = [bool(v) for v in p("zone_raised")]
        if len(edges) < 2:
            edges, raised = [], []
        self.arena = Arena(size_xy=tuple(float(v) for v in p("arena_size_m")),
                           takeoff_base=tuple(float(v) for v in
                                              p("takeoff_base_arena")),
                           zone_edges_x=edges, zone_raised=raised,
                           margin_m=float(p("zone_margin_m")))
        self.arena.validate()

        self.pose = None
        self.vz = 0.0
        self.armed = False
        self.fx = None
        self.cx = self.cy = None
        self.dets = []                 # (t, u, v, r)
        self.fcu_ok = not bool(p("check_fcu_params"))
        self._n_steps_logged = 0
        self._last_out = None

        self.pub = self.create_publisher(Range, p("out_topic"), 10)
        self.pub_s = self.create_publisher(Float32, "/hydrone/surface/offset",
                                           10)
        self.pub_state = self.create_publisher(String,
                                               "/hydrone/surface/state", 10)
        sd = qos_profile_sensor_data
        self.create_subscription(Range, p("raw_topic"), self._cb_range, sd)
        self.create_subscription(PoseStamped, p("pose_topic"), self._cb_pose,
                                 sd)
        self.create_subscription(TwistStamped, p("velocity_topic"),
                                 self._cb_vel, sd)
        self.create_subscription(State, p("state_topic"), self._cb_state, 10)
        self.create_subscription(PadDetection, p("detections_topic"),
                                 self._cb_det, 10)
        self.create_subscription(CameraInfo, p("camera_info_topic"),
                                 self._cb_info, sd)

        self.cli_rate = self.create_client(MessageInterval,
                                           "/mavros/set_message_interval")
        # MAVROS 2 mirrors the FCU parameters as ROS parameters of /mavros/param
        # (filled by its background pull, which can take a minute).
        self.cli_param = self.create_client(GetParameters,
                                            "/mavros/param/get_parameters")
        self._rate_set = False
        self._param_bad = []
        self.create_timer(1.0, self._housekeeping)
        self.create_timer(0.5, self._publish_state)

        self.get_logger().info(
            f"surface_offset ready — starting on a {self.so.s:.2f} m surface; "
            f"rangefinder at {self.rng_xy}, belly camera at {self.cam_xy}; "
            f"{len(raised)} zone(s) {raised}.")

    # ── inputs ────────────────────────────────────────────────────────────

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cb_pose(self, msg):
        self.pose = msg

    def _cb_vel(self, msg):
        self.vz = float(msg.twist.linear.z)

    def _cb_state(self, msg):
        if msg.armed and not self.armed and not self.arena.latched \
                and self.pose is not None:
            pp = self.pose.pose.position
            _, _, yaw = rpy_of(self.pose.pose.orientation)
            self.arena.latch(pp.x, pp.y, yaw)
            self.get_logger().info(
                f"arena latched at the first arming: takeoff base "
                f"{tuple(self.arena.takeoff_base)} (arena) = FCU "
                f"({pp.x:.2f}, {pp.y:.2f}, {math.degrees(yaw):.0f} deg).")
        self.armed = bool(msg.armed)

    def _cb_info(self, msg):
        if msg.k[0] > 0:
            self.fx, self.cx, self.cy = msg.k[0], msg.k[2], msg.k[5]

    def _cb_det(self, msg):
        if msg.camera and msg.camera != "down":
            return
        self.dets.append((self._now(), float(msg.u), float(msg.v),
                          float(msg.radius_px)))
        self.dets = self.dets[-20:]

    # ── evidence ──────────────────────────────────────────────────────────

    def _evidence(self, depth_m):
        why = []
        if not self.arena.zone_edges_x:
            why.append("no zones configured: every step is held")
        elif self.pose is not None and self.arena.latched:
            pp = self.pose.pose.position
            _, _, yaw = rpy_of(self.pose.pose.orientation)
            gx, gy = sensor_ground_point(pp.x, pp.y, yaw, self.rng_xy)
            ax, ay = self.arena.to_arena(gx, gy)
            if self.arena.raised_at(ax, ay):
                why.append(f"zone@({ax:.1f},{ay:.1f})")
        if self.fx:
            now = self._now()
            fresh = [(u, v, r) for t, u, v, r in self.dets
                     if now - t <= self.det_age]
            if fresh:
                spot = rangefinder_in_image(self.rng_xy, self.cam_xy,
                                            depth_m, self.fx, self.cx,
                                            self.cy)
                if camera_vouches(fresh, spot):
                    why.append("camera")
        return why

    # ── the work ──────────────────────────────────────────────────────────

    def _cb_range(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self._now()
        # No local position yet (EKF still aligning): feed anyway, level.
        # Every second without the feed is a second the EKF spends on the
        # barometer, which is what corrupts its height datum (datum.py).
        if self.pose is not None:
            roll, pitch, _ = rpy_of(self.pose.pose.orientation)
        else:
            roll = pitch = 0.0
        r = float(msg.range)
        why = self._evidence(max(r * math.cos(roll) * math.cos(pitch), 0.1))
        active = self.armed and r > msg.min_range + 0.1
        r_out, h = self.so.update(t, r, roll, pitch, self.vz,
                                  evidence=bool(why), active=active)
        while self._n_steps_logged < len(self.so.steps):
            st = self.so.steps[self._n_steps_logged]
            self._n_steps_logged += 1
            self.get_logger().warn(
                f"surface step {st.size:+.3f} m at tilt {st.tilt_deg:.1f} deg:"
                f" s {st.s_before:.2f} -> {st.s_after:.2f} ({st.reason}); "
                f"evidence {why or 'none yet'}")
            self._publish_state(event="step")
        for st in self.so.steps[-3:]:
            if not st.kept and not getattr(st, "_logged_release", False):
                st._logged_release = True
                self.get_logger().warn(
                    f"surface step from t={st.t:.2f} RELEASED: no zone and no "
                    f"camera vouched for it in {self.so.p.confirm_timeout_s} s "
                    f"— s back to {self.so.s:.2f}, the vehicle will climb.")
        if r_out is None:
            return                    # unusable sample: the FCU keeps the last
        if self.calibrate and self.pose is not None:
            z = float(self.pose.pose.position.z)
            if self.datum.update(t, h, z, self.armed):
                _, e, b = self.datum.adjustments[-1]
                self.get_logger().warn(
                    f"height datum: FCU z {z:.2f} is {e:+.2f} m off the floor "
                    f"height {h - self.datum.p.ground_range_m:.2f} — feed "
                    f"bias now {b:+.2f} m.")
            if self.datum.disabled and not self._datum_disabled_logged:
                self._datum_disabled_logged = True
                self.get_logger().error(
                    f"height datum calibration OFF: {self.datum.disabled}. "
                    f"z is NOT height above the floor — do not fly.")
        # Feed even before the FCU's parameters are confirmed: a gap sends
        # the EKF to the barometer, and in PLAIN mode an id-10 message has no
        # MAVLink backend to land in, so it is harmless.
        if not self.fcu_ok:
            self.get_logger().error(
                "FCU rangefinder parameters NOT confirmed "
                f"({self._param_bad or 'not read yet'}) — if the FCU is in "
                "PLAIN mode, run scripts/fcu_surface_offset.py apply.",
                throttle_duration_sec=10.0)
        tilt_c = max(math.cos(roll) * math.cos(pitch), 0.5)
        r_out = r_out + self.datum.bias / tilt_c
        out = Range()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id or "rangefinder_link"
        out.radiation_type = msg.radiation_type
        out.field_of_view = msg.field_of_view
        out.min_range = msg.min_range
        out.max_range = msg.max_range + 2.0
        out.range = float(r_out)
        self.pub.publish(out)
        self._last_out = (r, r_out, h)
        self.pub_s.publish(Float32(data=float(self.so.s)))

    def _publish_state(self, event="tick"):
        last = self._last_out or (None, None, None)
        self.pub_state.publish(String(data=json.dumps({
            "event": event, "s": round(self.so.s, 3),
            "raw": last[0], "out": last[1], "h_floor": last[2],
            "armed": self.armed, "fcu_ok": self.fcu_ok,
            "datum_bias": round(self.datum.bias, 3),
            "datum_ok": bool(self.datum.ok or not self.calibrate),
            "datum_off": self.datum.disabled,
            "fcu_z": (round(self.pose.pose.position.z, 3)
                      if self.pose is not None else None),
            "pending": self.so._pending is not None,
            "steps": len(self.so.steps)})))

    # ── FCU plumbing ──────────────────────────────────────────────────────

    def _housekeeping(self):
        if not self._rate_set and self.cli_rate.service_is_ready():
            req = MessageInterval.Request()
            req.message_id = DISTANCE_SENSOR_ID
            req.message_rate = self.rate_hz
            fut = self.cli_rate.call_async(req)
            fut.add_done_callback(self._rate_done)
        if not self.fcu_ok and self.cli_param.service_is_ready():
            fut = self.cli_param.call_async(
                GetParameters.Request(names=list(REQUIRED_FCU)))
            fut.add_done_callback(self._param_done)

    def _rate_done(self, fut):
        res = fut.result()
        if res is not None and res.success:
            self._rate_set = True
            self.get_logger().info(
                f"FCU asked for DISTANCE_SENSOR at {self.rate_hz:.0f} Hz.")

    def _param_done(self, fut):
        res = fut.result()
        if res is None or len(res.values) != len(REQUIRED_FCU):
            return
        bad = []
        for (name, want), v in zip(REQUIRED_FCU.items(), res.values):
            if v.type == 2:
                val = float(v.integer_value)
            elif v.type == 3:
                val = v.double_value
            else:                      # NOT_SET: not pulled from the FCU yet
                bad.append(f"{name}=?")
                continue
            if abs(val - want) > 1e-6:
                bad.append(f"{name}={val:g} (want {want})")
        self._param_bad = bad
        if not bad:
            self.fcu_ok = True
            self.get_logger().info("FCU rangefinder parameters match — "
                                   "feeding RNGFND1.")

def main(args=None):
    rclpy.init(args=args)
    node = SurfaceOffsetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
