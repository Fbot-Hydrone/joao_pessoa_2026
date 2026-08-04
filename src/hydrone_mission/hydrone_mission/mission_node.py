#!/usr/bin/env python3
"""
hydrone_mission — Top-level mission state machine.

This is the highest-level node.  It knows about each competition phase,
orchestrates all other nodes via services, tracks score, enforces timing
rules, and prints the mandatory terminal logs required by the judges.

Phase state machines
────────────────────
PHASE 1 — Localisation & Mapping
  IDLE → TAKEOFF → SCANNING → ROUTING → VISITING[n] → RETURN_HOME → LANDED

PHASE 2 — Package Transport
  IDLE → TAKEOFF → GO_PICKUP[n] → PICKUP → GO_DROPOFF[n] → DROPOFF → RETURN_HOME → LANDED

PHASE 3 — Human-Swarm Interaction
  IDLE → TAKEOFF → APPROACH_HUMAN → WAIT_GESTURE → EXECUTE_GESTURE → … → RETURN_HOME → LANDED

PHASE 4 — Confined Space Navigation
  IDLE → TAKEOFF → ENTER_MAZE → SEARCH_QR → EXIT_MAZE → LAND_ARENA → LANDED

Subscribed topics:
  /hydrone/controller/drone_pose  (geometry_msgs/PoseStamped)
  /hydrone/controller/status      (std_msgs/String)
  /hydrone/vision/landing_bases   (hydrone_msgs/LandingBase)
  /hydrone/vision/human_gesture   (hydrone_msgs/HumanGesture)
  /hydrone/vision/qr_codes        (hydrone_msgs/QRCode)

Published topics:
  /hydrone/mission_state          (hydrone_msgs/MissionState)
  /hydrone/controller/cmd_pose    (geometry_msgs/PoseStamped)   — direct overrides

Services called:
  /hydrone/controller/arm
  /hydrone/controller/takeoff
  /hydrone/controller/land
  /hydrone/nav/start_exploration
  /hydrone/nav/return_home
  /hydrone/nav/enter_maze

Services exposed:
  /hydrone/mission/start          (hydrone_msgs/srv/SetPhase)
  /hydrone/mission/abort          (std_srvs/Trigger)
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg      import String
from std_srvs.srv      import Trigger, SetBool

from hydrone_msgs.msg  import LandingBase, MissionState, HumanGesture, QRCode
from hydrone_msgs.srv  import SetPhase


# ── Scoring constants (per CBR 2026 rules) ──────────────────────────────────

P1_BASE_VISIT_PTS   = 20     # per unique base visit
P1_REPEAT_PENALTY   = -5     # per repeated visit
P1_AUTONOMOUS_MULT  = 2.0    # multiplier if autonomous return

P2_PICKUP_PTS       = 40     # per kit picked up
P2_DELIVERY_PTS     = 20     # per kit delivered correctly
P2_DROP_PENALTY     = -5     # per dropped kit (wrong location)
P2_AUTONOMOUS_MULT  = 2.0

P3_BASE_VISIT_PTS   = 20
P3_REPEAT_PENALTY   = -5
P3_TWO_DRONE_REPEAT = -10
P3_AUTONOMOUS_MULT  = 2.0

P4_MAZE_PTS         = 50     # for completing maze traverse
P4_QR_PTS           = 20     # per unique QR code
P4_AUTONOMOUS_MULT  = 2.0

OPEN_HW_MULT        = 2.0    # doubled score for open-hardware drones

# Phase attempt time limits (seconds)
ATTEMPT_TIME_LIMITS = {1: 600, 2: 600, 3: 900, 4: 600}


# ── Gesture → nav action mapping (Phase 3) ──────────────────────────────────

GESTURE_ACTION = {
    "both_arms_up":    "LAND_HERE",
    "right_arm_point": "MOVE_RIGHT",
    "left_arm_point":  "MOVE_LEFT",
    "arms_forward":    "MOVE_FORWARD",
    "arms_back":       "MOVE_BACK",
    "arms_cross":      "STOP",
    "thumbs_up":       "TAKEOFF",
    "thumbs_down":     "LAND",
}


class HydroneMissionNode(Node):

    def __init__(self):
        super().__init__("hydrone_mission")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("open_hardware",   False)
        self.declare_parameter("phase",           1)
        self.declare_parameter("use_two_drones",  False)
        # Monotonic-clock budget: does not stretch when the sim runs below
        # real-time. Overridden by hydrone_bringup/config/timeouts.yaml.
        self.declare_parameter("service_wait_timeout", 2.0)

        self.open_hardware  = self.get_parameter("open_hardware").value
        self.active_phase   = self.get_parameter("phase").value
        self.two_drones     = self.get_parameter("use_two_drones").value
        self.svc_wait_tmo   = self.get_parameter("service_wait_timeout").value

        # ── Scoring & state ──────────────────────────────────────────────────
        self.score          = 0.0
        self.state          = "IDLE"
        self.attempt_start  = 0.0
        self.human_detected = False   # Phase 3: operator recognised

        # Phase-1/3 base tracking
        self.visited_bases: set[int] = set()

        # Phase-2 package tracking
        self.kits_picked  = 0
        self.kits_delivered = 0

        # Phase-4 QR tracking
        self.qr_found: set[str] = set()
        self.maze_completed = False

        # Phase-3 gesture buffer (latest)
        self._last_gesture: str = ""

        # ── QoS ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ─────────────────────────────────────────────────────
        self.sub_pose    = self.create_subscription(
            PoseStamped, "/hydrone/controller/drone_pose",
            self._cb_pose, sensor_qos)
        self.sub_ctrl_st = self.create_subscription(
            String, "/hydrone/controller/status",
            self._cb_ctrl_status, 10)
        self.sub_base    = self.create_subscription(
            LandingBase, "/hydrone/vision/landing_bases",
            self._cb_base_detected, 10)
        self.sub_gesture = self.create_subscription(
            HumanGesture, "/hydrone/vision/human_gesture",
            self._cb_gesture, 10)
        self.sub_qr      = self.create_subscription(
            QRCode, "/hydrone/vision/qr_codes",
            self._cb_qr, 10)

        # ── Publishers ──────────────────────────────────────────────────────
        self.pub_state = self.create_publisher(
            MissionState, "/hydrone/mission_state", 10)

        # ── Service clients ──────────────────────────────────────────────────
        self.cli_arm      = self.create_client(Trigger,  "/hydrone/controller/arm")
        self.cli_takeoff  = self.create_client(SetBool,  "/hydrone/controller/takeoff")
        self.cli_land     = self.create_client(Trigger,  "/hydrone/controller/land")
        self.cli_explore  = self.create_client(Trigger,  "/hydrone/nav/start_exploration")
        self.cli_home     = self.create_client(Trigger,  "/hydrone/nav/return_home")
        self.cli_maze     = self.create_client(Trigger,  "/hydrone/nav/enter_maze")

        # ── Services exposed ─────────────────────────────────────────────────
        self.srv_start = self.create_service(
            SetPhase, "/hydrone/mission/start", self._svc_start)
        self.srv_abort = self.create_service(
            Trigger,  "/hydrone/mission/abort", self._svc_abort)

        # State machine timer (5 Hz)
        self.create_timer(0.2, self._state_machine_tick)
        # Score publisher timer (2 Hz)
        self.create_timer(0.5, self._publish_state)

        self.get_logger().info(
            f"HydroneMissionNode ready — Phase {self.active_phase}, "
            f"open_hw={self.open_hardware}")

    # ────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ────────────────────────────────────────────────────────────────────────

    def _cb_pose(self, msg: PoseStamped):
        self._drone_pose = msg

    def _cb_ctrl_status(self, msg: String):
        self._ctrl_status = msg.data

    def _cb_base_detected(self, msg: LandingBase):
        """Called when vision confirms a landing — update score accordingly."""
        bid = msg.base_id

        if self.state not in ("VISITING", "LANDING_ON_BASE", "WAIT_GESTURE",
                              "EXECUTE_GESTURE"):
            return

        if bid not in self.visited_bases:
            # First visit — award points
            if self.active_phase in (1, 3):
                self.score += P1_BASE_VISIT_PTS
                self.get_logger().info(
                    f"[Score] Base {bid} visited (+{P1_BASE_VISIT_PTS}). "
                    f"Total: {self.score:.0f}")
            self.visited_bases.add(bid)
        else:
            # Repeated visit — penalise
            penalty = P1_REPEAT_PENALTY
            if self.active_phase == 3 and self.two_drones:
                penalty = P3_TWO_DRONE_REPEAT
            self.score += penalty
            self.get_logger().warn(
                f"[Score] Base {bid} REPEATED ({penalty}). "
                f"Total: {self.score:.0f}")

    def _cb_gesture(self, msg: HumanGesture):
        """Phase 3: buffer the latest gesture for the state machine."""
        self._last_gesture = msg.gesture_name

        if not self.human_detected and msg.confidence > 0.6:
            self.human_detected = True
            self.get_logger().info(
                f"[Phase3] Human operator DETECTED at "
                f"({msg.human_position.x:.2f}, {msg.human_position.y:.2f})")

    def _cb_qr(self, msg: QRCode):
        """Phase 4: track unique QR detections and award points."""
        if msg.qr_id not in self.qr_found:
            self.qr_found.add(msg.qr_id)
            self.score += P4_QR_PTS
            self.get_logger().info(
                f"[Phase4] QR '{msg.qr_id}' — +{P4_QR_PTS} pts. "
                f"Found: {sorted(self.qr_found)}. "
                f"Total: {self.score:.0f}")

    # ────────────────────────────────────────────────────────────────────────
    # Service handlers
    # ────────────────────────────────────────────────────────────────────────

    def _svc_start(self, request: SetPhase.Request, response):
        self.active_phase  = request.phase
        self.open_hardware = request.open_hardware
        self.two_drones    = request.use_two_drones
        self._reset_attempt()

        self.get_logger().info(
            f"=== STARTING PHASE {self.active_phase} "
            f"| open_hw={self.open_hardware} "
            f"| two_drones={self.two_drones} ===")

        self._do_takeoff()
        response.success = True
        response.message = f"Phase {self.active_phase} started"
        return response

    def _svc_abort(self, request, response):
        self.get_logger().warn("MISSION ABORTED by operator.")
        self._call_service(self.cli_land, Trigger.Request())
        self.state = "ABORTED"
        response.success = True
        response.message = "Mission aborted"
        return response

    # ────────────────────────────────────────────────────────────────────────
    # State machine
    # ────────────────────────────────────────────────────────────────────────

    def _state_machine_tick(self):
        """Called at 5 Hz to drive the mission forward."""

        # ── Timeout guard ────────────────────────────────────────────────────
        if self.state not in ("IDLE", "LANDED", "ABORTED") and self.attempt_start > 0:
            elapsed = time.time() - self.attempt_start
            limit   = ATTEMPT_TIME_LIMITS.get(self.active_phase, 600)
            if elapsed > limit:
                self.get_logger().warn(
                    f"[Mission] Attempt time limit reached ({limit}s). Aborting.")
                self._call_service(self.cli_land, Trigger.Request())
                self._finalise_score(autonomous_return=False)
                return

        # ── Phase-specific ticks ─────────────────────────────────────────────
        if   self.active_phase == 1: self._tick_phase1()
        elif self.active_phase == 2: self._tick_phase2()
        elif self.active_phase == 3: self._tick_phase3()
        elif self.active_phase == 4: self._tick_phase4()

    # ── Phase 1 ──────────────────────────────────────────────────────────────

    def _tick_phase1(self):
        if self.state == "IN_AIR":
            # Start exploration once airborne
            self._call_service(self.cli_explore, Trigger.Request())
            self.state = "VISITING"

        elif self.state == "ROUTE_COMPLETE":
            self._call_service(self.cli_home, Trigger.Request())
            self.state = "RETURNING_HOME"

        elif self.state == "HOME_REACHED":
            self._finalise_score(autonomous_return=True)

    # ── Phase 2 ──────────────────────────────────────────────────────────────

    def _tick_phase2(self):
        """
        Phase 2 is highly hardware-dependent (gripper/manipulator).
        The skeleton below provides the state transitions; the actual
        pick/drop logic must be integrated with your gripper driver.
        """
        if self.state == "IN_AIR":
            self.state = "GO_PICKUP"
            self.get_logger().info("[Phase2] Starting package transport.")

        elif self.state == "KIT_LIFTED":
            self.score += P2_PICKUP_PTS
            self.get_logger().info(
                f"[Phase2] Kit lifted (+{P2_PICKUP_PTS}). "
                f"Score: {self.score:.0f}")
            self.state = "GO_DROPOFF"

        elif self.state == "KIT_DELIVERED":
            self.score += P2_DELIVERY_PTS
            self.kits_delivered += 1
            self.get_logger().info(
                f"[Phase2] Kit delivered (+{P2_DELIVERY_PTS}). "
                f"Delivered: {self.kits_delivered}/3. "
                f"Score: {self.score:.0f}")
            if self.kits_delivered >= 3:
                self._call_service(self.cli_home, Trigger.Request())
                self.state = "RETURNING_HOME"
            else:
                self.state = "GO_PICKUP"

        elif self.state == "HOME_REACHED":
            self._finalise_score(autonomous_return=True)

    # ── Phase 3 ──────────────────────────────────────────────────────────────

    def _tick_phase3(self):
        if self.state == "IN_AIR":
            self.state = "APPROACH_HUMAN"
            self.get_logger().info("[Phase3] Approaching human operator.")

        elif self.state == "APPROACH_HUMAN":
            if self.human_detected:
                self.state = "WAIT_GESTURE"
                self.get_logger().info("[Phase3] Human confirmed — waiting for gesture.")

        elif self.state == "WAIT_GESTURE":
            gesture = self._last_gesture
            if gesture and gesture != "unknown":
                action = GESTURE_ACTION.get(gesture, "UNKNOWN")
                # Print required by Phase 3 rules
                self.get_logger().info(
                    f"[Phase3] Gesture command received: {gesture} → {action}")
                self._execute_gesture_action(action)
                self._last_gesture = ""

        elif self.state == "ALL_BASES_VISITED":
            self._call_service(self.cli_home, Trigger.Request())
            self.state = "RETURNING_HOME"

        elif self.state == "HOME_REACHED":
            self._finalise_score(autonomous_return=True)

    def _execute_gesture_action(self, action: str):
        """Translate a gesture action string into a nav command."""
        # TODO: replace with actual nav service calls per gesture
        self.get_logger().info(f"[Phase3] Executing: {action}")
        if action == "LAND_HERE":
            self.state = "LANDING_ON_BASE"
            self._call_service(self.cli_land, Trigger.Request())
        elif action == "STOP":
            pass  # hold position — no new setpoint needed

    # ── Phase 4 ──────────────────────────────────────────────────────────────

    def _tick_phase4(self):
        if self.state == "IN_AIR":
            self._call_service(self.cli_maze, Trigger.Request())
            self.state = "IN_MAZE"
            self.get_logger().info("[Phase4] Entering confined space.")

        elif self.state == "MAZE_EXITED":
            if not self.maze_completed:
                self.maze_completed = True
                self.score += P4_MAZE_PTS
                self.get_logger().info(
                    f"[Phase4] Maze traversed! +{P4_MAZE_PTS} pts. "
                    f"QR found: {sorted(self.qr_found)}. "
                    f"Total: {self.score:.0f}")
            self._finalise_score(autonomous_return=True)

    # ────────────────────────────────────────────────────────────────────────
    # Score finalisation
    # ────────────────────────────────────────────────────────────────────────

    def _finalise_score(self, autonomous_return: bool):
        """
        Apply multipliers and print the final attempt score.
        Called once per attempt when the drone lands or time runs out.
        """
        elapsed = time.time() - self.attempt_start

        if autonomous_return and self.score > 0:
            mult_label = "autonomous_return×2"
            self.score *= 2.0
            self.get_logger().info(f"[Score] {mult_label} applied.")

        if self.open_hardware and self.score > 0:
            self.score *= OPEN_HW_MULT
            self.get_logger().info("[Score] open_hardware×2 applied.")

        self.score = max(0.0, self.score)  # floor at 0

        self.get_logger().info(
            f"\n{'='*50}\n"
            f"  PHASE {self.active_phase} ATTEMPT COMPLETE\n"
            f"  Score         : {self.score:.0f}\n"
            f"  Elapsed       : {elapsed:.1f}s\n"
            f"  Bases visited : {len(self.visited_bases)}\n"
            f"  QR found      : {sorted(self.qr_found)}\n"
            f"  Kits delivered: {self.kits_delivered}\n"
            f"{'='*50}")

        self.state = "LANDED"

    # ────────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────────

    def _do_takeoff(self):
        req       = SetBool.Request()
        req.data  = True
        self._call_service(self.cli_takeoff, req)
        self.state         = "TAKING_OFF"
        self.attempt_start = time.time()
        self.get_logger().info("[Mission] Takeoff commanded.")

        # After a short delay, mark as in-air
        # (In production, wait for altitude confirmation via pose feedback)
        self.create_timer(3.0, self._mark_in_air)

    def _mark_in_air(self):
        if self.state == "TAKING_OFF":
            self.state = "IN_AIR"
            self.get_logger().info("[Mission] Drone in air — starting phase logic.")

    def _reset_attempt(self):
        self.score          = 0.0
        self.state          = "IDLE"
        self.attempt_start  = 0.0
        self.visited_bases  = set()
        self.kits_picked    = 0
        self.kits_delivered = 0
        self.qr_found       = set()
        self.maze_completed = False
        self.human_detected = False
        self._last_gesture  = ""

    def _call_service(self, client, request):
        """Fire-and-forget async service call with availability check."""
        if not client.wait_for_service(timeout_sec=self.svc_wait_tmo):
            self.get_logger().warn(
                f"Service {client.srv_name} not available — skipping.")
            return
        client.call_async(request)

    def _publish_state(self):
        msg              = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.phase        = self.active_phase
        msg.state_name   = self.state
        msg.score        = self.score
        msg.open_hardware = self.open_hardware
        self.pub_state.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = HydroneMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
