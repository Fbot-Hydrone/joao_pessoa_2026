#!/usr/bin/env python3
"""
hydrone_nav — Navigation node for the Hydrone competition stack.

Builds on hydrone_controller to provide higher-level navigation:
  - Maintains a map of detected landing bases
  - Plans optimised visit routes (TSP-like for Phase 1)
  - Executes precise landing approach using vision feedback
  - Handles maze traversal for Phase 4

Subscribed topics:
  /hydrone/vision/landing_bases   (hydrone_msgs/LandingBase)
  /hydrone/controller/drone_pose  (geometry_msgs/PoseStamped)
  /hydrone/mission_state          (hydrone_msgs/MissionState)
  /hydrone/vision/qr_codes        (hydrone_msgs/QRCode)          — Phase 4

Published topics:
  /hydrone/controller/cmd_pose    (geometry_msgs/PoseStamped)    — position setpoint
  /hydrone/controller/cmd_vel     (geometry_msgs/Twist)          — velocity for fine approach
  /hydrone/nav/status             (std_msgs/String)

Services exposed:
  /hydrone/nav/go_to_base         (std_srvs/SetBool)  — data ID passed via parameter
  /hydrone/nav/return_home        (std_srvs/Trigger)
  /hydrone/nav/start_exploration  (std_srvs/Trigger)  — Phase 1/2 full route
  /hydrone/nav/enter_maze         (std_srvs/Trigger)  — Phase 4
"""

import math
import time
from itertools import permutations
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist, Point
from std_msgs.msg      import String
from std_srvs.srv      import Trigger, SetBool

from hydrone_msgs.msg  import LandingBase, MissionState, QRCode


# ── Constants ────────────────────────────────────────────────────────────────

# Cruise altitude between waypoints (metres)
CRUISE_ALT = 1.5

# Approach altitude just above the base before descending
APPROACH_ALT_OFFSET = 0.4   # metres above base height

# Descent speed for precision landing (m/s)
DESCENT_VEL = 0.25

# How close (m) the drone must be to the XY centre of a base before descending
XY_TOLERANCE_APPROACH = 0.15

# Home / takeoff base position in the local frame
HOME_X = 0.0
HOME_Y = 0.0
HOME_Z = 0.0

# Maze entry and exit waypoints in the local frame (adjust to real arena layout)
# Entry: red-arrow side; Exit: blue-arrow side (see Phase 4 rules Fig. 10)
MAZE_ENTRY_WP  = (2.0, 0.0, 1.0)   # (x, y, z)
MAZE_SEARCH_WPS = [                 # internal sweep waypoints
    (2.5, 0.5, 0.9),
    (3.5, 0.5, 0.9),
    (4.5, 0.5, 0.9),
    (4.5, 1.5, 0.9),
    (3.5, 1.5, 0.9),
    (2.5, 1.5, 0.9),
]
MAZE_EXIT_WP   = (8.0, 1.0, 1.0)   # exit through blue-arrow window
MAZE_LANDING_WP = (4.0, 4.0, 0.0)  # landing base centre after exiting maze


class HydroneNavNode(Node):

    def __init__(self):
        super().__init__("hydrone_nav")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("cruise_altitude",    CRUISE_ALT)
        self.declare_parameter("xy_tolerance",       XY_TOLERANCE_APPROACH)
        self.declare_parameter("descent_vel",        DESCENT_VEL)

        self.cruise_alt  = self.get_parameter("cruise_altitude").value
        self.xy_tol      = self.get_parameter("xy_tolerance").value
        self.descent_vel = self.get_parameter("descent_vel").value

        # ── Internal state ──────────────────────────────────────────────────
        self.bases: Dict[int, LandingBase] = {}      # known bases
        self.current_pose = PoseStamped()
        self.phase        = 1
        self.qr_found: List[str] = []                # Phase 4

        # Route execution state
        self._route: List[int] = []                  # ordered base IDs
        self._route_idx: int   = 0
        self._nav_state: str   = "IDLE"              # IDLE | CRUISING | APPROACHING | LANDING
        self._active_base: Optional[LandingBase] = None

        # ── QoS ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ─────────────────────────────────────────────────────
        self.sub_bases   = self.create_subscription(
            LandingBase, "/hydrone/vision/landing_bases",
            self._cb_base, 10)
        self.sub_pose    = self.create_subscription(
            PoseStamped, "/hydrone/controller/drone_pose",
            self._cb_pose, sensor_qos)
        self.sub_mission = self.create_subscription(
            MissionState, "/hydrone/mission_state",
            self._cb_mission, 10)
        self.sub_qr      = self.create_subscription(
            QRCode, "/hydrone/vision/qr_codes",
            self._cb_qr, 10)

        # ── Publishers ──────────────────────────────────────────────────────
        self.pub_cmd_pose = self.create_publisher(
            PoseStamped, "/hydrone/controller/cmd_pose", 10)
        self.pub_cmd_vel  = self.create_publisher(
            Twist, "/hydrone/controller/cmd_vel", 10)
        self.pub_status   = self.create_publisher(
            String, "/hydrone/nav/status", 10)

        # ── Services ────────────────────────────────────────────────────────
        self.srv_explore  = self.create_service(
            Trigger, "/hydrone/nav/start_exploration", self._svc_start_exploration)
        self.srv_home     = self.create_service(
            Trigger, "/hydrone/nav/return_home",       self._svc_return_home)
        self.srv_maze     = self.create_service(
            Trigger, "/hydrone/nav/enter_maze",        self._svc_enter_maze)

        # Navigation loop timer
        self.create_timer(0.1, self._nav_loop)   # 10 Hz

        self.get_logger().info("HydroneNavNode ready.")

    # ────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ────────────────────────────────────────────────────────────────────────

    def _cb_base(self, msg: LandingBase):
        """Update the base map with the latest detection."""
        bid = msg.base_id
        # Keep the most-recently-seen version unless already visited
        if bid not in self.bases or not self.bases[bid].is_visited:
            self.bases[bid] = msg

    def _cb_pose(self, msg: PoseStamped):
        self.current_pose = msg

    def _cb_mission(self, msg: MissionState):
        self.phase = msg.phase

    def _cb_qr(self, msg: QRCode):
        if msg.qr_id not in self.qr_found:
            self.qr_found.append(msg.qr_id)
            self.get_logger().info(
                f"[Phase4] QR '{msg.qr_id}' confirmed. "
                f"Total: {len(self.qr_found)}/5")

    # ────────────────────────────────────────────────────────────────────────
    # Service handlers
    # ────────────────────────────────────────────────────────────────────────

    def _svc_start_exploration(self, request, response):
        """
        Build an optimised route visiting all known bases and start execution.
        For Phase 1: visit all 6 bases then return home.
        """
        if len(self.bases) == 0:
            response.success = False
            response.message = "No bases detected yet — run a scan first"
            return response

        self._route     = self._plan_route(list(self.bases.keys()))
        self._route_idx = 0
        self._nav_state = "CRUISING"
        self._advance_route()

        response.success = True
        response.message = f"Route planned: {self._route}"
        self.get_logger().info(response.message)
        return response

    def _svc_return_home(self, request, response):
        self._go_to_xyz(HOME_X, HOME_Y, self.cruise_alt)
        self._nav_state  = "RETURNING_HOME"
        response.success = True
        response.message = "Returning to home base"
        return response

    def _svc_enter_maze(self, request, response):
        """Phase 4: fly entry → sweep → exit → land."""
        self._maze_waypoints = (
            [MAZE_ENTRY_WP] + MAZE_SEARCH_WPS + [MAZE_EXIT_WP]
        )
        self._maze_wp_idx = 0
        self._nav_state   = "MAZE_TRAVERSAL"
        self._go_to_xyz(*self._maze_waypoints[0])
        response.success  = True
        response.message  = "Entering maze"
        self.get_logger().info("Phase 4: Entering maze.")
        return response

    # ────────────────────────────────────────────────────────────────────────
    # Navigation loop (10 Hz)
    # ────────────────────────────────────────────────────────────────────────

    def _nav_loop(self):
        pos = self.current_pose.pose.position
        status = f"state={self._nav_state} base={self._route_idx}/{len(self._route)}"
        self.pub_status.publish(String(data=status))

        if self._nav_state == "CRUISING":
            self._handle_cruising(pos)

        elif self._nav_state == "APPROACHING":
            self._handle_approaching(pos)

        elif self._nav_state == "LANDING":
            self._handle_landing(pos)

        elif self._nav_state == "RETURNING_HOME":
            self._handle_return_home(pos)

        elif self._nav_state == "MAZE_TRAVERSAL":
            self._handle_maze(pos)

    # ── State handlers ───────────────────────────────────────────────────────

    def _handle_cruising(self, pos: Point):
        """Check whether we are above the next base; if so, switch to APPROACHING."""
        if self._active_base is None:
            return
        base_x = self._active_base.pose.position.x
        base_y = self._active_base.pose.position.y
        dx = pos.x - base_x
        dy = pos.y - base_y
        if math.sqrt(dx*dx + dy*dy) < self.xy_tol * 3:
            # Close enough horizontally — start precision approach
            approach_z = self._active_base.height + APPROACH_ALT_OFFSET
            self._go_to_xyz(base_x, base_y, approach_z)
            self._nav_state = "APPROACHING"
            self.get_logger().info(
                f"Approaching base {self._active_base.base_id}")

    def _handle_approaching(self, pos: Point):
        """Refine XY position over the base then switch to LANDING."""
        if self._active_base is None:
            return
        base_x = self._active_base.pose.position.x
        base_y = self._active_base.pose.position.y
        dx = pos.x - base_x
        dy = pos.y - base_y
        dist_xy = math.sqrt(dx*dx + dy*dy)

        if dist_xy < self.xy_tol:
            # Centred — start descent
            self._nav_state = "LANDING"
            self.get_logger().info(
                f"Starting descent on base {self._active_base.base_id}")
        else:
            # Slow correction toward base centre
            vel         = Twist()
            vel.linear.x = -dx * 0.4
            vel.linear.y = -dy * 0.4
            vel.linear.z = 0.0
            self.pub_cmd_vel.publish(vel)

    def _handle_landing(self, pos: Point):
        """
        Descend at constant velocity until the expected landing height is reached,
        then mark the base as visited and advance the route.
        """
        if self._active_base is None:
            return
        target_z = self._active_base.height

        if pos.z > target_z + 0.05:
            vel         = Twist()
            vel.linear.z = -self.descent_vel
            self.pub_cmd_vel.publish(vel)
        else:
            # Landed — stop descent, mark visited, go to next waypoint
            vel         = Twist()
            self.pub_cmd_vel.publish(vel)
            self._active_base.is_visited = True
            self.bases[self._active_base.base_id].is_visited = True
            self.get_logger().info(
                f"Landed on base {self._active_base.base_id} ✓")
            time.sleep(0.5)  # brief pause on base
            self._route_idx += 1
            if self._route_idx < len(self._route):
                self._advance_route()
            else:
                self._svc_return_home(None, Trigger.Response())

    def _handle_return_home(self, pos: Point):
        """Check if we are back at home."""
        dx = pos.x - HOME_X
        dy = pos.y - HOME_Y
        dz = pos.z - HOME_Z
        if math.sqrt(dx*dx + dy*dy + dz*dz) < 0.25:
            self._nav_state = "IDLE"
            self.get_logger().info("Home reached — mission complete.")

    def _handle_maze(self, pos: Point):
        """Advance through maze waypoints."""
        if not hasattr(self, "_maze_waypoints"):
            return
        wp = self._maze_waypoints[self._maze_wp_idx]
        dx = pos.x - wp[0]
        dy = pos.y - wp[1]
        dz = pos.z - wp[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist < 0.3:
            self._maze_wp_idx += 1
            if self._maze_wp_idx < len(self._maze_waypoints):
                next_wp = self._maze_waypoints[self._maze_wp_idx]
                self._go_to_xyz(*next_wp)
                self.get_logger().info(
                    f"Maze WP {self._maze_wp_idx}/{len(self._maze_waypoints)}")
            else:
                # Exited maze — land on centre base
                self._go_to_xyz(*MAZE_LANDING_WP)
                self._nav_state = "IDLE"
                self.get_logger().info("Exited maze — landing on arena base.")

    # ────────────────────────────────────────────────────────────────────────
    # Route planning (nearest-neighbour TSP heuristic)
    # ────────────────────────────────────────────────────────────────────────

    def _plan_route(self, base_ids: List[int]) -> List[int]:
        """
        Nearest-neighbour greedy TSP starting from the drone's current position.
        Good enough for 6 nodes; full exhaustive search would be ≤ 5! = 120.
        """
        if len(base_ids) <= 6:
            return self._exhaustive_tsp(base_ids)
        return self._nn_tsp(base_ids)

    def _exhaustive_tsp(self, base_ids: List[int]) -> List[int]:
        """Brute-force optimal route for ≤ 6 bases."""
        start = (self.current_pose.pose.position.x,
                 self.current_pose.pose.position.y)
        best_dist  = float("inf")
        best_route = base_ids[:]

        for perm in permutations(base_ids):
            dist = 0.0
            prev = start
            for bid in perm:
                b    = self.bases[bid]
                bx   = b.pose.position.x
                by   = b.pose.position.y
                dist += math.sqrt((bx - prev[0])**2 + (by - prev[1])**2)
                prev  = (bx, by)
            if dist < best_dist:
                best_dist  = dist
                best_route = list(perm)

        self.get_logger().info(
            f"Optimal route (d={best_dist:.1f}m): {best_route}")
        return best_route

    def _nn_tsp(self, base_ids: List[int]) -> List[int]:
        """Nearest-neighbour heuristic for larger sets."""
        unvisited = base_ids[:]
        route     = []
        cx = self.current_pose.pose.position.x
        cy = self.current_pose.pose.position.y

        while unvisited:
            nearest = min(
                unvisited,
                key=lambda bid: self._dist_to_base(cx, cy, bid)
            )
            b  = self.bases[nearest]
            cx = b.pose.position.x
            cy = b.pose.position.y
            route.append(nearest)
            unvisited.remove(nearest)

        return route

    def _dist_to_base(self, x: float, y: float, base_id: int) -> float:
        b  = self.bases[base_id]
        dx = b.pose.position.x - x
        dy = b.pose.position.y - y
        return math.sqrt(dx*dx + dy*dy)

    # ────────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────────

    def _advance_route(self):
        """Point at the next base in the route and start cruising toward it."""
        if self._route_idx >= len(self._route):
            return
        bid = self._route[self._route_idx]
        self._active_base = self.bases.get(bid)
        if self._active_base is None:
            self.get_logger().warn(f"Base {bid} not in map yet, skipping.")
            self._route_idx += 1
            self._advance_route()
            return

        cruise_z = max(self.cruise_alt,
                       self._active_base.height + APPROACH_ALT_OFFSET + 0.3)
        self._go_to_xyz(
            self._active_base.pose.position.x,
            self._active_base.pose.position.y,
            cruise_z,
        )
        self._nav_state = "CRUISING"
        self.get_logger().info(
            f"Cruising to base {bid} @ "
            f"({self._active_base.pose.position.x:.2f}, "
            f"{self._active_base.pose.position.y:.2f}, "
            f"{cruise_z:.2f})")

    def _go_to_xyz(self, x: float, y: float, z: float, yaw: float = 0.0):
        sp                    = PoseStamped()
        sp.header.frame_id    = "map"
        sp.header.stamp       = self.get_clock().now().to_msg()
        sp.pose.position.x    = x
        sp.pose.position.y    = y
        sp.pose.position.z    = z
        sp.pose.orientation.z = math.sin(yaw / 2.0)
        sp.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_cmd_pose.publish(sp)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = HydroneNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
