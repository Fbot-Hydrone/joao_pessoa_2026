#!/usr/bin/env python3
"""
hydrone_nav — navigation as a service to any phase.

  /hydrone/nav/navigate_to   hydrone_msgs/action/NavigateTo
      plan around /octomap/octomap_binary (hydrone_nav.navigator), then fly
      each waypoint through the controller's GoTo action
  /hydrone/nav/plan          nav_msgs/Path   the route, for RViz

It never talks to MAVROS: it is a client of hydrone_controller like everyone
else. The libraries it is built from (navigator, planner, coverage, route,
precision_landing, servo) can also be imported directly by a mission node.
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from octomap_msgs.msg import Octomap

from hydrone_msgs.action import NavigateTo

from hydrone_controller.vehicle import Vehicle
from hydrone_nav.navigator import Navigator, path_msg

POLL_S = 0.05


class NavNode(Node):

    def __init__(self, **kwargs):
        super().__init__("hydrone_nav", **kwargs)
        b = [float(v) for v in self.declare_parameter(
            "plan_bounds", [-5.0, -5.0, 0.3, 5.0, 5.0, 2.5]).value]
        self.nav = Navigator(
            bounds=(tuple(b[:3]), tuple(b[3:])),
            allow_unknown=self.declare_parameter(
                "plan_allow_unknown", True).value,
            log=self.get_logger())
        self.default_tol = self.declare_parameter("tolerance_m", 0.25).value

        cbg = ReentrantCallbackGroup()
        self.pose = None
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 lambda m: setattr(self, "pose", m),
                                 qos_profile_sensor_data, callback_group=cbg)
        # Latched: octomap_server publishes TRANSIENT_LOCAL; a volatile
        # subscription would match nothing and every leg would fly unchecked.
        self.create_subscription(
            Octomap, "/octomap/octomap_binary", self.nav.set_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST), callback_group=cbg)
        self.pub_plan = self.create_publisher(Path, "/hydrone/nav/plan", 10)
        self.vehicle = Vehicle(self)
        ActionServer(self, NavigateTo, "/hydrone/nav/navigate_to",
                     self._exec, callback_group=cbg,
                     goal_callback=lambda g: GoalResponse.ACCEPT,
                     cancel_callback=lambda g: CancelResponse.ACCEPT)
        self.get_logger().info("hydrone_nav ready — /hydrone/nav/navigate_to")

    def _exec(self, gh):
        g = gh.request
        res = NavigateTo.Result()
        if self.pose is None:
            res.message = "no pose yet"
            gh.abort()
            return res
        p = self.pose.pose.position
        leg = self.nav.plan((p.x, p.y, p.z),
                            (g.position.x, g.position.y, g.position.z), g.yaw)
        self.pub_plan.publish(path_msg(leg.path, g.yaw, "map",
                                       self.get_clock().now().to_msg()))
        if leg.blocked:
            self.get_logger().error(f"{leg.note} — REFUSING the leg.")
            res.blocked, res.message = True, leg.note
            gh.abort()
            return res
        if leg.note:
            self.get_logger().info(leg.note)

        tol = g.tolerance or self.default_tol
        n = len(leg.waypoints)
        for i, (x, y, z, yaw) in enumerate(leg.waypoints):
            gh.publish_feedback(NavigateTo.Feedback(waypoint=i, waypoints=n))
            job = self.vehicle.go_to(x, y, z, yaw, tolerance=tol)
            while not job.done:
                if gh.is_cancel_requested:
                    job.cancel()
                    gh.canceled()
                    res.message = "cancelled"
                    return res
                time.sleep(POLL_S)
            if not job.ok:
                res.message = f"waypoint {i + 1}/{n}: {job.message}"
                gh.abort()
                return res
        res.success, res.message = True, f"arrived ({n} waypoint(s))"
        gh.succeed()
        return res


def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
