"""vehicle — what a mission uses to fly. No MAVROS, no MAVLink.

Nectar-style facade over the controller node's actions:

    from hydrone_controller.vehicle import Vehicle

    self.vehicle = Vehicle(self)              # inside any rclpy Node
    job = self.vehicle.takeoff(2.5)           # returns at once
    ...
    if job.done and job.ok:                   # poll from your own timer
        ...

Every command returns a Job that is polled, never awaited, because missions
here are tick-driven state machines: blocking a tick would stop everything
else the node does. `Vehicle.setpoint()` streams a target with no arrival
wait (holds, nudges, visual servoing).
"""

import math

from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped

from hydrone_msgs.action import Arm, GoTo, Land, Takeoff

NS = "/hydrone/controller"


class Job:
    """One action goal in flight. Poll `.done`; read `.ok` and `.result`."""

    def __init__(self, client, goal, on_feedback=None):
        self.result = None
        self.feedback = None
        self._goal_handle = None
        self._rejected = False
        self._user_fb = on_feedback
        if not client.server_is_ready():
            self._rejected = True
            self.error = f"{client._action_name} not available"
            return
        self.error = ""
        fut = client.send_goal_async(goal, feedback_callback=self._on_fb)
        fut.add_done_callback(self._on_accept)

    def _on_fb(self, msg):
        self.feedback = msg.feedback
        if self._user_fb:
            self._user_fb(msg.feedback)

    def _on_accept(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self._rejected = True
            self.error = "goal rejected"
            return
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(
            lambda f: setattr(self, "result", f.result().result))

    @property
    def done(self) -> bool:
        return self._rejected or self.result is not None

    @property
    def ok(self) -> bool:
        return self.result is not None and bool(self.result.success)

    @property
    def message(self) -> str:
        return self.error or (self.result.message if self.result else "")

    def cancel(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()


class Vehicle:
    def __init__(self, node, namespace: str = NS):
        self.node = node
        self._arm = ActionClient(node, Arm, f"{namespace}/arm")
        self._takeoff = ActionClient(node, Takeoff, f"{namespace}/takeoff")
        self._goto = ActionClient(node, GoTo, f"{namespace}/go_to")
        self._land = ActionClient(node, Land, f"{namespace}/land")
        self._pub = node.create_publisher(PoseStamped,
                                          f"{namespace}/cmd_pose", 10)

    def ready(self) -> bool:
        return all(c.server_is_ready() for c in
                   (self._arm, self._takeoff, self._goto, self._land))

    def arm(self, timeout_s: float = 0.0, on_feedback=None) -> Job:
        return Job(self._arm, Arm.Goal(timeout_s=float(timeout_s)),
                   on_feedback)

    def takeoff(self, altitude: float, hold_z: float = float("nan"),
                on_feedback=None) -> Job:
        return Job(self._takeoff,
                   Takeoff.Goal(altitude=float(altitude),
                                hold_z=float(hold_z)), on_feedback)

    def go_to(self, x, y, z, yaw=0.0, tolerance=0.0, timeout_s=0.0,
              on_feedback=None) -> Job:
        g = GoTo.Goal(yaw=float(yaw), tolerance=float(tolerance),
                      timeout_s=float(timeout_s))
        g.position.x, g.position.y, g.position.z = float(x), float(y), float(z)
        return Job(self._goto, g, on_feedback)

    def land(self, disarm: bool = True, on_feedback=None) -> Job:
        return Job(self._land, Land.Goal(disarm=bool(disarm)), on_feedback)

    def setpoint(self, x, y, z, yaw=0.0, frame_id="map"):
        """Stream a target. No arrival wait; resend as often as you like."""
        sp = PoseStamped()
        sp.header.stamp = self.node.get_clock().now().to_msg()
        sp.header.frame_id = frame_id
        sp.pose.position.x, sp.pose.position.y = float(x), float(y)
        sp.pose.position.z = float(z)
        sp.pose.orientation.z = math.sin(yaw / 2.0)
        sp.pose.orientation.w = math.cos(yaw / 2.0)
        self._pub.publish(sp)
