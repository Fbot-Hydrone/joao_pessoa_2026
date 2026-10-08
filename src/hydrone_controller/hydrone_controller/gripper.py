"""gripper — what a mission uses to grab and release a payload.

Same pattern as vehicle.py: every call returns a Job polled from the tick.

    from hydrone_controller.gripper import Gripper

    self.gripper = Gripper(self)
    job = self.gripper.close()
    ...
    if job.done and job.ok: holding = job.result.holding

Whoever serves /hydrone/gripper/command does the work: ardubridge_node in the
simulator, gripper_dynamixel_node (AX-12A) on the drone. The mission never
knows which.
"""

from rclpy.action import ActionClient

from hydrone_msgs.action import Gripper as GripperAction

from hydrone_controller.vehicle import Job


class Gripper:
    def __init__(self, node, action="/hydrone/gripper/command"):
        self._client = ActionClient(node, GripperAction, action)

    def ready(self) -> bool:
        return self._client.server_is_ready()

    def close(self) -> Job:
        return Job(self._client, GripperAction.Goal(close=True))

    def open(self) -> Job:
        return Job(self._client, GripperAction.Goal(close=False))
