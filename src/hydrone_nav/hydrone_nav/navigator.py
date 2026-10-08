"""navigator — plan one leg around the occupancy map. ROS-light.

Used in-process by a mission (phase1_mission_node) and by nav_node, which
offers the same thing to any phase as the NavigateTo action.

Three outcomes, and the fall-back is deliberate (ported from the mission):

* the straight line is clear (the usual case) -> one waypoint, no extra
  decelerations
* it is not, and A* finds a way round -> the simplified waypoints
* there is no map yet, or it is too sparse -> the straight line, with a
  warning. Refusing because the map is thin would ground the vehicle at
  takeoff, when the map is always thin.

A leg that runs INTO something with no way round is REFUSED (`blocked`), never
flown "relying on the supervisor": that produced the drone hitting a wall.
"""

import math
from dataclasses import dataclass, field

from hydrone_map import octree
from hydrone_nav import planner


@dataclass
class Leg:
    waypoints: list = field(default_factory=list)   # [(x, y, z, yaw), ...]
    path: list = field(default_factory=list)        # [(x, y, z), ...] for RViz
    blocked: bool = False
    note: str = ""


class Navigator:
    def __init__(self, *, bounds=None, allow_unknown=True, log=None):
        self.bounds = bounds
        self.allow_unknown = allow_unknown
        self._log = log
        self._msg = None
        self.tree = None

    def set_map(self, octomap_msg):
        """Keep the newest tree as BYTES; decode only when a leg is planned.

        Decoding at the map's 2 Hz paid a full deserialize ~660 times a flight
        and buried the log under octomap-python's 'Tree size mismatch'.
        """
        self._msg = octomap_msg

    def _decode(self):
        if self._msg is None:
            return None
        try:
            return octree.tree_from_msg(self._msg)
        except ValueError as exc:
            if self._log:
                self._log.warn(f"octomap: {exc}", throttle_duration_sec=20.0)
            return None

    def occupancy(self):
        """The map as an inflated-occupancy callable, or None."""
        if self.tree is None:
            return None
        return lambda p: octree.inflated_state(self.tree, p)

    def plan(self, here, target, yaw) -> Leg:
        """A leg from `here` (x, y, z) to `target` (x, y, z) at heading yaw."""
        self.tree = self._decode()
        occ = self.occupancy()
        straight = Leg(waypoints=[(*target, yaw)], path=[here, target])
        if occ is None:
            straight.note = "no occupancy map — flying the leg straight"
            return straight
        # path_hits_obstacle, not path_is_clear_inflated: in a half-explored
        # arena almost no leg is MEASURED empty, and the strict test would
        # report everything blocked. What triggers a detour is something
        # actually in the way.
        if not octree.path_hits_obstacle(self.tree, here, target):
            return straight

        path = planner.plan(occ, here, target,
                            resolution=self.tree.getResolution(),
                            bounds=self.bounds,
                            allow_unknown=self.allow_unknown)
        if path is None:
            x, y, z = target
            col = " ".join(
                f"{zz:+.1f}:{octree.query(self.tree, (x, y, zz))[:4]}"
                for zz in (z - 0.6, z - 0.3, z, z + 0.3, z + 0.6))
            return Leg(path=[here], blocked=True, note=(
                f"({x:.2f}, {y:.2f}, {z:.2f}) is blocked and no way round it "
                f"exists in the map [goal raw={octree.query(self.tree, target)}"
                f" inflated={occ(target)} | column {col}]"))
        path = planner.simplify(
            path, lambda a, b: not octree.path_hits_obstacle(self.tree, a, b))
        return Leg(waypoints=[(p[0], p[1], p[2], yaw) for p in path[1:]],
                   path=path,
                   note=f"planned {len(path)} waypoints around the obstruction")


def path_msg(points, yaw, frame_id, stamp):
    """nav_msgs/Path for RViz. Purely informational."""
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    path = Path()
    path.header.stamp = stamp
    path.header.frame_id = frame_id
    for p in points:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x, ps.pose.position.y = float(p[0]), float(p[1])
        ps.pose.position.z = float(p[2])
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(ps)
    return path
