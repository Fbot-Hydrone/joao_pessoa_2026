"""
hydrone_bringup/launch/phase1_map_sweep.launch.py

AN EXPERIMENT. A different division of labour between the two cameras.

    ros2 launch hydrone_bringup phase1_map_sweep_sim.launch.py

What phase1 does today
----------------------
The forward ZED both FINDS a base and says WHERE it is, from across the arena,
and the search shapes (the U, twice) exist so it can see as much floor as
possible from as few positions as possible. The belly camera votes yes/no over
the candidate before the drone commits to landing. That mission has landed on
four of six bases in a measured run, and it stays exactly as it was — this file
sets arguments, it does not fork phase1.launch.py.

What this does instead
----------------------
The ZED stops detecting. It flies the odometry and it fills the occupancy map,
and nothing else.

    1  CLOSED PERIMETER at cruise. Four sides, back where it started. Its
       product is the map, not detections. The U skips its fourth side on the
       argument that the third already looks back across everything the fourth
       would cover — true for a camera aimed ACROSS the arena, false for one
       building an occupancy map out of a depth camera's own band, where the
       fourth side carries the only view of the strip beside it.

    2  LANES, spaced by the belly camera's FOOTPRINT. Not by a number: the
       lane pitch is computed at run time from the live CameraInfo and the
       height above the floor. This is the pass that finds bases.

And the belly camera becomes the detector AND the position source, because it
now has a way to answer "where" that does not assume anything:

    the pixel is a RAY. Cast it into the occupancy map. The first occupied
    voxel is the surface that pixel is looking at — the TOP of a raised base
    if that is what is under it, the floor if it is not.

That is the whole idea. Every previous route had to guess the surface: a plane
at `ground_z` is wrong for a base raised 0 to 1.5 m (MEASURED 2026-09-01, a
base 1.29 m tall seen from 7.7 m placed 1.06 m out), and the rangefinder
measures what is under the VEHICLE rather than under the pixel. The map already
holds the tops of the bases, swept in by the depth camera on the perimeter pass
— which is why the perimeter has to come first.

The rangefinder is the fallback, for the case the map cannot answer: a cell no
ray ever crossed, or a ray that leaves the tree. It measures the vehicle's own
nadir, so it is right while the pad is near the centre of the frame, which
during a lawnmower lane is where it will be.

WHAT IS NOT KNOWN ABOUT THIS
----------------------------
Compared with the U mission it trades the ZED's reach for the belly camera's
accuracy, and the trade has a clear failure mode: the lanes pass over every
part of the arena ONCE, so a base the belly camera misses on its single pass is
a base this mission never sees at all, where the U gets two looks from
different angles. Whether the map-cast position is worth that is the question a
run answers, and no run has answered it yet.

Two things the first bring-up did settle:

  * the wiring is right — ONE detector comes up, the belly one, logging
    `map=/octomap/octomap_binary`, and the takeoff base registers at (-3.36,
    3.00) against a true (-3.40, 3.00).
  * `max_map_speed` had to be raised or the mode cannot map at all. See below.

And one that is NOT fixed, because it is bigger than this file: the mission's
own planner queries the same octree in the same wrong frame. octomap_server
publishes in `odom`, the mission asks in `map`, and nothing transforms — see
_map_hit in pad_detector_node.py for what that costs and how it is done. It
would read as unknown almost everywhere, which is consistent with the planner
having stopped refusing anything once `plan_allow_unknown` was turned on.

Everything else — the state machine, the confirmation, the landing, the height
check, the return home — is phase1's, unchanged.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Where this launch's own octomap_server publishes the tree. Latched and ~3 KB,
# which is also what fits over a radio to the real drone.
OCTOMAP_TOPIC = "/octomap/octomap_binary"

# The belly camera's private detection topic, as phase1.launch.py names it.
# It is deliberately the SAME topic for both consumers: pad_map fuses the
# positions on it, the mission counts looks on it.
DOWN_DETECTIONS = "/hydrone/pads/down/detections"


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("hydrone_bringup"), "launch")

    # Only what this file OWNS. Every other mission argument reaches
    # phase1.launch.py by inheritance — re-declaring one here would shadow it
    # with this file's default and silently break command-line tuning, which is
    # a trap phase1_sim.launch.py documents having fallen into.
    args = [
        DeclareLaunchArgument(
            "sweep_overlap", default_value="0.25",
            description="Fraction of each belly swath the next lane repeats. "
                        "Two adjacent lanes are flown minutes apart on an "
                        "estimate that drifts, and the gap between them is "
                        "that drift — spacing lanes edge to edge opens holes, "
                        "and a hole in a single-pass sweep is a base that is "
                        "never seen."),
        DeclareLaunchArgument(
            "octomap", default_value="true",
            description="Must stay true here. The whole mission projects into "
                        "this map; without it the belly camera falls back to "
                        "the rangefinder for every detection and the "
                        "experiment is not being run."),
    ]

    phase1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "phase1.launch.py")),
        launch_arguments={
            "search_mode": "map_sweep",
            # The ZED flies the odometry and fills the map. It reports no pads.
            "forward_detector": "false",
            # The belly camera becomes the detector AND the position source.
            "down_project_position": "true",
            "down_map_topic": OCTOMAP_TOPIC,
            "down_range_as_depth": "true",
            # pad_map fuses what the belly reports. Same topic the mission
            # confirms on; they read different things off it.
            "map_down_detections": DOWN_DETECTIONS,
            # THE GATE THAT BLOCKS THIS MODE UNLESS IT IS RAISED. pad_map
            # refuses a detection while the vehicle is moving faster than
            # this, and its own comment says why that was free: "the search is
            # already rotate, settle, look". A lawnmower is not — it is
            # continuous translation, and the belly camera only ever sees a
            # base while passing OVER it. MEASURED on the first run of this
            # mode, every single belly detection was thrown out:
            #
            #   detection from down at (-3.15, 2.71, 0.65) conf 0.94 range 2.0 m
            #   REJECTED: vehicle is translating at 0.56 m/s > max_map_speed 0.15
            #
            # 2.0 clears the 1.26 m/s peak seen on that run. What the gate
            # protects against is a pose that is stale by the estimator's lag,
            # and the error that produces is lag x speed and grows with RANGE
            # as the ray gets shallow — which is the forward camera's 8 m
            # problem, not a belly camera's 2 m near-vertical one. That
            # argument is why this is worth trying; the landing accuracy is
            # what settles whether it is true.
            "max_map_speed": "2.0",
            # AND THE YAW GATE, which measurement showed is worse than merely
            # unhelpful here — it is anti-correlated with accuracy. On the
            # first full run of this mode, every belly detection scored
            # against the true base positions:
            #
            #     accepted           n=5   median xy error  0.53 m
            #     rejected for yaw   n=5   median xy error  0.06 m
            #
            # The reason is geometry. The vehicle turns 180 deg at the end of
            # each lane, and it is DURING that turn that a base sits under the
            # nadir — a short, near-vertical ray, which is the best case for
            # casting into the map. Mid-lane the same base is a glimpse at the
            # edge of the frame on a long shallow ray. The gate was written
            # for a search that stops to look, where slewing meant a smeared
            # scene; here it discards the only accurate readings there are.
            "max_map_yaw_rate_deg": "60.0",
            "octomap": LaunchConfiguration("octomap"),
            "sweep_overlap": LaunchConfiguration("sweep_overlap"),
        }.items(),
    )

    return LaunchDescription(args + [phase1])
