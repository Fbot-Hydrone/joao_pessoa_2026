"""
hydrone_bringup/launch/phase1_zed_detect.launch.py

THE OLDER DIVISION OF LABOUR: the forward ZED is what finds a base and what
says where it is.

    ros2 launch hydrone_bringup phase1_zed_detect_sim.launch.py
    ./scripts/docker_up.sh --zed-detect --ground-truth

This was `phase1` until 2026-09-02. It is kept, and kept runnable, because it
is the version that has been watched end to end the most, and because the two
missions fail in opposite ways — which makes it the thing to fall back to and
the thing to compare against.

What it does differently
------------------------
  * The forward ZED DETECTS. It sees a base across the arena and places it with
    depth, and `pad_map` fuses those positions. The belly camera contributes no
    position at all: it runs with `project_position: False`, on its own topic
    the map never subscribes to, and answers one question — "is a base under
    me" — before the drone commits to landing.
  * The search is the U: three sides of a rectangle at cruise, flown twice.
    Three and not four because from the third leg a camera aimed ACROSS the
    arena already looks back over everything the fourth would cover.
  * `pad_map`'s movement gates stay where they were (0.15 m/s, 10 deg/s),
    because this search stops to look — it is rotate, settle, look — so holding
    still costs it nothing. The default mission is a lawnmower and those same
    gates would reject every detection it makes.

What is measured about it
-------------------------
On seed 10, the one arena it has been flown on: 4 of 6 bases, 4 landings, every
one on a real base with the resting height matching to the centimetre, and map
positions 2-16 cm from truth.

That last number is the interesting one, and it is BETTER than the default
mission's 45 cm. The trade is visible in one line:

    this mission     finds fewer bases, places them well
    the default      finds more bases, places them poorly and lands by
                     centring the belly camera on the way down

Nobody has run this mission across the seed sweep, so the comparison is one
arena against seven and should not be read as settled. `MISSION=--zed-detect
scripts/seed_sweep.sh 1 2 3 4 5 6` is what would settle it.

A PURE ARGUMENT FLIP
--------------------
Everything below the argument block of phase1.launch.py — the state machine,
the confirmation, the landing, the octomap, the return home, every threshold —
is shared. This file sets eight arguments and nothing else, so a change to the
mission reaches both and neither can quietly drift from the other.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("hydrone_bringup"), "launch")

    # Deliberately NO DeclareLaunchArgument here. Launch configurations are
    # inherited by an included description, so every mission argument reaches
    # phase1.launch.py untouched and its defaults stay in one file — while
    # `takeoff_alt:=4` on the command line still works. Re-declaring one here
    # would shadow it with this file's default instead, which is a trap
    # phase1_sim.launch.py documents having fallen into and measured.
    phase1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "phase1.launch.py")),
        launch_arguments={
            # The U, twice, instead of perimeter-then-lanes.
            "search_mode": "u",
            # The ZED is the detector and the position source.
            "forward_detector": "true",
            # ...so the belly camera goes back to being a yes/no. Its only
            # projection route here would be a cast onto a flat floor at
            # `ground_z`, which a raised base breaks by construction.
            "down_project_position": "false",
            "down_map_topic": "",
            "down_range_as_depth": "false",
            # And the map does not fuse it. pad_map weights by
            # confidence / range^2, so a confirmation hover — hundreds of
            # close-range frames — would outvote the ZED and rewrite the very
            # entry the drone was flown there on.
            "map_down_detections": "",
            # The gates this search was designed around: it stops before it
            # looks, so refusing detections taken while moving costs it
            # nothing and buys a map free of smeared projections.
            "max_map_speed": "0.15",
            "max_map_yaw_rate_deg": "10.0",
        }.items(),
    )

    return LaunchDescription([phase1])
