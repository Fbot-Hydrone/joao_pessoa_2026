"""
hydrone_bringup/launch/phase3_sim.launch.py

ONE COMMAND for Phase 3 in BiguaSim:

    ros2 launch hydrone_bringup phase3_sim.launch.py        # or: docker_up.sh --phase3

  = phase4_sim.launch.py  (Kopis X8 + Mid-360 + FAST-LIO + MAVROS, no mission)
  + phase3.launch.py      (phase3_hri_node)

Phase 3 flies the same small aircraft as phase 4 (the rules cap it at 330 mm,
and the bridge already treats phases 3 and 4 alike), so the sources are phase
4's untouched, started with phase:=3 and no maze mission. The EKF flies on the
LIO; nothing here reads ground truth.

Then, in a SECOND terminal, be the camera:

    ros2 run hydrone_mission gesture_terminal
    # docker: scripts/phase3_terminal.sh

Mission arguments (takeoff_alt, step_m, target_bases, ...) are NOT declared
here, on purpose: they reach phase3.launch.py by inheritance, so its defaults
are the only ones (see phase1_sim.launch.py for why mirroring them broke
tuning). `ros2 launch -s` on phase3.launch.py lists them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("hydrone_bringup"), "launch")

    # Nothing declared here: phase4_sim's arguments (agent_name, ext_nav,
    # measure_drift, ...) and phase3.launch.py's reach them by inheritance, so
    # each keeps the one default it declares. Only what makes this phase 3 is
    # forwarded.
    sources = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "phase4_sim.launch.py")),
        launch_arguments={
            "phase": "3",
            "mission": "none",     # the maze node would fight for the vehicle
            "map_name": "phase3",
        }.items(),
    )

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "phase3.launch.py")),
    )

    return LaunchDescription([sources, autonomy])
