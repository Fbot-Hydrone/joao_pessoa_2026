"""
hydrone_bringup/launch/hydrone_sim.launch.py

BACK-COMPAT ALIAS. The sim sources moved to sources_sim.launch.py (see the
sim/real split in hydrone_bringup.launch.py). This thin wrapper just includes it
so existing entry points keep working unchanged:
  - docker-compose:            ros2 launch hydrone_bringup hydrone_sim.launch.py
  - hydrone_mission_sim.launch.py includes this file.

Prefer the top-level for new use:
  ros2 launch hydrone_bringup hydrone_bringup.launch.py            # sim + autonomy
  ros2 launch hydrone_bringup sources_sim.launch.py                # sim sources only
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("hydrone_bringup"), "launch")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "sources_sim.launch.py"))),
    ])
