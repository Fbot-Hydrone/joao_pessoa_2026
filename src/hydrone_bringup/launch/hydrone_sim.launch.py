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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("hydrone_bringup"), "launch")

    # Re-declared and forwarded rather than inherited: an argument declared only
    # in the included file is not settable from THIS file's command line, so
    # `hydrone_sim.launch.py odom_error_print:=true` would otherwise be silently
    # ignored. Defaults must match sources_sim.launch.py.
    args = [
        DeclareLaunchArgument(
            "odom_source", default_value="vo",
            description="EKF odometry source: ground_truth | vo. "
                        "See sources_sim.launch.py."),
        DeclareLaunchArgument(
            "odom_error", default_value="true",
            description="Run odom_error_node (VO drift vs ground truth -> CSV)."),
        DeclareLaunchArgument(
            "odom_error_print", default_value="false",
            description="Echo the VO drift to stdout at 1 Hz as well as the CSV."),
        DeclareLaunchArgument(
            "odom_error_dir", default_value="/ws/logs",
            description="Directory for the drift CSV. Empty = repo root."),
    ]

    sources_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "sources_sim.launch.py")),
        launch_arguments={
            "odom_source": LaunchConfiguration("odom_source"),
            "odom_error": LaunchConfiguration("odom_error"),
            "odom_error_print": LaunchConfiguration("odom_error_print"),
            "odom_error_dir": LaunchConfiguration("odom_error_dir"),
        }.items(),
    )

    return LaunchDescription(args + [sources_sim])
