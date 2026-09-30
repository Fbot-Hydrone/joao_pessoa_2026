"""
hydrone_bringup/launch/hydrone_bringup.launch.py

TOP-LEVEL entry point. One flag selects the SOURCES layer; the autonomy stack is
always the same.

  ros2 launch hydrone_bringup hydrone_bringup.launch.py                 # sim (default)
  ros2 launch hydrone_bringup hydrone_bringup.launch.py use_sim:=false  # real drone

Architecture:
  use_sim:=true  -> sources_sim.launch.py   (BiguaSim + SITL + zed_mimic + ...)
  use_sim:=false -> sources_real.launch.py  (zed_wrapper + FCU MAVROS; stub for now)
  ALWAYS         -> hydrone.launch.py        (vision/controller/nav/mission)

Both sources layers publish the SAME contract buses (/zed/zed_node/* and
/mavros/*). The autonomy stack consumes only those, so it is byte-for-byte
identical for sim and real — hydrone.launch.py has NO sim/real logic and is not
touched by this file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("hydrone_bringup"), "launch")

    use_sim        = LaunchConfiguration("use_sim")
    phase          = LaunchConfiguration("phase")
    open_hardware  = LaunchConfiguration("open_hardware")
    use_two_drones = LaunchConfiguration("use_two_drones")

    args = [
        DeclareLaunchArgument("use_sim", default_value="true",
                              description="true=simulation sources, false=real drone sources"),
        DeclareLaunchArgument("phase",          default_value="1"),
        DeclareLaunchArgument("open_hardware",  default_value="false"),
        DeclareLaunchArgument("use_two_drones", default_value="false"),
        # Sim-only VO drift logger; see sources_sim.launch.py. Re-declared here
        # so it is settable from this file's command line, and ignored outright
        # when use_sim:=false (there is no ground truth on the real drone).
        DeclareLaunchArgument("odom_source",      default_value="vo"),
        DeclareLaunchArgument("odom_error",       default_value="true"),
        DeclareLaunchArgument("odom_error_print", default_value="false"),
        DeclareLaunchArgument("odom_error_dir",   default_value=""),
    ]

    # ── SOURCES layer (exactly one, chosen by use_sim) ──────────────────────
    sources_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "sources_sim.launch.py")),
        condition=IfCondition(use_sim),
        launch_arguments={
            "odom_source": LaunchConfiguration("odom_source"),
            "odom_error": LaunchConfiguration("odom_error"),
            "odom_error_print": LaunchConfiguration("odom_error_print"),
            "odom_error_dir": LaunchConfiguration("odom_error_dir"),
        }.items(),
    )
    sources_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "sources_real.launch.py")),
        condition=UnlessCondition(use_sim),
    )

    # ── Autonomy stack (ALWAYS, unchanged, agnostic) ────────────────────────
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "hydrone.launch.py")),
        launch_arguments={
            "phase": phase,
            "open_hardware": open_hardware,
            "use_two_drones": use_two_drones,
        }.items(),
    )

    return LaunchDescription(args + [sources_sim, sources_real, autonomy])
