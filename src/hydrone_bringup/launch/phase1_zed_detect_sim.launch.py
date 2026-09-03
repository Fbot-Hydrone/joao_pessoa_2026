"""
hydrone_bringup/launch/phase1_zed_detect_sim.launch.py

ONE COMMAND to fly the ZED-detects mission in simulation:

    ros2 launch hydrone_bringup phase1_zed_detect_sim.launch.py
    ./scripts/docker_up.sh --zed-detect --ground-truth

  = sources_sim.launch.py           (BiguaSim + SITL + MAVROS + zed_mimic + down_cam)
  + phase1_zed_detect.launch.py     (phase1 with the two cameras' jobs swapped back)

The counterpart of phase1_sim.launch.py, and a PURE WRAPPER for the same
reason: it passes the autonomy layer nothing and does not restate the mission's
arguments. Re-declaring one here would carry THIS file's default and overwrite
whatever phase1.launch.py declared — measured 2026-08-22, and the symptom is
that editing a default does nothing whenever the sim wrapper is the entry
point, which is almost always.

    ros2 launch hydrone_bringup phase1_zed_detect_sim.launch.py takeoff_alt:=3.0

reaches phase1.launch.py unchanged.

Read phase1_zed_detect.launch.py for what this mission is and what is measured
about it.
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

    args = [
        DeclareLaunchArgument("odom_source", default_value="vo"),
        DeclareLaunchArgument("odom_error", default_value="true"),
        DeclareLaunchArgument("odom_error_print", default_value="false"),
        DeclareLaunchArgument("odom_error_dir", default_value="/ws/logs"),
        DeclareLaunchArgument("vo_stereo", default_value="false"),
    ]

    sources = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "sources_sim.launch.py")),
        launch_arguments={
            "odom_source": LaunchConfiguration("odom_source"),
            "odom_error": LaunchConfiguration("odom_error"),
            "odom_error_print": LaunchConfiguration("odom_error_print"),
            "odom_error_dir": LaunchConfiguration("odom_error_dir"),
            "vo_stereo": LaunchConfiguration("vo_stereo"),
        }.items(),
    )

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "phase1_zed_detect.launch.py")),
    )

    return LaunchDescription(args + [sources, autonomy])
