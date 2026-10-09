"""
hydrone_bringup/launch/phase2_sim.launch.py — Fase 2 no simulador, um comando:

    ./scripts/docker_up.sh --phase2 --ground-truth [--debug]

  = sources_sim.launch.py (BiguaSim + SITL + MAVROS + cameras; the ardubridge
    spawns the Phase 2 layout because docker_up exports ARENA_PHASE=2, and it
    serves /hydrone/gripper/command)
  + phase2.launch.py      (the autonomy, world-agnostic)

A pure wrapper, like phase1_sim.launch.py: no overrides into the autonomy.
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
            k: LaunchConfiguration(k) for k in (
                "odom_source", "odom_error", "odom_error_print",
                "odom_error_dir", "vo_stereo")}.items())
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "phase2.launch.py")))
    return LaunchDescription(args + [sources, autonomy])
