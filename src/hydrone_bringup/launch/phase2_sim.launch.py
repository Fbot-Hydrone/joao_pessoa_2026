"""
hydrone_bringup/launch/phase2_sim.launch.py — Fase 2 no simulador, um comando:

    ./scripts/docker_up.sh --phase2 --ground-truth [--debug]

  = sources_sim.launch.py (BiguaSim + SITL + MAVROS + cameras; the ardubridge
    spawns the Phase 2 layout because docker_up exports ARENA_PHASE=2, and it
    serves /hydrone/gripper/command)
  + phase2.launch.py      (the autonomy, world-agnostic)

A pure wrapper, like phase1_sim.launch.py: the only overrides are where the
task log and the bag go (log_dir, the mounted /ws/logs) and close_in_touch
(the sim gripper only grabs in the air).
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
    # The container keeps only /ws/logs (./logs on the host) after `down`.
    # The sim's grab volume catches the kit only BEFORE landing (a ground close
    # came back "nothing in reach", 2026-11-24); the real fingers are the other
    # way round, so closing in TOUCH is a fact about the sim.
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "phase2.launch.py")),
        launch_arguments={
            "log_dir": LaunchConfiguration("log_dir", default="/ws/logs"),
            "close_in_touch": LaunchConfiguration("close_in_touch",
                                                  default="true"),
        }.items())
    return LaunchDescription(args + [sources, autonomy])
