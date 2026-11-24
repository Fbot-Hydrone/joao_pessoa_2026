"""
hydrone_bringup/launch/phase2_real.launch.py — Fase 2 on the drone, one command:

    ros2 launch hydrone_bringup phase2_real.launch.py \\
        arena_walls_m:="[front, back, left, right]" \\
        pickup_1:="[x, y]" ... delivery_3:="[x, y]"

  = sources_real.launch.py (ZED + belly camera + MAVROS + gripper)
  + phase2.launch.py       (the autonomy, unchanged from the sim)

Same shape as phase1_real.launch.py: it DECLARES NOTHING, so every argument
of either include still comes from the command line with that file's own
default (test_launch_arguments.py). It only forwards facts about this
airframe and the real arena frame, each as LaunchConfiguration(name,
default=...) so a command-line value still wins:

  gripper          on: Phase 2 needs it
  belly_offset_xy  the belly camera's mount (phase1_real's number)
  ground_z         the floor in the FCU's z (phase1_real's ground_z())
  arena_yaw_deg    0: the real map has +y where the nose points at arming
  layout_file      phase2_bases_real.yaml: positions FROM THE TAKEOFF BASE
                   (x right, y forward), none known -> scan unless given
  cruise_alt       3.0 m above the floor, Phase 1's real height
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

# Belly camera from the motor centre, [x forward, y RIGHT], m. The same
# constant as phase1_real.launch.py (measured 2026-11-18; the C922 swap of
# 2026-11-23 put it at 6 cm left, still to be checked).
BELLY_OFFSET_XY = "[0.065, -0.071]"


def ground_z():
    """phase1_real.launch.py's ground_z(): 0 with surface_offset, else minus
    the takeoff base's height."""
    return [PythonExpression([
        "0.0 if '", LaunchConfiguration("surface_offset", default="true"),
        "'.lower() in ('true', '1') else -float('",
        LaunchConfiguration("takeoff_base_height_m", default="0.5"), "')"])]


def generate_launch_description():
    bringup = get_package_share_directory("hydrone_bringup")
    launch_dir = os.path.join(bringup, "launch")
    sources = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "sources_real.launch.py")),
        launch_arguments={
            "gripper": LaunchConfiguration("gripper", default="true"),
        }.items(),
    )
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "phase2.launch.py")),
        launch_arguments={
            "belly_offset_xy": LaunchConfiguration(
                "belly_offset_xy", default=BELLY_OFFSET_XY),
            "ground_z": LaunchConfiguration("ground_z", default=ground_z()),
            "arena_yaw_deg": LaunchConfiguration("arena_yaw_deg",
                                                 default="0.0"),
            "layout_file": LaunchConfiguration("layout_file", default=os.path.join(
                bringup, "config", "phase2_bases_real.yaml")),
            "cruise_alt": LaunchConfiguration("cruise_alt", default="3.0"),
        }.items(),
    )
    return LaunchDescription([sources, autonomy])
