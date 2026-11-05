"""
hydrone_bringup/launch/phase3_real.launch.py

ONE COMMAND for Phase 3 on the REAL Kopis X8:

    ros2 launch hydrone_bringup phase3_real.launch.py
    ros2 service call /hydrone/phase3/start std_srvs/srv/Trigger    # then: go

  = phase4_real.launch.py  (Mid-360 + FAST-LIO + MAVROS, no maze)
  + phase3.launch.py       (phase3_hri_node, unchanged)
  + gesture_camera         (the drone's camera -> /hydrone/vision/human_gesture)

The mirror of phase3_sim.launch.py, where gesture_terminal or the PC's webcam
stand in for the camera. Mission arguments (takeoff_alt, step_m, ...) are NOT
declared here: they reach phase3.launch.py by inheritance, so its defaults are
the only ones.

ONE exception: auto_start is forwarded as false. It is not a mission setting
but a fact about the real drone — nothing arms until the team computer calls
/hydrone/phase3/start. `auto_start:=true` on the command line still wins.

No preview window (show:=false): the Pi flies headless. The gestures are
printed for the judge either way.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("hydrone_bringup"), "launch")

    sources = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "phase4_real.launch.py")),
        launch_arguments={
            "mission": "none",     # the maze node would fight for the vehicle
            "map_name": "phase3",
        }.items(),
    )

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "phase3.launch.py")),
        launch_arguments={"auto_start": LaunchConfiguration("auto_start", default="false")}.items(),
    )

    camera = Node(
        package="hydrone_mission", executable="gesture_camera", output="screen",
        emulate_tty=True,
        parameters=[{
            "camera": LaunchConfiguration("camera"),
            "flip": ParameterValue(LaunchConfiguration("camera_flip"), value_type=bool),
            "show": False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera", default_value="/dev/video0",
            description="The camera facing the operator: /dev/videoN or an index."),
        DeclareLaunchArgument(
            "camera_flip", default_value="false",
            description="true if DIREITA/ESQUERDA come out swapped."),
        sources, autonomy, camera,
    ])
