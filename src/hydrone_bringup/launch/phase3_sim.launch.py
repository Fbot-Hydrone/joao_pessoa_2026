"""
hydrone_bringup/launch/phase3_sim.launch.py

PHASE 3 — gesture interaction, in simulation. The same aircraft and the same
LIO stack as Phase 4 (phase4_sim.launch.py, included whole), plus:

  config-KopisX8Cam.yaml   the Kopis with a forward FrontCamera
  gesture_detector_node    FrontCamera -> /hydrone/vision/human_gesture
  phase3_gesture_node      takeoff, 1 m forward, 90 deg right, find the operator,
                           then fly the gestures (docs/Phase 3 Gesture Mission.md)

    ros2 launch hydrone_bringup phase3_sim.launch.py      # or: docker_up.sh --phase3

BiguaSim has no person to gesture, so after FIND the drone hovers and sweeps
until a gesture arrives. Type them in (they hold while repeated, like an arm):

    ros2 topic pub -r 10 /hydrone/gesture/inject std_msgs/String "data: HOVER"
    ros2 topic pub -r 10 /hydrone/gesture/inject std_msgs/String "data: DIREITA"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Declares NOTHING (test_launch_arguments.py): every phase4_sim argument
    arrives by inheritance, command line included. The four forwards are what
    makes this Phase 3 and not Phase 4. agent_name and map_name forward with a
    fallback default, so a command-line value still wins; phase and mission are
    the point of this file and are fixed."""
    phase4 = os.path.join(get_package_share_directory('hydrone_bringup'), 'launch',
                          'phase4_sim.launch.py')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(phase4),
        launch_arguments={
            'agent_name': LaunchConfiguration('agent_name', default='KopisX8Cam'),
            'map_name': LaunchConfiguration('map_name', default='phase3'),
            'phase': '3',
            'mission': 'gesture',
        }.items(),
    )])
