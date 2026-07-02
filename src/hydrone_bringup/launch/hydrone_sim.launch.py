"""
hydrone_bringup/launch/hydrone_sim.launch.py

Simulation bringup: ArduPilot SITL flying inside BiguaSim.

Starts, in one launch:
  1. ardubridge_node (biguasim_main) — BiguaSim as the physics/sensor
     backend for SITL: receives motor PWM over UDP (JSON model), steps the
     sim, sends state back, publishes BiguaSim sensors to ROS 2.
  2. ArduPilot SITL + micro_ros_agent + MAVProxy, via the upstream
     ardupilot_sitl composite launch (sitl_dds_udp.launch.py).

The BiguaSim UE5 simulator must be reachable on this machine (the
`biguasim` Python package launches/attaches to it — see the README).

The hydrone autonomy stack is started separately:
  ros2 launch hydrone_bringup hydrone.launch.py phase:=1

Notes:
  - No `refs:=dds_xrce_profile.xml` is passed: this ArduPilot version
    creates DDS entities from the client side and the profile file no
    longer exists upstream. Passing a missing path kills the agent.
  - micro_ros_agent is NOT launched here explicitly: sitl_dds_udp.launch.py
    already includes it (a second agent on the same port conflicts).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sitl_pkg = get_package_share_directory('ardupilot_sitl')
    bringup_pkg = get_package_share_directory('hydrone_bringup')

    holybro_parm = os.path.join(
        bringup_pkg, 'config', 'params', 'holybro_sitl.parm')
    dds_udp_parm = os.path.join(
        sitl_pkg, 'config', 'default_params', 'dds_udp.parm')

    ardubridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('biguasim_main'),
                         'launch', 'ardubridge.launch.py')
        )
    )

    sitl_dds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sitl_pkg, 'launch', 'sitl_dds_udp.launch.py')
        ),
        launch_arguments={
            'transport': 'udp4',
            'port': '2019',
            'synthetic_clock': 'True',
            'wipe': 'True',
            'model': 'JSON',
            'speedup': '1',
            'slave': '0',
            'instance': '0',
            'defaults': f'{holybro_parm},{dds_udp_parm}',
            'sim_address': '127.0.0.1',
            'master': 'tcp:127.0.0.1:5760',
            'sitl': '127.0.0.1:5501',
        }.items(),
    )

    return LaunchDescription([
        ardubridge,
        sitl_dds,
    ])
