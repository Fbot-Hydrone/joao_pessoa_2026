"""
hydrone_bringup/launch/sensors_real.launch.py

SENSORS ONLY on the real Kopis X8 — nothing here commands the vehicle:

    ros2 launch hydrone_bringup sensors_real.launch.py              # Livox only
    ros2 launch hydrone_bringup sensors_real.launch.py fcu:=true    # + Pixhawk

  livox_ros_driver2  -> /livox/lidar (PointCloud2, so rviz can draw it), /livox/imu
  mavros (fcu:=true) -> /mavros/state, /mavros/imu/data,
                        /mavros/rangefinder_pub (the rangefinder on the Pixhawk)
  static TF          base_link -> livox_frame, base_link -> lidar (rangefinder,
                     facing down; mavros' apm_config names its frame "lidar")

No mission, no setpoints, no external nav into the EKF: MAVROS only reads.
To look at it from the team computer:

    rviz2 -d src/hydrone_bringup/rviz/sensors_real.rviz

xfer_format is 0 (PointCloud2) here and 1 (CustomMsg) in phase4_real: FAST-LIO
reads CustomMsg, rviz cannot draw it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _floats(text):
    return [v.strip() for v in text.strip('[] ').split(',')]


def _launch_setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    bringup_pkg = get_package_share_directory('hydrone_bringup')
    mount = _floats(lc('lidar_mount'))

    livox = Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node',
        name='livox_lidar_publisher', output='screen',
        parameters=[{
            'xfer_format': 0,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': os.path.join(bringup_pkg, 'config', 'livox', 'mid360.json'),
            'cmdline_input_bd_code': 'livox0000000001',
        }],
    )

    tf_livox = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_livox',
        arguments=['--x', mount[0], '--y', mount[1], '--z', mount[2],
                   '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
    )

    # pitch +90 deg: the rangefinder's x axis points at the ground
    tf_range = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_rangefinder',
        arguments=['--pitch', '1.5708',
                   '--frame-id', 'base_link', '--child-frame-id', 'lidar'],
    )

    mavros_share = get_package_share_directory('mavros')
    mavros = Node(
        package='mavros', executable='mavros_node', output='screen',
        condition=IfCondition(LaunchConfiguration('fcu')),
        parameters=[
            os.path.join(mavros_share, 'launch', 'apm_pluginlists.yaml'),
            os.path.join(mavros_share, 'launch', 'apm_config.yaml'),
            {
                'fcu_url': lc('fcu_url'),
                'gcs_url': '',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
            },
        ],
    )

    return [livox, tf_livox, tf_range, mavros]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu', default_value='false',
            description='true: also start MAVROS against the Pixhawk (read only).'),
        DeclareLaunchArgument(
            'fcu_url', default_value='/dev/ttyAMA0:921600',
            description="The Pi 5's GPIO UART; /dev/ttyACM0:115200 over USB."),
        DeclareLaunchArgument(
            'lidar_mount', default_value='[0.0, 0.0, 0.1]',
            description='Mid-360 origin over base_link, m.'),
        OpaqueFunction(function=_launch_setup),
    ])
