"""
hydrone_bringup/launch/phase3_real.launch.py

PHASE 3 on the REAL Kopis X8 (Raspberry Pi 5, Livox Mid-360, USB camera,
Pixhawk). Same autonomy as phase3_sim.launch.py; only the sources differ:

  livox_ros_driver2 (NOT started here, see below) -> /livox/lidar, /livox/imu
  fastlio_mapping     fast_lio_mid360_real.yaml   -> /Odometry, /cloud_registered
  lio_odom_adapter                                -> /hydrone/lio/odom(_raw), TF
  motion_prior_node   (gate:=true only)           -> /hydrone/lio/consistency
  mavros              fcu_url                     -> /mavros/*
  vision_odom_bridge  LIO pose + velocity into the EKF (external nav)
  gesture_detector    the USB camera, opened directly (no image over DDS)
  phase3_gesture_node auto_start FALSE: nothing arms until

      ros2 service call /hydrone/phase3/start std_srvs/srv/Trigger

The Livox driver: src/livox_ros_driver2 in this repo is only its messages.
Run the full driver (Livox-SDK2 + livox_ros_driver2) beside this, publishing
CustomMsg (xfer_format 1) on /livox/lidar with the IMU on /livox/imu:

    ros2 launch livox_ros_driver2 msg_MID360_launch.py

NOT FLOWN ON HARDWARE YET. Before the first flight: props off, run it, move the
drone by hand and check /hydrone/lio/odom follows; wave at the camera and watch
/hydrone/gesture/debug_image and the [PROVA3] lines; confirm the EKF uses
external nav (EK3_SRC1_POSXY/VELXY/POSZ/YAW = 6) and that a flip to LOITER on
the RC takes over (the node goes silent on any mode it did not ask for).
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def _floats(text):
    return [float(v) for v in text.strip('[] ').split(',')]


def _launch_setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    bringup_pkg = get_package_share_directory('hydrone_bringup')
    lio_pkg = get_package_share_directory('hydrone_lio')
    mavros_share = get_package_share_directory('mavros')
    mount_xyz = _floats(lc('lidar_mount'))
    mount_rpy = _floats(lc('lidar_rpy_deg'))
    gate = lc('gate').lower() == 'true'

    fast_lio = Node(
        package='fast_lio', executable='fastlio_mapping', name='fast_lio', output='screen',
        parameters=[os.path.join(lio_pkg, 'config', 'fast_lio_mid360_real.yaml')])

    adapter = Node(
        package='hydrone_lio', executable='lio_odom_adapter', output='screen',
        parameters=[{'mount_xyz': mount_xyz, 'mount_rpy_deg': mount_rpy,
                     'gate_enabled': gate}])

    prior = Node(
        package='hydrone_lio', executable='motion_prior_node', output='screen',
        condition=IfCondition(LaunchConfiguration('gate')))

    mavros = Node(
        package='mavros', executable='mavros_node', output='screen',
        respawn=True, respawn_delay=3.0,
        parameters=[
            os.path.join(lio_pkg, 'config', 'mavros_pluginlists_phase4.yaml'),
            os.path.join(mavros_share, 'launch', 'apm_config.yaml'),
            os.path.join(bringup_pkg, 'config', 'timeouts.yaml'),
            {'fcu_url': lc('fcu_url'), 'gcs_url': lc('gcs_url'),
             'tgt_system': 1, 'tgt_component': 1, 'fcu_protocol': 'v2.0'},
        ])

    lio_nav = Node(
        package='hydrone_bringup', executable='vision_odom_bridge', name='lio_nav',
        output='screen',
        parameters=[os.path.join(bringup_pkg, 'config', 'timeouts.yaml'),
                    {'in_odom': '/hydrone/lio/odom',
                     'out_speed': '/mavros/vision_speed/speed_twist'}])

    detector = Node(
        package='hydrone_vision', executable='gesture_detector_node', output='screen',
        parameters=[{'source': 'device', 'device': lc('camera_device'),
                     'rotate_deg': int(lc('camera_rotate_deg')),
                     'mirror': lc('camera_mirror').lower() == 'true',
                     'debug_image': lc('debug_image').lower() == 'true'}])

    mission = Node(
        package='hydrone_mission', executable='phase3_gesture_node', output='screen',
        parameters=[{'lidar_mount': mount_xyz,
                     'auto_start': lc('auto_start').lower() == 'true',
                     'allow_inject': False,
                     'takeoff_alt': float(lc('takeoff_alt')),
                     'forward_dist': float(lc('forward_dist')),
                     'turn_deg': float(lc('turn_deg'))}])

    return [fast_lio, adapter, prior, mavros, lio_nav, detector, mission]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyAMA0:921600',
                              description="Pixhawk link: the Pi 5's UART by default; "
                                          '/dev/ttyACM0:115200 over USB.'),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('lidar_mount', default_value='[0.0, 0.0, 0.1]',
                              description='Mid-360 origin over base_link, m (measure it).'),
        DeclareLaunchArgument('lidar_rpy_deg', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('gate', default_value='false',
                              description='Motion-prior gate: needs ESC RPM telemetry on '
                                          '/mavros/esc_telemetry/telemetry.'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_rotate_deg', default_value='0'),
        DeclareLaunchArgument('camera_mirror', default_value='false',
                              description='true if the camera mirrors the image '
                                          '(DIREITA/ESQUERDA come out swapped).'),
        DeclareLaunchArgument('debug_image', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='true arms by itself 5 s after odometry: sim only.'),
        DeclareLaunchArgument('takeoff_alt', default_value='1.0'),
        DeclareLaunchArgument('forward_dist', default_value='1.0'),
        DeclareLaunchArgument('turn_deg', default_value='-90.0'),
        OpaqueFunction(function=_launch_setup),
    ])
