"""
hydrone_bringup/launch/phase4_real.launch.py

PHASE 4 on the REAL Kopis X8 (Raspberry Pi 5, Livox Mid-360, Pixhawk):

    ros2 launch hydrone_bringup phase4_real.launch.py                 # vehicle only
    ros2 launch hydrone_bringup phase4_real.launch.py mission:=maze   # + the maze
    ros2 service call /hydrone/phase4/start std_srvs/srv/Trigger       # then: go

The real-hardware counterpart of phase4_sim.launch.py. Everything above the
sources is the same node with the same parameters; what differs:

  livox_ros_driver2   the Mid-360 itself (config/livox/mid360.json) replaces
                      livox_mimic + ardubridge -> /livox/lidar, /livox/imu
  fast_lio            fast_lio_mid360_real.yaml: the real IMU sits off the
                      lidar origin, the mimic's does not
  mavros              fcu_url to the Pixhawk instead of SITL; no timeouts.yaml
                      (real time runs at real time)
  motion_prior_node   only with gate:=true — it needs ESC RPM telemetry on
                      /mavros/esc_telemetry/telemetry, which the sim fakes
  no odom_error_node  there is no ground truth to measure against

Mount: the sim reads it from the scenario file; here it is lidar_mount /
lidar_rpy_deg, defaulting to the scenario's (0.1 m over base_link). Measure it.

Network: the Mid-360 at 192.168.1.145 sends to 192.168.1.50 on eth0; both IPs
live in config/livox/mid360.json.

NOT FLOWN YET. Before the first flight: props off, run it, move the drone by
hand and check /hydrone/lio/odom follows; confirm the EKF uses external nav
(EK3_SRC1_POSXY/VELXY/POSZ/YAW = 6) and that a flip to LOITER on the RC takes
over.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _floats(text):
    return [float(v) for v in text.strip('[] ').split(',')]


def _launch_setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    bringup_pkg = get_package_share_directory('hydrone_bringup')
    lio_pkg = get_package_share_directory('hydrone_lio')
    mavros_share = get_package_share_directory('mavros')
    mount_xyz = _floats(lc('lidar_mount'))
    mount_rpy = _floats(lc('lidar_rpy_deg'))

    livox = Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node',
        name='livox_lidar_publisher', output='screen',
        parameters=[{
            'xfer_format': 1,       # CustomMsg, what FAST-LIO reads
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': os.path.join(bringup_pkg, 'config', 'livox', 'mid360.json'),
            'cmdline_input_bd_code': 'livox0000000001',
        }],
    )

    fast_lio = Node(
        package='fast_lio', executable='fastlio_mapping', name='fast_lio',
        output='screen',
        parameters=[os.path.join(lio_pkg, 'config', 'fast_lio_mid360_real.yaml')],
    )

    adapter = Node(
        package='hydrone_lio', executable='lio_odom_adapter', output='screen',
        parameters=[{
            'mount_xyz': mount_xyz,
            'mount_rpy_deg': mount_rpy,
            'gate_enabled': lc('gate').lower() == 'true',
        }],
    )

    prior = Node(
        package='hydrone_lio', executable='motion_prior_node', output='screen',
        condition=IfCondition(LaunchConfiguration('gate')),
    )

    mavros = Node(
        package='mavros', executable='mavros_node', output='screen',
        respawn=True, respawn_delay=3.0,
        parameters=[
            # apm_pluginlists minus vision_speed_estimate, which the LIO velocity needs
            os.path.join(lio_pkg, 'config', 'mavros_pluginlists_phase4.yaml'),
            os.path.join(mavros_share, 'launch', 'apm_config.yaml'),
            {
                'fcu_url': lc('fcu_url'),
                'gcs_url': lc('gcs_url'),
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
            },
        ],
    )

    lio_nav = Node(
        package='hydrone_bringup', executable='vision_odom_bridge', name='lio_nav',
        output='screen',
        parameters=[{'in_odom': '/hydrone/lio/odom',
                     'out_speed': '/mavros/vision_speed/speed_twist'}],
    )

    lio_map = Node(
        package='hydrone_lio', executable='lio_map_node', output='screen',
        parameters=[{
            'map_name': LaunchConfiguration('map_name'),
            'load_map': LaunchConfiguration('load_map'),
        }],
    )

    maze = Node(
        package='hydrone_mission', executable='phase4_maze_node', output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mission'), "' == 'maze'"])),
        parameters=[{
            'lidar_mount': mount_xyz,
            'flight_z': float(lc('flight_z')),
            'auto_start': lc('auto_start').lower() == 'true',
        }],
    )

    return [livox, fast_lio, adapter, prior, mavros, lio_nav, lio_map, maze]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url', default_value='/dev/ttyAMA0:921600',
            description="Pixhawk link: the Pi 5's GPIO UART by default; "
                        '/dev/ttyACM0:115200 over USB.'),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument(
            'lidar_mount', default_value='[0.0, 0.0, 0.1]',
            description='Mid-360 origin over base_link, m. The sim mount — measure it.'),
        DeclareLaunchArgument('lidar_rpy_deg', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument(
            'gate', default_value='false',
            description='Motion-prior gate: needs ESC RPM telemetry on '
                        '/mavros/esc_telemetry/telemetry.'),
        DeclareLaunchArgument(
            'map_name', default_value='phase4',
            description='Saved under $HYDRONE_MAPS/<map_name>/.'),
        DeclareLaunchArgument('load_map', default_value='false'),
        DeclareLaunchArgument(
            'mission', default_value='none',
            description="'maze' runs phase4_maze_node; none just brings the vehicle up."),
        DeclareLaunchArgument('flight_z', default_value='0.5'),
        DeclareLaunchArgument(
            'auto_start', default_value='false',
            description='false: the maze waits for /hydrone/phase4/start. true arms '
                        'by itself 5 s after odometry: sim habit, not for the drone.'),
        OpaqueFunction(function=_launch_setup),
    ])
