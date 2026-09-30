"""
hydrone_bringup/launch/phase4_simple.launch.py

PHASE 4, simple backend: the known-map maze stack on the kinematic sim.
No BiguaSim, no SITL, no MAVROS, no FAST-LIO; runs anywhere in seconds.

    ros2 launch hydrone_bringup phase4_simple.launch.py
    ros2 launch hydrone_bringup phase4_simple.launch.py closed:="['b2_t2']" degen_rate:=0.15

  kinematic_sim_node   box world from the map YAML, Mid-360-like cloud, FCU mock
                       (MAVROS topics and services), drifting LIO-like odometry
  beam_localizer_node  beams vs the known map -> map pose
  pose_mux_node        LIO outside / map pose inside -> /hydrone/nav/odom
  vision_odom_bridge   /hydrone/nav/odom -> /mavros/vision_pose (the FCU mock reads it)
  phase4_known_node    the mission
  viz_udp_node         -> tools/maze_viewer (UDP 47800)

The same nodes run in phase4_sim.launch.py mission:=known against BiguaSim.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    map_yaml = lc('map_yaml') or os.path.join(
        get_package_share_directory('hydrone_mission'), 'config', 'phase4', 'known_maps', 'rulebook.yaml')
    world_yaml = lc('world_yaml') or map_yaml
    closed = [c.strip(" '\"") for c in lc('closed').strip('[]').split(',') if c.strip(" '\"")]
    bringup = get_package_share_directory('hydrone_bringup')
    return [
        Node(package='hydrone_confined', executable='kinematic_sim_node', name='kinematic_sim', output='screen',
             parameters=[{'map_yaml': world_yaml, 'qr': int(lc('qr')), 'seed': int(lc('seed')),
                          'closed': closed or [''], 'degen_rate': float(lc('degen_rate')),
                          'yaw_error_deg': float(lc('yaw_error_deg'))}]),
        Node(package='hydrone_confined', executable='beam_localizer_node', name='beam_localizer', output='screen',
             parameters=[{'map_yaml': map_yaml, 'cloud': '/livox/lidar_pc', 'cloud_type': 'pc2'}]),
        Node(package='hydrone_confined', executable='pose_mux_node', name='pose_mux', output='screen',
             parameters=[{'map_yaml': map_yaml}]),
        Node(package='hydrone_bringup', executable='vision_odom_bridge', name='nav_bridge', output='screen',
             parameters=[os.path.join(bringup, 'config', 'timeouts.yaml'),
                         {'in_odom': '/hydrone/nav/odom', 'out_speed': '/mavros/vision_speed/speed_twist',
                          'origin_max_sends': 0}]),
        Node(package='hydrone_mission', executable='phase4_known_node', output='screen',
             parameters=[{'map_yaml': map_yaml, 'spin': lc('spin').lower() == 'true',
                          'start_delay': 3.0}]),
        Node(package='hydrone_confined', executable='viz_udp_node', name='viz_udp', output='screen',
             parameters=[{'map_yaml': map_yaml}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', default_value='', description='known map the drone gets (default: rulebook)'),
        DeclareLaunchArgument('world_yaml', default_value='', description='map the sim world is built from (default: map_yaml)'),
        DeclareLaunchArgument('closed', default_value='[]', description='passages walled up in the world, not in the map'),
        DeclareLaunchArgument('qr', default_value='5'),
        DeclareLaunchArgument('seed', default_value='2'),
        DeclareLaunchArgument('degen_rate', default_value='0.0', description='m/s of LIO sliding while inside'),
        DeclareLaunchArgument('yaw_error_deg', default_value='0.0'),
        DeclareLaunchArgument('spin', default_value='true', description='turn around at each look point (QR)'),
        OpaqueFunction(function=_setup),
    ])
