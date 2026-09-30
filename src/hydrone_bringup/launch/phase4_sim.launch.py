"""
hydrone_bringup/launch/phase4_sim.launch.py

PHASE 4 — the Kopis X8 flying on its Livox Mid-360, in simulation:

    ros2 launch hydrone_bringup phase4_sim.launch.py

  ardubridge (BiguaSim <-> SITL, lidar wobble, ESC RPM)  +  ArduPilot SITL  +  MAVROS
  +  livox_mimic         -> /livox/lidar (CustomMsg), /livox/imu
  +  fastlio_mapping     -> /Odometry, /cloud_registered
  +  lio_odom_adapter    -> /hydrone/lio/odom, TF odom -> base_link
  +  motion_prior_node   -> /hydrone/lio/consistency (gates the adapter)
  +  vision_odom_bridge  -> LIO pose into the EKF as external nav
  +  lio_map_node        -> persistent map (/hydrone/map/*)
  +  odom_error_node     -> LIO drift vs ground truth, CSV (measurement only)

The EKF flies on the LIO. Ground truth reaches nothing but odom_error_node.
Docs: docs/Phase 4 Pipeline.md, docs/LIO Odometry.md, docs/Livox Mid-360 Sim.md.

Clocks: the bridge stamps every sensor with simulation time (anchored to the
wall clock at start, `stamp_clock:=sim`), because FAST-LIO integrates the IMU
over stamps and the sim runs slower than real time. vision_odom_bridge
restamps with now() on the way to MAVROS, as it always did.

Local simulator only: a `--world` bridge can't wobble the lidar (logged in the
docs as a known limit).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

BIGUASIM_NS = 'biguasim'
# scenario file's name for the Mid-360
LIDAR_SENSOR = 'Mid360'


def _biguasim_config_path(agent_name):
    config_dir = os.path.join(get_package_share_directory('biguasim_main'), 'config')
    path = os.path.join(config_dir, f'config-{agent_name}.yaml')
    if not os.path.exists(path):
        raise RuntimeError(f"no biguasim config for agent_name:={agent_name} ({path})")
    return path


def _find_scenario(node):
    if isinstance(node, dict):
        if 'biguasim_scenario' in node:
            return node['biguasim_scenario']
        for value in node.values():
            found = _find_scenario(value)
            if found is not None:
                return found
    elif isinstance(node, list):
        for item in node:
            found = _find_scenario(item)
            if found is not None:
                return found
    return None


def _agent_block(config_path):
    with open(config_path) as f:
        return _find_scenario(yaml.safe_load(f))['agents'][0]


def _sensor(agent, sensor_type, sensor_name=None):
    for s in agent.get('sensors', []):
        if s.get('sensor_type') != sensor_type:
            continue
        if sensor_name is None or s.get('sensor_name', sensor_type) == sensor_name:
            return s
    return None


def _launch_setup(context, *args, **kwargs):
    agent_name = LaunchConfiguration('agent_name').perform(context)
    agent = _agent_block(_biguasim_config_path(agent_name))
    prefix = f"/{BIGUASIM_NS}/{agent['agent_name']}_id0"

    lidar = _sensor(agent, 'RaycastLidar', LIDAR_SENSOR)
    if lidar is None:
        raise RuntimeError(
            f"{agent_name} declares no RaycastLidar '{LIDAR_SENSOR}', so there is no "
            "Mid-360 to fly on. Add it to the scenario file.")
    if _sensor(agent, 'IMUSensor') is None:
        raise RuntimeError(f"{agent_name} must list IMUSensor (ros_publish) for the Mid-360 IMU")
    # one source for the mount: the scenario file
    mount_xyz = [float(v) for v in lidar.get('location', (0.0, 0.0, 0.0))]
    mount_rpy = [float(v) for v in lidar.get('rotation', (0.0, 0.0, 0.0))]

    sitl_pkg = get_package_share_directory('ardupilot_sitl')
    bringup_pkg = get_package_share_directory('hydrone_bringup')
    lio_pkg = get_package_share_directory('hydrone_lio')
    biguasim_launch = os.path.join(get_package_share_directory('biguasim_main'), 'launch')

    if os.environ.get('WORLD_ADDRESS', '').strip():
        raise RuntimeError("phase 4 runs on the local simulator only: a remote world "
                           "can't wobble the lidar. Unset WORLD_ADDRESS / drop --world.")

    ardubridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(biguasim_launch, 'ardubridge.launch.py')),
        launch_arguments={
            'agent_name': agent_name,
            'phase': LaunchConfiguration('phase'),
            'stamp_clock': 'sim',
        }.items(),
    )

    sitl_dds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sitl_pkg, 'launch', 'sitl_dds_udp.launch.py')),
        launch_arguments={
            'transport': 'udp4',
            'port': '2019',
            'synthetic_clock': 'True',
            'wipe': 'True',
            'model': 'JSON',
            'speedup': '1',
            'slave': '0',
            'instance': '0',
            # kopis_sitl.parm last: it overlays the shared Holybro file
            'defaults': ','.join([
                os.path.join(bringup_pkg, 'config', 'params', 'holybro_sitl.parm'),
                os.path.join(sitl_pkg, 'config', 'default_params', 'dds_udp.parm'),
                os.path.join(bringup_pkg, 'config', 'params', 'kopis_sitl.parm'),
            ]),
            'sim_address': '127.0.0.1',
            'master': 'tcp:127.0.0.1:5760',
            'sitl': '127.0.0.1:5501',
        }.items(),
    )

    livox = Node(
        package='hydrone_bringup', executable='livox_mimic_node', name='livox_mimic',
        output='screen',
        parameters=[{
            'in_cloud': f'{prefix}/{LIDAR_SENSOR}',
            'in_imu': f'{prefix}/IMUSensor',
            'mount_xyz': mount_xyz,
            'mount_rpy_deg': mount_rpy,
        }],
    )

    fast_lio = Node(
        package='fast_lio', executable='fastlio_mapping', name='fast_lio',
        output='screen',
        parameters=[os.path.join(lio_pkg, 'config', 'fast_lio_mid360_sim.yaml')],
    )

    adapter = Node(
        package='hydrone_lio', executable='lio_odom_adapter', output='screen',
        parameters=[{
            'mount_xyz': mount_xyz,
            'mount_rpy_deg': mount_rpy,
            'gate_enabled': LaunchConfiguration('gate'),
        }],
    )

    prior = Node(
        package='hydrone_lio', executable='motion_prior_node', output='screen',
        # the sim has no ESC telemetry through ArduPilot's JSON backend, so the
        # bridge publishes the same message on its own topic
        parameters=[{'in_esc': '/hydrone/sim/esc_telemetry'}],
    )

    mavros_share = get_package_share_directory('mavros')
    # respawn: under the slow sim MAVROS sometimes dies at startup with
    # "std::future_error: Promise already satisfied" (command retry race while
    # timesync RTT is seconds); a restart comes up fine
    mavros = Node(
        package='mavros', executable='mavros_node', output='screen',
        respawn=True, respawn_delay=3.0,
        parameters=[
            # apm_pluginlists minus vision_speed_estimate, which the LIO velocity needs
            os.path.join(lio_pkg, 'config', 'mavros_pluginlists_phase4.yaml'),
            os.path.join(mavros_share, 'launch', 'apm_config.yaml'),
            os.path.join(bringup_pkg, 'config', 'timeouts.yaml'),
            {
                'fcu_url': 'udp://:14551@',
                'gcs_url': '',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
            },
        ],
    )

    lio_nav = Node(
        package='hydrone_bringup', executable='vision_odom_bridge', name='lio_nav',
        output='screen',
        parameters=[
            os.path.join(bringup_pkg, 'config', 'timeouts.yaml'),
            # ground_truth is a DEBUGGING AID for tuning the airframe apart
            # from the estimator; a flight on it proves nothing about the LIO
            {'in_odom': '/hydrone/lio/odom' if LaunchConfiguration('ext_nav').perform(context) == 'lio'
             else f'{prefix}/DynamicsSensor/Odom',
             'out_speed': '/mavros/vision_speed/speed_twist'},
        ],
    )

    lio_map = Node(
        package='hydrone_lio', executable='lio_map_node', output='screen',
        parameters=[{
            'map_name': LaunchConfiguration('map_name'),
            'load_map': LaunchConfiguration('load_map'),
        }],
    )

    odom_error = Node(
        package='hydrone_bringup', executable='odom_error_node', name='lio_error',
        output='screen',
        condition=IfCondition(LaunchConfiguration('measure_drift')),
        parameters=[{
            'in_odom': '/hydrone/lio/odom_raw',
            'in_odom_gt': f'{prefix}/DynamicsSensor/Odom',
            'print_diff': True,
            # next to the maps, so it survives the container
            'log_dir': '/ws/maps/logs',
        }],
    )

    maze = Node(
        package='hydrone_mission', executable='phase4_maze_node', output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mission'), "' == 'maze'"])),
        parameters=[{
            # /cloud_registered is in camera_init; odom is the lidar mount above it
            'lidar_mount': mount_xyz,
            'flight_z': float(LaunchConfiguration('flight_z').perform(context)),
            'auto_start': LaunchConfiguration('auto_start').perform(context).lower() == 'true',
        }],
    )

    # Phase 3 (mission:=gesture, via phase3_sim.launch.py): the camera the
    # scenario calls FrontCamera -> gestures -> the mission. The sim has no
    # person in it, so /hydrone/gesture/inject stands in for the arms.
    is_gesture = IfCondition(PythonExpression(["'", LaunchConfiguration('mission'), "' == 'gesture'"]))
    gesture_detector = Node(
        package='hydrone_vision', executable='gesture_detector_node', output='screen',
        condition=is_gesture,
        parameters=[{
            'source': 'topic',
            'image_topic': f'{prefix}/FrontCamera',
        }],
    )
    gesture_mission = Node(
        package='hydrone_mission', executable='phase3_gesture_node', output='screen',
        condition=is_gesture,
        parameters=[{
            'lidar_mount': mount_xyz,
            'auto_start': LaunchConfiguration('auto_start').perform(context).lower() == 'true',
            'allow_inject': LaunchConfiguration('allow_inject').perform(context).lower() == 'true',
        }],
    )

    return [ardubridge, sitl_dds, livox, fast_lio, adapter, prior, mavros, lio_nav,
            lio_map, odom_error, maze, gesture_detector, gesture_mission]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_name', default_value='KopisX8',
            description='biguasim scenario config-<agent_name>.yaml; must declare '
                        'the Mid360 RaycastLidar and IMUSensor.'),
        DeclareLaunchArgument(
            'phase', default_value='4',
            description='Competition phase passed to ardubridge (3 or 4 fly the Kopis).'),
        DeclareLaunchArgument(
            'ext_nav', default_value='lio',
            description="What the EKF flies on: 'lio', or 'ground_truth' to tune "
                        'the airframe without the estimator in the loop.'),
        DeclareLaunchArgument(
            'gate', default_value='true',
            description='Hold LIO poses back from the EKF when they disagree with '
                        'the motor physics.'),
        DeclareLaunchArgument(
            'map_name', default_value='phase4',
            description='Saved under maps/<map_name>/ at the repo root.'),
        DeclareLaunchArgument(
            'load_map', default_value='false',
            description='Start from the saved map of the same name.'),
        DeclareLaunchArgument(
            'mission', default_value='none',
            description="'maze' flies phase4_maze_node through the structure "
                        "right of spawn; 'gesture' flies the Phase 3 gesture mission "
                        '(use phase3_sim.launch.py); none just brings the vehicle up.'),
        DeclareLaunchArgument(
            'flight_z', default_value='0.5',
            description='odom height the maze mission plans at (window centre).'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='Let the maze mission arm and take off by itself.'),
        DeclareLaunchArgument(
            'allow_inject', default_value='true',
            description='mission:=gesture: accept gestures typed on /hydrone/gesture/inject.'),
        DeclareLaunchArgument(
            'measure_drift', default_value='true',
            description='Log LIO drift against ground truth (sim only, never fed back).'),
        OpaqueFunction(function=_launch_setup),
    ])
