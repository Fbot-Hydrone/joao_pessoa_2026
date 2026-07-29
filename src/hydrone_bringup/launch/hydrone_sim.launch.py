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

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# Must match the namespace set in biguasim_main/launch/ardubridge.launch.py.
BIGUASIM_NS = 'biguasim'


def _find_biguasim_scenario(node):
    """Recursively locate the 'biguasim_scenario' block in a parsed config."""
    if isinstance(node, dict):
        if 'biguasim_scenario' in node:
            return node['biguasim_scenario']
        for value in node.values():
            found = _find_biguasim_scenario(value)
            if found is not None:
                return found
    elif isinstance(node, list):
        for item in node:
            found = _find_biguasim_scenario(item)
            if found is not None:
                return found
    return None


def _biguasim_topic_prefix(config_path):
    """Build the ROS topic prefix that ardubridge_node publishes sensors under.

    The agent name comes from config.yaml (single source of truth). BiguaSim's
    environment appends a batch suffix to it -> '<name>-id0', which the ROS
    bridge renders as '<name>_id0'. Single-agent only for now; multi-agent would
    need per-index prefixes.
    """
    with open(config_path) as f:
        scenario = _find_biguasim_scenario(yaml.safe_load(f))
    agent_name = scenario['agents'][0]['agent_name']
    return f'/{BIGUASIM_NS}/{agent_name}_id0'


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
            # Synthetic clock: ArduPilot advances time in lockstep with the FDM,
            # so control stays stable even when the sim runs below real-time.
            # (Wall-clock made ArduPilot over-control on stale sensor data ->
            # uncontrollable hover. The MAVROS timesync warnings it causes are
            # cosmetic; vision fusion still works via the reliable-QoS pose.)
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

    # Fake ZED: republishes BiguaSim sensors under the real ZED wrapper's
    # topic names and frames. On the real drone, launch zed_wrapper instead.
    # Input topics are derived from the biguasim agent name in config.yaml so
    # renaming the agent in one place propagates to the bridge and here.
    biguasim_config = os.path.join(
        get_package_share_directory('biguasim_main'), 'config', 'config.yaml')
    prefix = _biguasim_topic_prefix(biguasim_config)

    zed_mimic = Node(
        package='hydrone_bringup',
        executable='zed_mimic_node',
        output='screen',
        parameters=[{
            'in_rgb':      f'{prefix}/RGBCamera',
            'in_rgb_info': f'{prefix}/RGBCamera/camera_info',
            'in_depth':    f'{prefix}/DepthCamera',
            'in_odom':     f'{prefix}/DynamicsSensor/Odom',
            'in_imu':      f'{prefix}/DynamicsSensor/IMU',
            # Real VO (visual_odometry_node) owns /zed/zed_node/odom; ground truth
            # goes to /zed/zed_node/odom_GT so the two can be compared in RViz/rqt.
            'out_odom':    '/zed/zed_node/odom_GT',
        }],
    )

    # Real visual odometry on the ZED RGB-D stream (ORB features -> depth
    # back-projection -> PnP/RANSAC), the honest analogue of the ZED 2i's
    # stereo-VO core. Owns /zed/zed_node/odom; vision_odom_bridge is unchanged.
    visual_odometry = Node(
        package='hydrone_bringup',
        executable='visual_odometry_node',
        output='screen',
    )

    # MAVROS + visual-odometry feed. The SITL params (holybro_sitl.parm) put
    # EKF3 on external nav with GPS disabled (CBR rules), so the vehicle can only
    # hold position if it is fed vision — hence this lives in the base sim, not
    # just the mission launch. mavros connects to MAVProxy's udp:14551 output;
    # apm_pluginlists.yaml is what makes the vision_pose plugin load.
    mavros_share = get_package_share_directory("mavros")
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[
            os.path.join(mavros_share, "launch", "apm_pluginlists.yaml"),
            os.path.join(mavros_share, "launch", "apm_config.yaml"),
            {
                "fcu_url": "udp://:14551@",
                "gcs_url": "",
                "tgt_system": 1,
                "tgt_component": 1,
                "fcu_protocol": "v2.0",
            },
        ],
    )

    # /zed/zed_node/odom -> /mavros/vision_pose/pose (VISION_POSITION_ESTIMATE).
    vision_odom = Node(
        package="hydrone_bringup",
        executable="vision_odom_bridge",
        output="screen",
    )

    return LaunchDescription([
        ardubridge,
        sitl_dds,
        zed_mimic,
        visual_odometry,
        mavros,
        vision_odom,
    ])
