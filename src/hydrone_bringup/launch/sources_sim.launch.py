"""
hydrone_bringup/launch/sources_sim.launch.py

SIM SOURCES layer. Produces the agnostic contract buses from the simulation:

  /zed/zed_node/*   (sensors)   and   /mavros/*   (flight)

Everything above this layer (the autonomy stack in hydrone.launch.py) consumes
ONLY those two buses and is identical for sim and real. The real counterpart is
sources_real.launch.py — it MUST publish the same output topics listed below.

Nodes (all sim-only or agnostic plumbing):
  - ardubridge_node   : BiguaSim <-> ArduPilot SITL (PWM in / state out, JSON FDM)   [SIM-ONLY]
  - ArduPilot SITL + micro-ROS agent + MAVProxy (sitl_dds_udp.launch.py)             [SIM-ONLY]
  - zed_mimic_node    : /biguasim/* -> /zed/zed_node/* (stand-in for zed_wrapper)     [SIM-ONLY]
  - visual_odometry_node : ZED RGB-D -> /zed/zed_node/odom (stands in for the ZED     [SIM-ONLY]
                           SDK's native VIO, which zed_wrapper provides on real)
  - mavros_node       : SITL MAVLink <-> /mavros/* (fcu_url = SITL via MAVProxy)      [agnostic role, sim fcu_url]
  - vision_odom_bridge: /zed/zed_node/odom -> /mavros/vision_pose/pose                [AGNOSTIC — also on real]
  - rangefinder_bridge: BiguaSim RangeFinderSensor -> /mavros/distance_sensor/*       [SIM-ONLY shim]

OUTPUT CONTRACT (must match sources_real.launch.py 1:1):
  /zed/zed_node/rgb/image_rect_color, /zed/zed_node/rgb/camera_info,
  /zed/zed_node/depth/depth_registered, /zed/zed_node/depth/camera_info,
  /zed/zed_node/imu/data, /zed/zed_node/odom,
  /mavros/*  (incl. /mavros/vision_pose/pose, /mavros/distance_sensor/rangefinder)
  ( /zed/zed_node/odom_GT is a SIM-ONLY debug extra — ground truth — NOT part of
    the contract the autonomy stack consumes. )

Notes:
  - No `refs:=dds_xrce_profile.xml`: this ArduPilot creates DDS entities client-side.
  - micro_ros_agent is included by sitl_dds_udp.launch.py (don't launch a 2nd).
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
            # TF owner is visual_odometry_node; keep mimic silent (Phase 1).
            'publish_tf':  False,
        }],
    )

    # Real visual odometry on the ZED RGB-D stream (ORB features -> depth
    # back-projection -> PnP/RANSAC), the honest analogue of the ZED 2i's
    # stereo-VO core. SIM-ONLY: on the real drone zed_wrapper (ZED SDK) publishes
    # /zed/zed_node/odom natively, so this node is not launched there.
    visual_odometry = Node(
        package='hydrone_bringup',
        executable='visual_odometry_node',
        output='screen',
    )

    # MAVROS <-> ArduPilot SITL. The SITL params (holybro_sitl.parm) put EKF3 on
    # external nav with GPS disabled (CBR rules), so the vehicle can only hold
    # position if it is fed vision. mavros connects to MAVProxy's udp:14551 output;
    # apm_pluginlists.yaml is what makes the vision_pose plugin load.
    mavros_share = get_package_share_directory("mavros")
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[
            os.path.join(mavros_share, "launch", "apm_pluginlists.yaml"),
            os.path.join(mavros_share, "launch", "apm_config.yaml"),
            # SIM-ONLY: distance_sensor plugin in SUBSCRIBER mode, so MAVROS reads
            # the Range from rangefinder_bridge and sends DISTANCE_SENSOR to SITL.
            # On real, the plugin PUBLISHES /mavros/distance_sensor/* from the
            # VL53L1X that ArduPilot read natively — no subscriber config needed.
            os.path.join(bringup_pkg, "config", "mavros_distance_sensor.yaml"),
            {
                "fcu_url": "udp://:14551@",   # SITL via MAVProxy (sim fcu_url)
                "gcs_url": "",
                "tgt_system": 1,
                "tgt_component": 1,
                "fcu_protocol": "v2.0",
            },
        ],
    )

    # AGNOSTIC plumbing (runs on sim AND real): /zed/zed_node/odom ->
    # /mavros/vision_pose/pose (VISION_POSITION_ESTIMATE). Consumes the agnostic
    # /zed odom, produces the agnostic /mavros pose — no sim assumption.
    vision_odom = Node(
        package="hydrone_bringup",
        executable="vision_odom_bridge",
        output="screen",
    )

    # Down-facing rangefinder — SIM-ONLY shim (analogous to zed_mimic).
    # On the REAL drone the rangefinder is a VL53L1X wired over I2C straight into
    # the Pixhawk; ArduPilot reads it natively and the data leaves via MAVROS as
    # /mavros/distance_sensor/*. So there is NO rangefinder_bridge on the real
    # drone — this node only exists to inject into SITL what I2C injects on real:
    #   BiguaSim RangeFinderSensor (LaserScan) -> Range -> MAVROS distance_sensor
    #   -> DISTANCE_SENSOR -> ArduPilot RNGFND1 (landing/flare Z ref, not EKF Z).
    # It legitimately consumes /biguasim/* (that is its place); the agnostic
    # contract is its OUTPUT (/mavros/distance_sensor/*).
    rangefinder = Node(
        package="hydrone_bringup",
        executable="rangefinder_bridge",
        output="screen",
        parameters=[{'in_scan': f'{prefix}/RangeFinderSensor'}],
    )

    return LaunchDescription([
        ardubridge,
        sitl_dds,
        zed_mimic,
        visual_odometry,
        mavros,
        vision_odom,
        rangefinder,
    ])
