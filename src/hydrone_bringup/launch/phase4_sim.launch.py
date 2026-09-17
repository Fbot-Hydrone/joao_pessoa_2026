"""
hydrone_bringup/launch/phase4_sim.launch.py

PHASE 4 — the Kopis X8 carrying a Livox Mid-360, in simulation:

    ros2 launch hydrone_bringup phase4_sim.launch.py

  ardubridge (BiguaSim <-> SITL)  +  ArduPilot SITL  +  MAVROS
  +  livox_mimic_node             -> /livox/lidar, /livox/imu
  +  vision_odom_bridge           -> ground truth as external nav

SENSORS: ONE. This airframe carries the Mid-360 and nothing else. There is no
ZED, no belly camera and no rangefinder on it, so no node from the Holybro
stack appears here and no /zed topic is published, subscribed or remapped. The
two aircraft share the bridge, SITL and MAVROS — the plumbing every vehicle
needs — and nothing above that. If a change here needs a file from
sources_sim.launch.py, that is the signal to copy the ten lines rather than
couple the two.

NO AUTONOMY. Nothing detects, maps, decides or commands: this is a sources
layer, and its whole job is that the topics a Mid-360 publishes are on the
graph, with a vehicle underneath them that holds position while you look. The
navigation that consumes them is the next piece of work — see the notes at the
bottom of this file.

WHAT FLIES IT, AND WHY THAT IS TEMPORARY
----------------------------------------
BiguaSim's ground-truth pose is fed straight to MAVROS as
VISION_POSITION_ESTIMATE (`vision_odom_bridge`, the same MAVLink-side plumbing
the other aircraft uses — it is not a ZED node; it relays whatever Odometry it
is pointed at). With GPS off in the SITL params, that IS the EKF's position.

So the vehicle knows exactly where it is, for free. That is a SCAFFOLD, not a
result: a run that holds position here says nothing about whether the Mid-360
can localize the drone, because nothing in this launch is trying to. Ground
truth comes out the moment a lidar-inertial odometry lands and takes over the
topic — at which point this line, and only this line, changes.

Nothing owns the odom -> base_link transform yet, deliberately: that transform
belongs to whatever estimates it, and here nothing does. A LIO publishes it as
its first act. Until then RViz has a static base_link -> livox_frame from the
mimic and no vehicle motion in TF; set the Fixed Frame to livox_frame to watch
the cloud.

THE ROTATING CAMERA
-------------------
BiguaSim has no lidar sensor, so the Mid-360 is a depth camera that ardubridge
turns between simulation steps. The bridge does the turning and REPORTS the
angle on `<sensor>/spin`; livox_mimic_node buffers each frame's points at the
yaw that topic gives it and flushes the buffer at 10 Hz, which is how the real
unit behaves. `phase:=4` below is what switches the spin on — the Holybro
flown in phases 1 and 2 has a DepthCamera of its own, the ZED's, and turning
that one would quietly rotate the depth behind zed_mimic and the VO.

Two consequences worth knowing before reading the cloud:

  * `--world` cannot spin. The sensor lives in the world process and the
    protocol has no rotate command, so every frame comes back at yaw 0 and the
    cloud is one fixed wedge. The remote bridge says so on startup.
  * The wedge spacing is whatever the tick and camera rates make it. With
    ticks_per_sec 200 and the camera at 100 Hz it renders every 2 ticks, so
    consecutive frames are 2 spin steps apart — at the default 60 deg/step
    that is 120 deg, three distinct azimuths per revolution rather than six.
    Nothing is WRONG (the angles are measured, not assumed), it is just
    coarser coverage. ticks_per_sec 240 with the camera at 60 Hz gives six
    60-degree wedges and clears the ~3-tick delay a rotate takes to apply.

See livox_mimic_node's docstring for what this sensor is NOT (no 360 x 59
coverage, no non-repetitive pattern, no dropouts).

WHAT IS NOT WIRED
-----------------
/livox/imu. The real unit has an IMU inside it and the driver publishes one,
and a lidar-inertial odometry will not start without it. Two things are in the
way, neither of them here:

  1. config-KopisX8.yaml declares no IMUSensor. Its DynamicsSensor is not a
     substitute: BiguaSim returns that sensor's data in the GLOBAL frame with
     no gravity term, and biguasim_main's DynamicsIMUEncoder reads the angular
     ACCELERATION slot into the message's angular velocity field. A LIO's
     gravity alignment fails on the first sample of either.
  2. Adding an IMUSensor crashes the bridge as it stands: sensor_data_encode.py
     lists IMUSensor in `multi_publisher_sensors` with a 'Bias' suffix but has
     no 'IMUSensorBias' encoder, so the lookup returns None and the bridge
     calls it.

Both are small fixes on the BiguaSim side. When they land, add the sensor to
the scenario file and set `in_imu` below; the node already does the rest.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Namespace ardubridge publishes under; matches biguasim_main's launch files.
BIGUASIM_NS = 'biguasim'

# The scenario file's name for the sensor standing in for the Mid-360. Pinned
# by name so adding a second camera to the agent cannot silently hand the lidar
# the wrong mounting pose.
LIVOX_SENSOR = 'DepthCamera'


def _biguasim_config_path(agent_name):
    """The scenario file for `agent_name`, or a readable error."""
    config_dir = os.path.join(
        get_package_share_directory('biguasim_main'), 'config')
    path = os.path.join(config_dir, f'config-{agent_name}.yaml')
    if not os.path.exists(path):
        raise RuntimeError(
            f"no biguasim config for agent_name:={agent_name} ({path})")
    return path


def _find_scenario(node):
    """Locate the 'biguasim_scenario' block in a parsed config."""
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
    """The first agent's block. Single-agent scenarios only, as everywhere."""
    with open(config_path) as f:
        scenario = _find_scenario(yaml.safe_load(f))
    return scenario['agents'][0]


def _sensor(agent, sensor_type, sensor_name=None):
    """First matching sensor block, or None. Names default to the type."""
    for s in agent.get('sensors', []):
        if s.get('sensor_type') != sensor_type:
            continue
        name = s.get('sensor_name', sensor_type)
        if sensor_name is None or name == sensor_name:
            return s
    return None


def _launch_setup(context, *args, **kwargs):
    agent_name = LaunchConfiguration('agent_name').perform(context)
    agent = _agent_block(_biguasim_config_path(agent_name))

    # BiguaSim appends a batch suffix to the agent name; the bridge renders
    # '<name>-id0' as '<name>_id0'.
    prefix = f"/{BIGUASIM_NS}/{agent['agent_name']}_id0"

    lidar = _sensor(agent, 'DepthCamera', LIVOX_SENSOR)
    if lidar is None:
        raise RuntimeError(
            f"{agent_name} declares no '{LIVOX_SENSOR}' sensor, so there is no "
            "Mid-360 to simulate. Phase 4 is that sensor; add it to the "
            "scenario file or launch a different airframe.")
    # Mounting pose of the unit, straight out of the scenario file — the SAME
    # numbers that place the simulated camera build the ROS transform, so the
    # two cannot drift. BiguaSim's body frame is GLU, identical to base_link's
    # FLU, so `location` carries over with no sign flip.
    mount_xyz = [float(v) for v in lidar.get('location', (0.0, 0.0, 0.0))]
    mount_rpy = [float(v) for v in lidar.get('rotation', (0.0, 0.0, 0.0))]

    # An IMUSensor if the scenario ever declares one; empty disables the
    # republish. See the module docstring for why it cannot be the
    # DynamicsSensor's.
    imu = _sensor(agent, 'IMUSensor')
    in_imu = f"{prefix}/IMUSensor" if imu is not None else ''

    sitl_pkg = get_package_share_directory('ardupilot_sitl')
    bringup_pkg = get_package_share_directory('hydrone_bringup')

    # Local simulator, or one running elsewhere? Keyed off the environment for
    # the same reason sources_sim is: scripts/docker_up.sh --world sets it, and
    # an env var reaches every entry point without any of them forwarding it.
    world_address = os.environ.get('WORLD_ADDRESS', '').strip()
    world_port = os.environ.get('WORLD_PORT', '8770').strip() or '8770'
    biguasim_launch = os.path.join(
        get_package_share_directory('biguasim_main'), 'launch')

    if world_address:
        ardubridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(biguasim_launch, 'remote_ardubridge.launch.py')),
            launch_arguments={
                'world_address': world_address,
                'world_port': world_port,
                'agent_name': agent_name,
                'phase': LaunchConfiguration('phase'),
            }.items(),
        )
    else:
        ardubridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(biguasim_launch, 'ardubridge.launch.py')),
            launch_arguments={
                'agent_name': agent_name,
                # What turns the spin on. See the docstring.
                'phase': LaunchConfiguration('phase'),
            }.items(),
        )

    # ArduPilot SITL flown from BiguaSim's dynamics over the JSON FDM, exactly
    # as the other airframe is. kopis_sitl.parm comes LAST and overlays what
    # this vehicle does not have; see that file.
    sitl_dds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sitl_pkg, 'launch', 'sitl_dds_udp.launch.py')),
        launch_arguments={
            'transport': 'udp4',
            'port': '2019',
            # Synthetic clock: ArduPilot advances time with the FDM, so control
            # stays stable when the sim runs below real time.
            'synthetic_clock': 'True',
            'wipe': 'True',
            'model': 'JSON',
            'speedup': '1',
            'slave': '0',
            'instance': '0',
            'defaults': ','.join([
                os.path.join(bringup_pkg, 'config', 'params',
                             'holybro_sitl.parm'),
                os.path.join(sitl_pkg, 'config', 'default_params',
                             'dds_udp.parm'),
                os.path.join(bringup_pkg, 'config', 'params',
                             'kopis_sitl.parm'),
            ]),
            'sim_address': '127.0.0.1',
            'master': 'tcp:127.0.0.1:5760',
            'sitl': '127.0.0.1:5501',
        }.items(),
    )

    # The Mid-360 itself.
    livox = Node(
        package='hydrone_bringup',
        executable='livox_mimic_node',
        name='livox_mimic',
        output='screen',
        parameters=[{
            'in_depth': f'{prefix}/{LIVOX_SENSOR}',
            'in_depth_info': f'{prefix}/{LIVOX_SENSOR}/camera_info',
            # Where the bridge reports the sensor's actual orientation.
            'in_spin': f'{prefix}/{LIVOX_SENSOR}/spin',
            'in_imu': in_imu,
            'publish_rate_hz': ParameterValue(
                LaunchConfiguration('publish_rate_hz'), value_type=float),
            'stride': ParameterValue(
                LaunchConfiguration('stride'), value_type=int),
            'mount_xyz': mount_xyz,
            'mount_rpy_deg': mount_rpy,
        }],
    )

    # MAVROS <-> SITL. No distance-sensor config: that file puts the plugin in
    # SUBSCRIBER mode for the Holybro's rangefinder, which this airframe does
    # not carry.
    mavros_share = get_package_share_directory('mavros')
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            os.path.join(mavros_share, 'launch', 'apm_pluginlists.yaml'),
            os.path.join(mavros_share, 'launch', 'apm_config.yaml'),
            # Widen the steady-clock command/link timeouts, which do not
            # stretch with the synthetic clock when the sim runs below real
            # time. Shared with the other airframe: it is a fact about the
            # simulator, not about a vehicle.
            os.path.join(bringup_pkg, 'config', 'timeouts.yaml'),
            {
                'fcu_url': 'udp://:14551@',   # SITL via MAVProxy
                'gcs_url': '',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
            },
        ],
    )

    # SCAFFOLD (see the module docstring): BiguaSim's ground-truth pose becomes
    # VISION_POSITION_ESTIMATE, so the EKF has a position while the lidar
    # pipeline is built. The bridge is pointed straight at the bridge's own
    # odometry topic — it is stamped with system time and already labelled
    # odom -> base_link, so nothing needs to republish it.
    ground_truth_nav = Node(
        package='hydrone_bringup',
        executable='vision_odom_bridge',
        name='ground_truth_nav',
        output='screen',
        parameters=[
            os.path.join(bringup_pkg, 'config', 'timeouts.yaml'),
            {'in_odom': f'{prefix}/DynamicsSensor/Odom'},
        ],
    )

    return [ardubridge, sitl_dds, livox, mavros, ground_truth_nav]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_name', default_value='KopisX8',
            description='Which biguasim scenario file to fly: '
                        'config-<agent_name>.yaml. It must declare the '
                        'DepthCamera that stands in for the Mid-360.'),
        DeclareLaunchArgument(
            'phase', default_value='4',
            description='Competition phase, passed to ardubridge. 3 and 4 fly '
                        'this aircraft and spin its depth camera; 1 and 2 fly '
                        'the Holybro and must never spin the ZED. Set 3 when '
                        'flying phase 3 on the same airframe; anything else '
                        'leaves the sensor still, which is a way to look at '
                        'one wedge.'),
        DeclareLaunchArgument(
            'publish_rate_hz', default_value='10.0',
            description="The real unit's publish rate. The buffer is flushed "
                        'this often and starts empty again, so a slower '
                        'simulator makes thinner messages rather than later '
                        'ones.'),
        DeclareLaunchArgument(
            'stride', default_value='1',
            description='Publish every Nth pixel in each direction. 1 keeps '
                        'every return (~199k points a revolution at 182x182); '
                        "3 lands near the real Mid-360's ~20k."),
        OpaqueFunction(function=_launch_setup),
    ])
