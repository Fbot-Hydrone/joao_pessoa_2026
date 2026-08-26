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
  - zed_mimic_node    : /biguasim/* -> /zed/zed_node/* (stand-in for zed_wrapper,      [SIM-ONLY]
                        including the ZED's own point cloud)
  - down_cam_mimic_node : /biguasim/*/DownCamera -> /down_cam/* (stand-in for the     [SIM-ONLY]
                          belly camera's driver). Only launched if config.yaml
                          declares the DownCamera sensor.
  - visual_odometry_node : ZED RGB-D -> visual odometry (stands in for the ZED       [SIM-ONLY]
                           SDK's native VIO, which zed_wrapper provides on real).
                           Whether it FLIES the vehicle or just gets measured is
                           the odom_source argument — see below.
  - mavros_node       : SITL MAVLink <-> /mavros/* (fcu_url = SITL via MAVProxy)      [agnostic role, sim fcu_url]
  - vision_odom_bridge: /zed/zed_node/odom -> /mavros/vision_pose/pose                [AGNOSTIC — also on real]
  - rangefinder_bridge: BiguaSim RangeFinderSensor -> MAVROS, and a mimic of          [SIM-ONLY shim]
                        MAVROS's own /mavros/distance_sensor/rangefinder
  - odom_error_node   : /zed/zed_node/odom vs odom_GT -> VO drift CSV at the repo     [SIM-ONLY debug]
                        root. Measurement only — publishes nothing, so it cannot
                        affect flight. Disable with odom_error:=false.

Launch arguments:
  odom_source:=vo|ground_truth  what the EKF navigates on             (default vo)
                                vo: the real visual odometry flies the vehicle,
                                  as the ZED SDK's VIO will on the drone. KNOWN
                                  ISSUE: on the CompetionMap superflat the
                                  forward camera sees bare ground, so the VO
                                  drifts ~0.4 m / 9 deg in 78 s WITH THE DRONE
                                  STATIONARY and the vehicle flies away on arming.
                                  That is a real defect to fix, not to bypass.
                                ground_truth: BiguaSim dynamics fly the vehicle.
                                  A DEBUGGING TOOL for isolating autonomy bugs
                                  from localization ones — a run on truth proves
                                  nothing about the real drone, which has none.
  odom_error:=true|false        run the VO drift logger            (default true)
  odom_error_print:=true|false  also echo drift to stdout at 1 Hz  (default false)
  odom_error_dir:=<path>        where the CSV goes (default: repo root, /ws in
                                the container — which is NOT bind-mounted, so
                                point this at a mounted path to keep the file)

OUTPUT CONTRACT (must match sources_real.launch.py 1:1):
  /zed/zed_node/rgb/image_rect_color, /zed/zed_node/rgb/camera_info,
  /zed/zed_node/depth/depth_registered, /zed/zed_node/depth/camera_info,
  /zed/zed_node/point_cloud/cloud_registered,
  /zed/zed_node/imu/data, /zed/zed_node/odom,
  /down_cam/image_raw, /down_cam/camera_info,
  /mavros/*  (incl. /mavros/vision_pose/pose, /mavros/distance_sensor/rangefinder)
  TF: base_link -> zed_camera_link -> zed_left_camera_frame
                                   -> zed_left_camera_optical_frame
      base_link -> down_cam_link   -> down_cam_optical_frame

  ( SIM-ONLY debug extras, NOT part of the contract the autonomy stack consumes:
    /zed/zed_node/odom_GT and /zed/zed_node/odom_VO — whichever of ground truth
    and the real VO is not currently flying the vehicle — plus
    /zed/zed_node/pose_GT, the ground-truth position AND quaternion as a plain
    PoseStamped, for answering "is the estimate wrong, or did the airframe
    actually flip?" )

  The autonomy layer above is launched with NO overrides — see
  landing_sites_sim.launch.py. Anything it would otherwise need told apart
  belongs HERE, as a mimic of what the hardware publishes.

Notes:
  - No `refs:=dds_xrce_profile.xml`: this ArduPilot creates DDS entities client-side.
  - micro_ros_agent is included by sitl_dds_udp.launch.py (don't launch a 2nd).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Must match the namespace set in biguasim_main/launch/ardubridge.launch.py.
BIGUASIM_NS = 'biguasim'

# `sensor_name` of the belly camera in biguasim's config.yaml. It is what lets a
# second RGBCamera coexist with the ZED's, and it becomes the topic name.
DOWN_CAM_SENSOR = 'DownCamera'


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


def _biguasim_agent(config_path):
    """Return the first agent's config block from biguasim's config.yaml.

    Single source of truth for everything sim-side below: agent name (-> topic
    prefix) and the sensor blocks (-> camera mount offset, rangefinder range).
    Single-agent only for now; multi-agent would need per-index lookups.
    """
    with open(config_path) as f:
        scenario = _find_biguasim_scenario(yaml.safe_load(f))
    return scenario['agents'][0]


def _biguasim_topic_prefix(agent):
    """Build the ROS topic prefix that ardubridge_node publishes sensors under.

    The agent name comes from config.yaml. BiguaSim's environment appends a
    batch suffix to it -> '<name>-id0', which the ROS bridge renders as
    '<name>_id0'.
    """
    return f"/{BIGUASIM_NS}/{agent['agent_name']}_id0"


def _sensor(agent, sensor_type, sensor_name=None):
    """First matching sensor block, or None if not configured.

    `sensor_name` disambiguates the two RGBCameras: BiguaSim defaults a sensor's
    name to its type, so the forward ZED is 'RGBCamera' and the belly camera is
    the one that sets sensor_name explicitly.
    """
    for s in agent.get('sensors', []):
        if s.get('sensor_type') != sensor_type:
            continue
        name = s.get('sensor_name', sensor_type)
        if sensor_name is None or name == sensor_name:
            return s
    return None


def _camera_offset_xyz(agent, default=(0.14, 0.0, -0.08)):
    """ZED mounting position on the body, from the config.yaml sensor block.

    BiguaSim's body frame is GLU (x forward, y left, z up) — identical to ROS
    base_link (FLU) — so the sensor `location` carries over 1:1 with no sign
    flip. Floats are forced because YAML writes bare integers (e.g. `0`) and a
    mixed int/float list is not a valid double_array parameter override.

    Pinned to sensor_name 'RGBCamera' so adding the down camera (a second
    RGBCamera) cannot silently hand the ZED the wrong mount.
    """
    cam = (_sensor(agent, 'RGBCamera', 'RGBCamera')
           or _sensor(agent, 'DepthCamera'))
    loc = (cam or {}).get('location', default)
    return [float(v) for v in loc]


def _down_cam_block(agent, name=DOWN_CAM_SENSOR):
    """The down-facing camera's sensor block, or None if it is not configured."""
    return _sensor(agent, 'RGBCamera', name)


def _down_cam_offset_xyz(agent, default=(0.0, 0.0, -0.12)):
    loc = (_down_cam_block(agent) or {}).get('location', default)
    return [float(v) for v in loc]


def _down_cam_rpy_deg(agent, default=(0.0, 90.0, 0.0)):
    """Down camera mount rotation, straight out of config.yaml.

    Single source of truth: the SAME numbers aim the simulated camera and build
    the ROS mount TF, so the two cannot drift apart. See the sign convention in
    down_cam_mimic_node's module docstring.
    """
    rot = (_down_cam_block(agent) or {}).get('rotation', default)
    return [float(v) for v in rot]


def odom_wiring(odom_source):
    """Decide who flies the vehicle, from the `odom_source` substitution.

    Returns (mimic_odom, mimic_tf, vo_odom, vo_tf) as launch substitutions.

    Whatever lands on /zed/zed_node/odom becomes VISION_POSITION_ESTIMATE and,
    with GPS off, *is* the EKF's position. So this function decides which
    estimator the aircraft's life depends on, and it must always put exactly one
    node on that topic and exactly one node on the odom->base_link TF — two
    broadcasters of the same transform is a corrupt TF tree, and zero position
    sources is an aircraft that cannot arm.

    Ground truth is opt-in: ONLY the exact string 'ground_truth' selects it, so
    a typo can never hand the vehicle perfect localization behind your back.
    Flying on truth would make a green simulator run prove nothing about the
    real drone, which has none — the polarity is deliberately this way round.

    Module-level and pure so test_odom_source.py can evaluate it directly;
    burying it inside generate_launch_description() would leave the wiring
    untestable, and it is not the kind of thing to leave unpinned.
    """
    is_gt = ["'", odom_source, "' == 'ground_truth'"]
    return (
        PythonExpression(["'/zed/zed_node/odom' if "] + is_gt
                         + [" else '/zed/zed_node/odom_GT'"]),
        PythonExpression(["'true' if "] + is_gt + [" else 'false'"]),
        PythonExpression(["'/zed/zed_node/odom_VO' if "] + is_gt
                         + [" else '/zed/zed_node/odom'"]),
        PythonExpression(["'false' if "] + is_gt + [" else 'true'"]),
    )


def _rangefinder_max_range(agent, default=40.0):
    """Rangefinder max distance in meters, from the config.yaml sensor block."""
    rf = _sensor(agent, 'RangeFinderSensor') or {}
    value = rf.get('configuration', {}).get('LaserMaxDistance', default)
    return float(value)


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
    # topic names and frames, and back-projects its depth into the ZED's own
    # point_cloud/cloud_registered — the camera's product, produced where the
    # camera is. On the real drone, launch zed_wrapper instead.
    # Input topics AND the camera mount offset are derived from biguasim's
    # config.yaml, so editing the agent name or the sensor position there
    # propagates to the bridge and to every node below — one source of truth.
    biguasim_config = os.path.join(
        get_package_share_directory('biguasim_main'), 'config', 'config.yaml')
    agent = _biguasim_agent(biguasim_config)
    prefix = _biguasim_topic_prefix(agent)

    # WHICH ODOMETRY THE VEHICLE ACTUALLY FLIES ON.
    #
    # With GPS disabled, whatever lands on /zed/zed_node/odom becomes
    # VISION_POSITION_ESTIMATE and *is* the EKF's position. Measured on the
    # CompetionMap superflat: visual_odometry_node logged 302 starved frames
    # (4-11 matches against a 12 minimum) because the forward camera is staring
    # at empty ground, and accumulated 0.39 m / 8.6 deg of error in 78 s WHILE
    # THE DRONE HAD NOT MOVED A CENTIMETRE (ground truth 0.000 throughout).
    # Arm on that and GUIDED chases a phantom position error until the vehicle
    # flies away and flips ("Crash: Disarming: AngErr=165>30").
    #
    #   vo (default)  the real visual odometry flies the vehicle, exactly as
    #                 the ZED SDK's VIO will on the drone. This is the only
    #                 setting under which a successful run means anything: fly
    #                 on truth and the simulator stops testing the thing that
    #                 has to work.
    #   ground_truth  BiguaSim dynamics fly the vehicle. A DEBUGGING TOOL, for
    #                 isolating an autonomy bug from a localization one — if a
    #                 behaviour fails under vo and passes under ground_truth,
    #                 the fault is in the estimate, not the behaviour. Never a
    #                 way to declare something working.
    #
    # In both cases the estimator that is NOT flying still runs and is still
    # measured against the other by odom_error_node.
    # The flight topic goes to whichever source is selected; the other one is
    # parked on a side topic. See odom_wiring() above.
    mimic_odom, mimic_tf, vo_odom, vo_tf = odom_wiring(
        LaunchConfiguration('odom_source'))
    # odom_error always compares THE REAL VO against THE TRUTH, whichever
    # topics those happen to be — so the drift number stays honest in both modes.
    err_vo = vo_odom
    err_gt = mimic_odom

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
            # Set by odom_source (see above): this node either flies the
            # vehicle on ground truth, or steps aside to /zed/zed_node/odom_GT
            # and lets the real VO fly it.
            'out_odom':    mimic_odom,
            'publish_tf':  ParameterValue(mimic_tf, value_type=bool),
            # base_link -> zed_camera_link static TF, taken from the camera's
            # `location` in config.yaml (same GLU/FLU convention, no conversion).
            'camera_offset_xyz': _camera_offset_xyz(agent),
        }],
    )

    # Down-facing belly camera — SIM-ONLY shim, same role as zed_mimic. Only
    # launched when config.yaml actually declares the sensor, so commenting the
    # camera out there (to buy back render time) does not leave a node spinning
    # on a topic nobody publishes.
    # On the REAL drone: a USB/CSI camera driver publishes /down_cam/image_raw +
    # /down_cam/camera_info, and a static_transform_publisher supplies
    # base_link -> down_cam_link -> down_cam_optical_frame.
    down_cam_nodes = []
    if _down_cam_block(agent) is not None:
        down_cam_nodes.append(Node(
            package='hydrone_bringup',
            executable='down_cam_mimic_node',
            output='screen',
            parameters=[{
                'in_image': f'{prefix}/{DOWN_CAM_SENSOR}',
                'in_info': f'{prefix}/{DOWN_CAM_SENSOR}/camera_info',
                'camera_offset_xyz': _down_cam_offset_xyz(agent),
                'camera_rpy_deg': _down_cam_rpy_deg(agent),
            }],
        ))

    # Real visual odometry on the ZED RGB-D stream (ORB features -> depth
    # back-projection -> PnP/RANSAC), the honest analogue of the ZED 2i's
    # stereo-VO core. SIM-ONLY: on the real drone zed_wrapper (ZED SDK) publishes
    # /zed/zed_node/odom natively, so this node is not launched there.
    visual_odometry = Node(
        package="hydrone_localization",
        executable='visual_odometry_node',
        output='screen',
        parameters=[{
            'out_odom': vo_odom,
            'publish_tf': ParameterValue(vo_tf, value_type=bool),
        }],
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
            # SIM-ONLY: widen the steady-clock command/link timeouts, which do
            # NOT stretch with the synthetic clock when the sim runs below
            # real-time (MAVROS waits for COMMAND_ACK on a steady-clock futex,
            # so `use_sim_time` cannot fix it). Same file the autonomy layer
            # reads in hydrone.launch.py — one place to tune everything.
            os.path.join(bringup_pkg, "config", "timeouts.yaml"),
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
        package="hydrone_localization",
        executable="vision_odom_bridge",
        output="screen",
        parameters=[os.path.join(bringup_pkg, "config", "timeouts.yaml")],
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
        parameters=[{
            'in_scan': f'{prefix}/RangeFinderSensor',
            # This MAVROS build's distance_sensor subscriber listens on ~/<name>
            # from the /mavros namespace, i.e. /mavros/rangefinder (NOT
            # /mavros/distance_sensor/rangefinder). Align the bridge output to it.
            'out_range': '/mavros/rangefinder',
            # Valid-range ceiling = the sim sensor's LaserMaxDistance, so raising
            # it in config.yaml doesn't silently keep the bridge clipping returns.
            'max_range': _rangefinder_max_range(agent),
        }],
    )

    # SIM-ONLY debug: measures how far visual_odometry_node has drifted from the
    # BiguaSim ground truth on /zed/zed_node/odom_GT and logs it to a timestamped
    # CSV at the repo root. It only SUBSCRIBES — no topics, no TF — so it cannot
    # perturb the flight stack; the cost is one extra process and one file per run.
    # Both streams are anchored on their first synchronized pair before being
    # compared (they do not share an origin), and pairs whose stamps are more than
    # max_dt apart are dropped rather than compared. See odom_error_node.py.
    odom_error = Node(
        package='hydrone_bringup',
        executable='odom_error_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('odom_error')),
        parameters=[{
            # ParameterValue with an explicit type: LaunchConfiguration resolves
            # to the STRING 'false', which would fail the node's bool parameter.
            'print_diff': ParameterValue(
                LaunchConfiguration('odom_error_print'), value_type=bool),
            'log_dir': ParameterValue(
                LaunchConfiguration('odom_error_dir'), value_type=str),
            'in_odom': err_vo,
            'in_odom_gt': err_gt,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_source', default_value='vo',
            description="Which odometry the EKF navigates on. 'vo' (default) "
                        "flies on the real visual odometry, as the drone will. "
                        "'ground_truth' flies on BiguaSim dynamics — a "
                        "debugging tool for telling an autonomy bug apart from "
                        "a localization one, never a way to call something "
                        "working."),
        DeclareLaunchArgument(
            'odom_error', default_value='true',
            description='Run odom_error_node (VO drift vs ground truth -> CSV).'),
        DeclareLaunchArgument(
            'odom_error_print', default_value='false',
            description='Echo the VO drift to stdout at 1 Hz as well as the CSV.'),
        DeclareLaunchArgument(
            'odom_error_dir', default_value='',
            description='Directory for the drift CSV. Empty = repo root.'),
        ardubridge,
        sitl_dds,
        zed_mimic,
        *down_cam_nodes,
        visual_odometry,
        mavros,
        vision_odom,
        rangefinder,
        odom_error,
    ])
