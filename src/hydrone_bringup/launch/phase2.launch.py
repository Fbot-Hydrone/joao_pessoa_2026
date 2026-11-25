"""
hydrone_bringup/launch/phase2.launch.py — Fase 2 (transporte de pacotes).

AUTONOMY ONLY, world-agnostic like phase1.launch.py: it consumes /down_cam/*,
/zed/zed_node/*, /mavros/* and /hydrone/gripper/command, and never knows
whether a simulator or the drone produces them. phase2_sim.launch.py and
phase2_real.launch.py put the sources under it.

  hydrone_controller   the only node that commands the FCU
  hydrone_nav          NavigateTo: A* over the octomap
  cloud_filter+octomap the occupancy map — built in the `map` frame here, the
                       frame the Navigator queries (in phase 1 it is `odom`,
                       which the Navigator does not transform; see the memory
                       'Octomap está em odom, não em map')
  map_odom             map -> odom TF
  pad_detector_down    the BASES (YOLO base_pouso) — delivery descent, and
                       their positions (rangefinder as depth) for pad_map
  pad_map              the bases found, for the scan (Phase 1's map)
  kit_detector         the KITS (YOLO 'lipo') — pickup descent + verify
  kit_axis             which way each kit lies — the turn before the grab
  ring_detector        the base's yellow RING — delivery fallback (kit hides it)
  phase2_mission_node  the strategy
  ros2 bag             record:=true, with the task log in the same folder

Positions: pickup_1..3 / delivery_1..3 (arena frame, "" = layout_file's
entry, "scan" = unknown), else hydrone_bringup/config/phase2_bases.yaml. Any
unknown base -> the mission scans the arena first.
"""
import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, \
    OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

MODELS = PathJoinSubstitution([FindPackageShare("hydrone_vision"), "models"])

BASES = [f"pickup_{i}" for i in (1, 2, 3)] + [f"delivery_{i}" for i in (1, 2, 3)]

# What the bag keeps. No raw images by default: 10 min of the belly camera is
# ~8 GB. record_images:=true adds it.
BAG_TOPICS = [
    "/mavros/state", "/mavros/local_position/pose",
    "/mavros/local_position/velocity_local", "/mavros/statustext/recv",
    "/mavros/distance_sensor/rangefinder", "/hydrone/mission/status",
    "/hydrone/controller/status", "/hydrone/pads/down/detections",
    "/hydrone/pads/down/ring", "/hydrone/pads/map", "/hydrone/kits/detections",
    "/hydrone/kits/axis", "/down_cam/camera_info", "/tf", "/tf_static",
    "/clock", "/rosout",
]


def _f(context, name):
    return yaml.safe_load(LaunchConfiguration(name).perform(context))


def _boxes(context):
    """plan_bounds / pad_map's arena_bounds: from arena_walls_m when set
    (the real arena, takeoff base in a corner), else the sim's boxes."""
    from hydrone_nav import arena
    plan = [float(v) for v in _f(context, "plan_bounds")]
    boxes = arena.arena_boxes(_f(context, "arena_walls_m"),
                              plan_z=(plan[2], plan[5]))
    if boxes is None:
        return plan, [plan[0] + 0.5, plan[1] + 0.5, plan[3] - 0.5, plan[4] - 0.5]
    return boxes


def _run_dir(context):
    """One folder per run for the task log and the bag."""
    base = os.path.expanduser(LaunchConfiguration("log_dir").perform(context))
    d = os.path.join(base, time.strftime("phase2_%Y%m%d_%H%M%S"))
    os.makedirs(d, exist_ok=True)
    return d


def _planned(context):
    plan, reject = _boxes(context)
    run = _run_dir(context)
    ground_z = float(_f(context, "ground_z"))

    nav = Node(package="hydrone_nav", executable="nav_node", name="hydrone_nav",
               output="screen", parameters=[{"plan_bounds": plan}])

    pad_map = Node(
        package="hydrone_map", executable="pad_map_node", name="pad_map",
        output="screen",
        parameters=[{
            "range_topic": LaunchConfiguration("range_topic"),
            "down_detections_topic": "/hydrone/pads/down/detections",
            # phase1.launch.py's values: the scan flies lanes at cruise speed
            "max_map_speed": 2.0,
            "max_map_yaw_rate_deg": 60.0,
            "require_armed": True,
            "provisional_ttl_s": 120.0,
            "arena_bounds": reject,
            "takeoff_base_radius": 1.5,
        }])

    params = {
        "layout_file": LaunchConfiguration("layout_file"),
        "arena_yaw_deg": float(_f(context, "arena_yaw_deg")),
        "ground_z": ground_z,
        "cruise_alt": float(_f(context, "cruise_alt")),
        "mission_budget_s": float(_f(context, "mission_budget_s")),
        "belly_offset_xy": [float(v) for v in _f(context, "belly_offset_xy")],
        "range_topic": LaunchConfiguration("range_topic"),
        "plan_bounds": [float(v) for v in plan],
        "sweep_side": LaunchConfiguration("sweep_side"),
        "align_kit": bool(_f(context, "align_kit")),
        "gripper_axis_deg": float(_f(context, "gripper_axis_deg")),
        "touch_max_off_m": float(_f(context, "touch_max_off_m")),
        "close_in_touch": bool(_f(context, "close_in_touch")),
        "auto_start": bool(_f(context, "auto_start")),
        "task_log": os.path.join(run, "tasks.log"),
    }
    for b in BASES:
        params[b] = LaunchConfiguration(b).perform(context)
    mission = Node(package="hydrone_mission", executable="phase2_mission_node",
                   name="phase2_mission", output="screen", parameters=[params])

    topics = BAG_TOPICS + (["/down_cam/image_raw"]
                           if _f(context, "record_images") else [])
    bag = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", os.path.join(run, "bag")] + topics,
        output="log", condition=IfCondition(LaunchConfiguration("record")))

    return [LogInfo(msg=f"phase2: plan_bounds={plan}, logs in {run}"),
            nav, pad_map, mission, bag]


def generate_launch_description():
    bringup = get_package_share_directory("hydrone_bringup")
    args = [
        DeclareLaunchArgument("layout_file", default_value=os.path.join(
            bringup, "config", "phase2_bases.yaml"),
            description="Posicoes das bases, frame da arena (Figura 7)."),
        *[DeclareLaunchArgument(
            b, default_value="",
            description="Arena frame: '' = layout_file, 'scan' = desconhecida, "
                        "'[x, y]' ou '{grid: [col, lin]}'. Qualquer uma "
                        "desconhecida -> varredura da Fase 1 antes dos kits.")
          for b in BASES],
        DeclareLaunchArgument("arena_yaw_deg", default_value="-90.0",
                              description="Rotacao arena->map. Sim: -90."),
        DeclareLaunchArgument("ground_z", default_value="-0.7",
                              description="Chao da arena no z da FCU. Sim: -0.7."),
        DeclareLaunchArgument("cruise_alt", default_value="3.2",
                              description="Cruzeiro, m acima do CHAO."),
        DeclareLaunchArgument("mission_budget_s", default_value="600.0"),
        # Belly webcam from the vehicle centre, m [x forward, y right]. Real
        # drone: measure the mount. Sim: on the axis.
        DeclareLaunchArgument("belly_offset_xy", default_value="[0.0, 0.0]"),
        DeclareLaunchArgument(
            "plan_bounds", default_value="[-5.0, -5.0, 0.3, 5.0, 5.0, 3.0]",
            description="Caixa do A* e da varredura (sim). Com arena_walls_m "
                        "so o z vale."),
        DeclareLaunchArgument(
            "arena_walls_m", default_value="[0.0]",
            description="[frente, tras, esquerda, direita] da base de "
                        "decolagem as paredes, como em phase1. [0.0] = "
                        "plan_bounds."),
        DeclareLaunchArgument("sweep_side", default_value="right"),
        DeclareLaunchArgument("align_kit", default_value="true",
                              description="Girar o kit para a garra antes de pegar."),
        DeclareLaunchArgument(
            "gripper_axis_deg", default_value="0.0",
            description="Direcao (corpo) em que o kit tem que ficar: 0 = "
                        "frente-tras, os dedos fecham pelos lados."),
        DeclareLaunchArgument(
            "touch_max_off_m", default_value="0.22",
            description="Centro do drone ate o centro da base de coleta: "
                        "0,5 - 0,22 (pe mais longe) - 0,06."),
        DeclareLaunchArgument(
            "close_in_touch", default_value="false",
            description="Fecha a garra no TOUCH (no ar, a touch_range_m) em vez "
                        "de no chao. No real os dedos ficam acima do kit ali."),
        DeclareLaunchArgument(
            "auto_start", default_value="true",
            description="false: espera /hydrone/mission/start (contagem na arena)."),
        DeclareLaunchArgument(
            "yolo_device", default_value="cpu",
            description="cpu ou 0 (CUDA). A Orin tem CUDA (torch 2.8)."),
        DeclareLaunchArgument("range_topic",
                              default_value="/mavros/distance_sensor/rangefinder"),
        DeclareLaunchArgument(
            "fcu_range_topic",
            default_value="/mavros/distance_sensor/rangefinder_fcu",
            description="Leitura em que a FCU voa (RNGFND1), como em phase1. "
                        "Sem surface_offset (PLAIN) nao existe o eco: "
                        "/mavros/distance_sensor/rangefinder."),
        DeclareLaunchArgument("record", default_value="true",
                              description="ros2 bag + log de tarefas em log_dir."),
        DeclareLaunchArgument("record_images", default_value="false"),
        DeclareLaunchArgument("log_dir", default_value="~/hydrone_logs"),
        DeclareLaunchArgument("debug", default_value="false"),
    ]
    ignore = [0.75, 0.0, 1.0, 0.22, 0.0, 0.78, 0.16, 1.0]   # airframe in frame

    controller = Node(package="hydrone_controller",
                      executable="controller_node", name="hydrone_controller",
                      output="screen",
                      parameters=[{"fcu_range_topic":
                                   LaunchConfiguration("fcu_range_topic")}])
    cloud_filter = Node(package="hydrone_map", executable="cloud_filter_node",
                        name="cloud_filter", output="screen")
    octomap = Node(
        package="octomap_server", executable="octomap_server_node",
        name="octomap_server", namespace="octomap", output="screen",
        parameters=[{
            "resolution": 0.15, "frame_id": "map", "base_frame_id": "base_link",
            "point_cloud_min_z": -0.5, "point_cloud_max_z": 3.0,
            "sensor_model.max_range": 12.0, "sensor_model.hit": 0.7,
            "sensor_model.miss": 0.4, "filter_ground_plane": False,
            "filter_speckles": True, "occupancy_min_z": 0.25,
            "occupancy_max_z": 2.5, "latch": True}],
        remappings=[("cloud_in", "/hydrone/map/cloud_filtered")])
    map_odom = Node(package="hydrone_localization", executable="map_odom_node",
                    name="map_odom", output="screen")

    def detector(name, camera, weights, classes, out, imgsz, conf, project):
        return Node(
            package="hydrone_vision", executable="pad_detector_node",
            name=name, output="screen",
            parameters=[{
                "camera": camera,
                "image_topic": "/down_cam/image_raw",
                "camera_info_topic": "/down_cam/camera_info",
                "depth_topic": "",
                "optical_frame": "down_cam_optical_frame",
                # The bases' positions (phase 1's rangefinder-as-depth) feed
                # pad_map for the scan; the kits only need their pixel.
                "project_position": project,
                "range_as_depth": project,
                "range_depth_max_deg": 25.0,
                "range_topic": LaunchConfiguration("range_topic"),
                "ground_z": ParameterValue(LaunchConfiguration("ground_z"),
                                           value_type=float),
                "out_topic": out,
                "publish_debug": True,
                "ignore_regions": ignore,
                "min_confidence": conf,
                "detector_backend": "yolo",
                "yolo_weights_path": PathJoinSubstitution([MODELS, weights]),
                "yolo_target_classes": classes,
                "yolo_imgsz": imgsz,
                "yolo_conf_threshold": conf,
                "yolo_device": LaunchConfiguration("yolo_device"),
            }])

    bases = detector("pad_detector_down", "down", "pad_seg_yolo11.pt",
                     ["base_pouso"], "/hydrone/pads/down/detections", 640, 0.30,
                     True)
    kits = detector("kit_detector", "kit", "lipo_mix_yolo11s.pt",
                    ["lipo"], "/hydrone/kits/detections", 960, 0.50, False)
    kit_axis = Node(package="hydrone_vision", executable="kit_axis_node",
                    name="kit_axis", output="screen")

    # Delivery fallback: the base's yellow ring around the carried kit.
    rings = Node(package="hydrone_vision", executable="ring_detector_node",
                 name="ring_detector", output="screen")

    rqt = Node(package="rqt_image_view", executable="rqt_image_view",
               arguments=["/hydrone/kits/axis_debug"],
               condition=IfCondition(LaunchConfiguration("debug")))

    return LaunchDescription(args + [
        controller, cloud_filter, octomap, map_odom, bases, kits, kit_axis,
        rings, rqt, OpaqueFunction(function=_planned)])

