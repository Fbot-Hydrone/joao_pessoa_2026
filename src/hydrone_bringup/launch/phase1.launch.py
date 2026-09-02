"""
hydrone_bringup/launch/phase1.launch.py

AUTONOMY layer for the Phase 1 mission: take off, turn on the spot until a
landing base is in the map, fly over it, confirm it on the belly camera, land,
repeat, then come home to the base we started on.

This is an ALTERNATIVE to hydrone.launch.py and to landing_sites.launch.py, not
an addition to either. All three drive the vehicle, and running two of them puts
two nodes on /mavros/setpoint_position/local fighting over the setpoint. Pick
one.

  ros2 launch hydrone_bringup phase1_sim.launch.py     # sim, everything
  ros2 launch hydrone_bringup phase1.launch.py         # autonomy only

Nodes
-----
  pad_detector (forward)  ZED RGB+depth  -> /hydrone/pads/detections
  pad_detector (down)     belly RGB      -> /hydrone/pads/down/detections
  pad_map                 forward dets   -> /hydrone/pads/map + RViz markers
  feature_map             ZED point cloud-> /hydrone/map/cloud + coverage
  cloud_filter+octomap    ZED cloud      -> 3-D occupancy map      [opt-in: octomap:=true]
  map_odom_tf             measured map -> odom (joins TF's two trees)
  phase1_mission          map + MAVROS   -> the flight itself

How this differs from landing_sites.launch.py
---------------------------------------------
The sensing half is identical — same two detectors, same map, same tuning — and
that is deliberate: the detector is the part that is partially validated and it
should not be forked. What changes is above it.

  * `phase1_mission_node` replaces `pad_mission_node`. The old one flies +X in
    steps and lands on whatever the belly camera happens to see; this one never
    translates without a target and searches by turning in place.
  * **Both cameras now feed the decision, in different currencies.** The old
    mission threw away every forward-camera detection. This one takes its leads
    from the MAP — built from the ZED alone — and the belly camera votes yes/no
    on what it finds there.

    The belly camera contributes NO POSITION, and that is the point. It runs
    with `project_position: False` and on its own topic, so pad_map_node never
    sees it. Its old ground-plane cast assumed a flat floor at `ground_z`, which
    the competition's RAISED bases break: from overhead the ray crosses the
    assumed plane past the pad it actually hit. And pad_map weights projections
    by `confidence / max(range, 1)`, so a confirmation hover — hundreds of
    close-range frames — would have outvoted the ZED and rewritten the very map
    entry the drone was flown there on. The ZED is the position estimate for
    everything; the belly camera answers one question, "is a base under me".
    A simpler pipeline has fewer ways to be wrong.
  * `pad_map` maps nothing until the vehicle first arms, and the base the drone
    starts on is REGISTERED rather than detected — see docs/PHASE1-MISSION.md.
  * Altitude is 1 m, not 2.5 m. This is test code and a fall from 1 m is cheap.
    Note that it therefore does NOT clear the 1.5 m structure the landing_sites
    cruise altitude was chosen for: this launch assumes the Phase 1 arena is
    clear, which is the arena being flown.

Like the rest of the autonomy layer this consumes ONLY the agnostic contract
buses (/zed/zed_node/*, /down_cam/*, /mavros/*), so it is identical in sim and
on the real drone. phase1_sim adds the sources that produce those buses from
BiguaSim and passes no overrides.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# The belly camera's private detection topic. Deliberately NOT the
# /hydrone/pads/detections bus pad_map_node fuses: these detections carry no
# position, and the map must not be able to consume one by accident.
DOWN_DETECTIONS = "/hydrone/pads/down/detections"


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "takeoff_alt", default_value="2.5",
            description="Altitude for everything: takeoff, turning, travelling "
                        "and the confirmation hover, m above the top of the "
                        "base the drone starts on. Low on purpose — this is "
                        "test code and a fall from 1 m is cheap. At 1 m the "
                        "REAL belly camera (640x480, measured fx 814.6, so "
                        "about 43 deg horizontal) covers roughly 0.8 x 0.6 m "
                        "of floor — NARROWER than the 90 deg simulated one "
                        "this text used to describe, and narrower than a 1 m "
                        "base. The base will overfill the frame at this "
                        "height; the detector needs the whole ring to score "
                        "well, so if confirmation fails at 1 m that is the "
                        "first thing to raise. There is NO obstacle "
                        "avoidance: raise this only for an arena you know is "
                        "clear at the new height."),
        DeclareLaunchArgument(
            "field_mode", default_value="blue",
            description="Which pad the detector is looking at, and so how it "
                        "is found. 'blue' is BiguaSim's: a bright saturated "
                        "blue field on brown ground, found by hue. "
                        "'dark_blue' is the REAL arena's, which lies on foam "
                        "of its own hue and whose paint the ZED renders GREEN "
                        "and washed out (H 58, S 44) -- found by local "
                        "contrast, with no HSV band involved. Run 'blue' "
                        "against the real pad and the yellow mask comes back "
                        "empty, so no check ever runs and nothing is detected "
                        "or explained. phase1_real.launch.py sets this; see "
                        "docs/LANDING-SITES.md."),
        DeclareLaunchArgument(
            "target_bases", default_value="6",
            description="How many landing sites to visit before returning to "
                        "the takeoff base. The takeoff base is not one of "
                        "them. ONE while the mission has never been flown: the "
                        "first thing worth knowing is whether a single "
                        "find-confirm-land-return cycle closes, and a second "
                        "base only adds a leg on a position estimate that has "
                        "already been through a landing and a takeoff. Raise "
                        "it to 2 (the competition number) once one cycle has "
                        "been watched end to end."),
        DeclareLaunchArgument(
            "u_side_x_m", default_value="6.0",
            description="Length of the U's legs along x, in metres, stated "
                        "outright. 0 derives it from the arena instead: "
                        "leg = arena_size - 2 * survey_inset_m. Set it when "
                        "the sweep should be a particular size for a reason "
                        "the arena dimensions do not express — a smaller "
                        "rectangle in a big hall, or a shape matched to what "
                        "the camera actually reaches. The rectangle is "
                        "centred in the arena either way."),
        DeclareLaunchArgument(
            "u_side_y_m", default_value="6.0",
            description="Length of the U's legs along y. See u_side_x_m. Two "
                        "numbers and not one because the competition arena is "
                        "8 x 8 and the team's own is 5 x 6."),
        DeclareLaunchArgument(
            "survey_inset_m", default_value="1.2",
            description="How far the U is flown inside the arena bounds when "
                        "u_side_* is 0. Far enough not to skim a wall, close "
                        "enough that the camera still reaches the far side."),
        DeclareLaunchArgument(
            "settle_s", default_value="5.0",
            description="Time held stationary after each turn before the map "
                        "is believed, s. Detections taken while yaw is slewing "
                        "are projected through a moving estimate and land in "
                        "the map metres out. Keep this short — it is there to "
                        "let the estimate stop, not to loiter."),
        DeclareLaunchArgument(
            "confirm_detections", default_value="6",
            description="Belly-camera looks above confirm_confidence needed "
                        "before committing to a landing. One frame can be a "
                        "glint on something blue."),
        DeclareLaunchArgument(
            "confirm_confidence", default_value="0.30",
            description="Confidence that counts as a look. Raise it if the "
                        "drone lands on things that are merely blue."),
        DeclareLaunchArgument(
            "confirm_timeout_s", default_value="25.0",
            description="How long to hover over a candidate before declaring "
                        "it is not a landing site, blacklisting it and "
                        "resuming the search. Wall-clock, and BiguaSim runs "
                        "well below real time — see config/timeouts.yaml for "
                        "the same trap."),
        DeclareLaunchArgument(
            "auto_start", default_value="true",
            description="Arm and take off as soon as the FCU is ready. Set "
                        "false to hold until /hydrone/mission/start is "
                        "called."),
        DeclareLaunchArgument(
            "dry_run", default_value="false",
            description="Rehearse the mission with NOTHING SENT TO THE FCU. "
                        "The state machine, the map and the belly camera all "
                        "run for real; a human carries the drone and is the "
                        "actuator, following the >>> instructions the mission "
                        "prints, and every transition still waits on the "
                        "measured pose. In dry run the mission node never even "
                        "creates its arm/mode/takeoff clients or its setpoint "
                        "publisher. Set by phase1_dry.launch.py, which ALSO "
                        "denies MAVROS's actuation plugins — this argument on "
                        "its own leaves /mavros/cmd/arming callable by "
                        "anything else in the graph, so it is not by itself a "
                        "reason to hold the drone."),
        DeclareLaunchArgument(
            "debug_images", default_value="true",
            description="Publish annotated detector views on "
                        "/hydrone/pads/<camera>/debug_image."),
        DeclareLaunchArgument(
            "feature_map", default_value="true",
            description="Run the world/coverage mapper over the ZED's point "
                        "cloud. Pure observer — turn it off to save CPU."),
        DeclareLaunchArgument(
            "octomap", default_value="true",
            description="Run the 3-D occupancy map (cloud_filter_node + "
                        "octomap_server). Read it from /octomap_binary — the "
                        "whole tree, ~3 KB, which is also what fits over a "
                        "radio to the real drone."),
        DeclareLaunchArgument(
            "octomap_free_space", default_value="false",
            description="Also publish /free_cells_vis_array. A DEBUGGING view: "
                        "it is 172 KB per update on a small scene and grows "
                        "with the flight. Turn it on to inspect the map, not "
                        "to fly with it."),
        DeclareLaunchArgument(
            "octomap_hz", default_value="2.0",
            description="How often the cloud is handed to octomap. The camera "
                        "runs at 10 Hz and the drone moves centimetres between "
                        "frames, so 2 Hz maps the same thing for a fifth of the "
                        "CPU and a fifth of the marker traffic."),
        DeclareLaunchArgument(
            "octomap_res", default_value="0.15",
            description="OctoMap leaf size in metres. MEASURED on a 6x6 m "
                        "floor plus a wall: 0.10 -> 111 KB per marker update, "
                        "0.15 -> 52 KB, 0.20 -> 31 KB. 0.15 halves the traffic "
                        "of 0.10 and still leaves ~5 cells across a Phase 4 "
                        "window (0.8 m); 0.20 leaves 4, too coarse to trust a "
                        "330 mm drone through."),
        DeclareLaunchArgument(
            "map_odom_tf", default_value="true",
            description="Publish the measured map -> odom that joins TF's two "
                        "trees. Set false if something else takes over that "
                        "transform — on the real drone, zed_wrapper does "
                        "unless publish_map_tf is false."),
        DeclareLaunchArgument(
            "require_armed", default_value="true",
            description="Map nothing until the vehicle first arms. On the "
                        "ground both cameras are looking at the base the drone "
                        "is standing on, from the grazing angles that detect "
                        "it best; mapping then opens the run with a candidate "
                        "the mission has to fly out and rule out."),
        DeclareLaunchArgument(
            "range_topic", default_value="/mavros/distance_sensor/rangefinder",
            description="Downward rangefinder Range topic, used to measure pad "
                        "heights. The SAME topic in sim and on the drone: "
                        "MAVROS publishes it from the VL53L1X on real, and "
                        "rangefinder_bridge mimics that publication in sim."),
        DeclareLaunchArgument(
            "ground_z", default_value="-0.7",
            description="Height of the arena FLOOR above the takeoff plane, m. "
                        "FORWARD CAMERA ONLY, and only as a fallback: the ZED "
                        "places pads with depth, and casts a ray onto this "
                        "plane just for the pixels where depth came back "
                        "empty. The belly camera no longer projects at all, so "
                        "getting this wrong can no longer bias a confirmation. "
                        "The takeoff plane is the top of the base the drone "
                        "starts on, so if that base is raised this is "
                        "negative. 0.0 is correct while everything else in the "
                        "arena is at ground level."),
    ]

    takeoff_alt = LaunchConfiguration("takeoff_alt")
    debug_images = LaunchConfiguration("debug_images")
    ground_z = LaunchConfiguration("ground_z")

    # HSV thresholds, shared by both detectors. Carried over UNCHANGED from
    # landing_sites.launch.py — this mission changes what is done with
    # detections, never how they are made. The measurement behind these numbers
    # (blue S 37-75, yellow S 38-59 on a lossless /down_cam frame at 3 m hover,
    # 2026-08-18, against a library floor of S >= 110 that admitted zero pixels
    # of either) is written out in full there and in docs/LANDING-SITES.md §3.
    #
    # SIM VALUES, and they apply to field_mode:="blue" ONLY. The real arena
    # runs field_mode:="dark_blue", which uses no HSV band at all — retuning
    # these would not move it. Its knobs are mark_delta / mark_window_frac /
    # real_min_radius_px on pad_detector_node; docs/LANDING-SITES.md 3.
    # MEDIDO 2026-09-01, no simulador, com o drone parado na base de decolagem
    # e depois em voo: a auto-exposicao do mapa move a imagem inteira entre os
    # dois extremos da faixa dinamica.
    #
    #                       V mediana   S do azul   S do amarelo
    #   frames iniciais           3         255          235
    #   em voo                  244          51           28
    #
    # V por um fator de 80, S por 5x. A banda anterior ([95,30,50]/[18,30,90])
    # reprovava nas DUAS pontas: no escuro por V (3 < 50), no estourado por S
    # (28 < 30). Nao existe par (S,V) fixo que cubra os dois regimes — o alvo
    # se move mais que a largura de qualquer banda. E a causa nao e ajustavel
    # daqui: RGBCamera.cpp parseia so TicksPerCapture, e captura com
    # SCS_FinalColorLDR, ou seja herda o tonemap/auto-exposicao do MAPA.
    #
    # O que sobrevive aos dois regimes e o HUE. Entao a cor passa a ser apenas
    # a PROPOSTA, com S e V quase abertos, e quem discrimina sao os testes
    # estruturais de _evaluate — area, solidez, aspecto, fracao de amarelo,
    # concentricidade e a varredura polar — que nao dependem de cor absoluta.
    # MEDIDO 2026-09-01, DEPOIS de consertar a exposicao no RGBCamera.cpp
    # (ManualExposure/ExposureBias no config.yaml). Com a imagem lavada, o pad
    # azul caia para S~50 e estes valores tinham sido baixados para 30 so para
    # continuar admitindo alguma coisa. Com a exposicao correta o pad mede
    # S p50 146-196 -- mas o CHAO da arena tambem e azul saturado, e a 30 a
    # mascara deixou de separar os dois: 47% do quadro virava "azul", os
    # contornos fundiam pad com piso e a cascata reprovava tudo em solidity e
    # aspect (MEDIDO: contours=2, solidity=1, aspect=1).
    #
    # Voltando para perto do default do proprio pad_detector.py, que foi
    # medido numa imagem bem exposta.
    blue_hsv_low = [95, 110, 50]
    yellow_hsv_low = [18, 80, 90]
    field_mode = LaunchConfiguration("field_mode")

    # ── Detectors: one per camera, same algorithm, different geometry ───────
    # The forward ZED sees pads across the arena and has depth to place them
    # with. In this mission it is the IDENTIFIER: everything the search finds,
    # it finds here first — and it is the SOLE source of the map's positions,
    # because it is the only camera that measures range rather than assuming a
    # floor height.
    forward_detector = Node(
        package="hydrone_vision",
        executable="pad_detector_node",
        name="pad_detector_forward",
        output="screen",
        parameters=[{
            "camera": "forward",
            "image_topic": "/zed/zed_node/rgb/image_rect_color",
            "camera_info_topic": "/zed/zed_node/rgb/camera_info",
            "depth_topic": "/zed/zed_node/depth/depth_registered",
            "optical_frame": "zed_left_camera_optical_frame",
            "publish_debug": ParameterValue(debug_images, value_type=bool),
            # THE FORWARD CAMERA'S OWN BLUE BAND, and the V is the whole point.
            #
            # A competition base is a BOX: a bright top face carrying the ring
            # and cross, standing on side walls of the same hue. From 2.5 m the
            # ZED sees both, and at V >= 50 the mask admits both — so the top
            # and the wall come back as ONE contour, in an L, and every check
            # after that measures a shape that is not a pad. MEASURED over 150
            # labelled frames of a real run (404 visible base appearances):
            #
            #     top face      V median 188      side wall  V median  61
            #
            # Cutting at 160 keeps the top and drops the wall. What that alone
            # is worth, same frames, same everything else:
            #
            #     V >= 50    78/404 = 19.3%   solidity killed 22.9%
            #     V >= 160  157/404 = 38.9%   solidity killed  1.5%
            #
            # THE COST, stated because it will matter: this is an ABSOLUTE
            # brightness, and the bases it still misses are the ones in shadow
            # — the 97 appearances that reach no contour measure V p90 median
            # 90. Frame-relative and per-blob versions of the same split were
            # measured too: both reach the same recall and DOUBLE the false
            # positives (15 -> 35), because on a frame with no bright blue
            # their cut slides down and admits floor. If the arena's exposure
            # changes, this number moves, and the debug image is where to see
            # it. docs/LANDING-SITES.md 3.
            "blue_hsv_low": [95, 110, 160],
            "yellow_hsv_low": yellow_hsv_low,
            # Both relaxed for THIS camera only, and only once the footprint
            # above was right — measured with the old merged contour they moved
            # nothing at all (17.4% -> 18.9%), which is what said the contour
            # and not the threshold was wrong. With the top face isolated:
            #
            #     yellow_frac_min 0.02 -> 0.006   38.9% -> 41.1%   (6-8 m: 45 -> 54)
            #     ring_cov_min    0.55 -> 0.35    41.1% -> 42.3%   (4-6 m: 112 -> 113)
            #
            # False positives did not move (15) across both: what holds the
            # line here is the structural sweep and the confidence, not these.
            # A pad at 7 m is a few dozen yellow pixels on a foreshortened top;
            # asking it for the same marking fraction as a pad at hover is
            # asking it to be closer than it is.
            "yellow_frac_min": 0.006,
            "ring_cov_min": 0.35,
            "min_confidence": 0.35,
            "field_mode": field_mode,
            # dark_blue: this camera's answer becomes a WORLD POSITION, so it
            # must not read a pad hanging off the edge of the frame. There are
            # always two readings of a clipped pad -- the arc, whose sweep runs
            # off the image but whose centre is right, and a compact cluster
            # wholly inside the frame whose centre is 60-80 px biased -- and a
            # high min_seen refuses both rather than take the biased one.
            "min_seen": 0.85,
            "ground_z": ParameterValue(ground_z, value_type=float),
        }],
    )

    # The belly camera is the VALIDATOR and NOTHING ELSE: nothing is landed on
    # until it has seen the pad from directly above, where the ring and cross
    # are hundreds of pixels across — and that is the entire contribution.
    #
    # It publishes no position. `project_position: False` because its only
    # route was a cast onto a flat floor at ground_z, and the competition's
    # bases are RAISED: from overhead that ray crosses the assumed plane past
    # the pad it actually hit, by (base height / altitude) x the lateral offset.
    # `out_topic` off the shared bus because pad_map weights by
    # confidence / max(range, 1), so a hover's worth of close-range looks would
    # have outvoted the ZED even when they were right. No ground_z parameter
    # here any more: nothing in this node consumes it once the cast is gone.
    down_detector = Node(
        package="hydrone_vision",
        executable="pad_detector_node",
        name="pad_detector_down",
        output="screen",
        parameters=[{
            "camera": "down",
            "image_topic": "/down_cam/image_raw",
            "camera_info_topic": "/down_cam/camera_info",
            "depth_topic": "",
            "optical_frame": "down_cam_optical_frame",
            # CONFIRMATION ONLY, for now. The rangefinder projection works
            # (measured 0.04-0.20 m against 1 m for the forward camera on an
            # elevated base) but it also let the belly camera CREATE map
            # entries, and a bad one there becomes a landing. While the basics
            # are being settled, the ZED is the only thing that says WHERE a
            # base is, and this camera only says whether one is underneath.
            "project_position": False,
            "range_as_depth": False,
            "range_topic": LaunchConfiguration("range_topic"),
            "ground_z": ParameterValue(ground_z, value_type=float),
            "out_topic": DOWN_DETECTIONS,
            "publish_debug": ParameterValue(debug_images, value_type=bool),
            "blue_hsv_low": blue_hsv_low,
            "yellow_hsv_low": yellow_hsv_low,
            # O detector filtra com min_confidence ANTES de publicar, e a missao
            # testa confirm_confidence depois. Com 0.50 contra 0.40 o gate da
            # missao era letra morta: quem cortava era o detector.
            #
            # MEDIDO 2026-09-01: sobre uma base real, a 0.07 m do centro, a
            # barriga entregou 2 frames em 25 s quando a missao pedia 6 — os
            # demais pontuavam logo abaixo de 0.50 e nunca eram publicados.
            # Alinhado com confirm_confidence; a barreira de seguranca continua
            # sendo confirm_detections frames SEPARADOS acima dela.
            "min_confidence": 0.30,
            # A 1 m pad at the 1.5 m confirmation hover is ~213 px across and
            # its markings 10-20 px wide; 5 px cannot bridge them.
            "close_px": 25,
            "field_mode": field_mode,
            # dark_blue, and the mirror image of the forward camera's setting.
            # At landing height the pad no longer fits in this camera's view --
            # measured on belly footage from 2026-08-23, roughly a third of the
            # frames with a pad in them show only part of it, and in some the
            # centre is outside the image altogether. An arc of the circle plus
            # the cross is enough to answer yes, and the fitted ellipse puts
            # the centre where the pad's centre really is. This camera reports
            # no position, so a centre outside the frame costs nothing.
            "min_seen": 0.30,
            # The drone's own landing legs, as x0,y0,x1,y1 fractions. A dark
            # object with a bright edge on blue foam passes every test in the
            # detector; one scored 0.95 in the footage. MEASURED OFF THAT
            # FOOTAGE -- re-measure if the camera or the legs move, by watching
            # /hydrone/pads/down/debug_image on the ground with rotors stopped.
            "ignore_regions": [0.75, 0.0, 1.0, 0.22,
                               0.0, 0.78, 0.16, 1.0],
        }],
    )

    pad_map = Node(
        package="hydrone_map",
        executable="pad_map_node",
        name="pad_map",
        output="screen",
        parameters=[{
            "range_topic": LaunchConfiguration("range_topic"),
            # Empty: the belly camera is confirmation-only for now, so it
            # publishes no position for the map to fuse.
            "down_detections_topic": "",
            "require_armed": ParameterValue(
                LaunchConfiguration("require_armed"), value_type=bool),
            # The default 20 s is wall-clock, and BiguaSim runs ~5-8x below
            # real time — so a candidate had ~3 FLIGHT-seconds to be re-seen
            # before being dropped as a false positive. It matters more here
            # than it did for the forward run: this mission's whole search is
            # "turn away, turn back", and a base sighted on one heading has to
            # survive in the map until the drone has finished looking at the
            # other seven.
            "provisional_ttl_s": 120.0,
        }],
    )

    # Accumulates the ZED's own point cloud into a persistent voxel map plus a
    # coverage grid. Pure observer; nothing in this mission reads it.
    feature_map = Node(
        package="hydrone_map",
        executable="feature_map_node",
        name="feature_map",
        output="screen",
        condition=IfCondition(LaunchConfiguration("feature_map")),
    )

    # ── 3-D occupancy map (opt-in) ───────────────────────────────────────────
    #
    # Two nodes, and the split matters. cloud_filter_node removes the points
    # that would lie to a ray; octomap_server casts the rays. Pointing
    # octomap_server straight at the camera is the obvious wiring and the wrong
    # one: a flying pixel at 18 m carves free space through the wall at 4.86 m
    # that it actually belongs to. See hydrone_map/cloud_filter_node.py.
    #
    # The cloud stays in the SENSOR's frame — octomap_server finds the ray
    # origin by looking the frame_id up in TF, and map_odom (below) is what
    # makes that lookup reach the world.
    # Everything it publishes lands under /octomap/ (the node's namespace), so
    # the whole 3-D map is one group in rviz2's topic tree instead of six names
    # scattered through the root:
    #
    #   /octomap/octomap_binary            Octomap        the tree, ~3 KB
    #   /octomap/octomap_full              Octomap        tree + probabilities
    #   /octomap/projected_map             OccupancyGrid  2-D projection
    #   /octomap/occupied_cells_vis_array  MarkerArray    cubes  [see below]
    #   /octomap/free_cells_vis_array      MarkerArray    free space [opt-in]
    #   /octomap/octomap_point_cloud_centers  PointCloud2
    #
    # `cloud_in` is remapped absolutely (leading /) so the namespace does not
    # drag the subscription along with the publishers.
    #
    # WHICH ONE TO DISPLAY: the MarkerArrays are rebuilt and republished whole
    # on every insert, and rviz2 redraws from scratch each time — which is what
    # makes the cubes blink while everything else on the bus sits still. Use
    # octomap_rviz_plugins' OccupancyGrid display on
    # /octomap/octomap_binary instead: it is latched, ~3 KB, decoded locally,
    # and there is nothing to redraw between updates.
    cloud_filter = Node(
        package="hydrone_map",
        executable="cloud_filter_node",
        name="cloud_filter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("octomap")),
        parameters=[{
            "process_hz": ParameterValue(
                LaunchConfiguration("octomap_hz"), value_type=float),
        }],
    )

    octomap = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        namespace="octomap",
        output="screen",
        condition=IfCondition(LaunchConfiguration("octomap")),
        parameters=[{
            "resolution": ParameterValue(
                LaunchConfiguration("octomap_res"), value_type=float),
            # The frame the map is built in. `odom` is continuous; `map` steps
            # whenever the EKF corrects, and a stepped frame tears an occupancy
            # map exactly like it tears a cloud. feature_map_node publishes in
            # `odom` for the same reason.
            "frame_id": "odom",
            # Ray origin. Must be the vehicle, not the map: octomap uses it to
            # decide what a ray passed THROUGH.
            "base_frame_id": "base_link",
            # The arena's ceiling is the net at ~2.5 m and its floor is flat.
            # Clamping keeps the sky (and any reflection off the white floor)
            # out of the tree instead of paying to ray-cast it. VERIFIED
            # against `ros2 param list` on octomap_server 2.3.1: the names are
            # point_cloud_min_z / point_cloud_max_z. `pointcloud_*_z` (no
            # underscore) is the name in a lot of older docs and it does NOT
            # exist here — the node accepts it as an undeclared override and
            # ignores it, so the clamp would silently never happen.
            "point_cloud_min_z": -0.5,
            "point_cloud_max_z": 3.0,
            # Same cap as cloud_filter_node's max_depth, for the same reason:
            # past the arena's 11.3 m diagonal there is nothing real to map.
            "sensor_model.max_range": 12.0,
            # Occupancy is a competition-critical judgement, so make it slow to
            # believe and slow to forget: defaults (0.7/0.4) flip a cell on one
            # frame. Phase 4 flies through gaps this map defines.
            "sensor_model.hit": 0.7,
            "sensor_model.miss": 0.4,
            "filter_ground_plane": False,
            # An isolated occupied voxel with no occupied neighbour is noise,
            # and octomap_server leaves it in the tree by default. Harmless to
            # look at and a phantom obstacle to a planner: it makes the drone
            # dodge nothing, and in a confined arena dodging nothing is how a
            # path gets pushed into a wall. On from 2026-08-27, when there
            # started being a planner that reads this map.
            "filter_speckles": True,
            # Height band that /projected_map collapses into 2-D. WITHOUT it
            # the arena floor is projected as obstacle and the whole grid comes
            # back occupied — MEASURED on a 6x6 m floor: 1681 occupied cells
            # and 4 free, which is useless to a planner. Clipped to 0.25-2.5 m
            # the same scene gives 1260 free, 41 occupied (the wall) and 547
            # unknown.
            #
            # 0.25 m is above the floor AND above a landing pad sitting on it:
            # a pad is somewhere to land, not something to avoid. The house
            # (1.5 m) and the walls stay in, which is what must be flown
            # around. 2.5 m is the arena's net.
            "occupancy_min_z": 0.25,
            "occupancy_max_z": 2.5,
            # TRUE, and this is the fix for the display that goes red and
            # empties. With latch False octomap_server publishes ONLY ON
            # CHANGE, so a viewer that connects later — or reconnects after a
            # dropped message — gets nothing at all until the map next changes,
            # and rviz draws an empty, failed display in the meantime. Latched
            # (TRANSIENT_LOCAL) every new subscriber is handed the current map
            # immediately. It costs one retained message per topic.
            "latch": True,
            # The single most expensive thing octomap publishes, and OFF by
            # default here for that reason. MEASURED on a 6x6 m floor plus one
            # wall, resolution 0.10: 172 KB per update of free cells and 111 KB
            # of occupied cells, against 6 KB for the whole tree on
            # /octomap_binary — the marker arrays republish EVERY voxel ever
            # seen on every insert, so they grow for the length of the flight
            # and are what makes rviz stutter and drop them.
            #
            # Turn it on to LOOK at the map (octomap_free_space:=true); leave
            # it off to fly, and read the tree from /octomap_binary, which is
            # what a planner wants anyway and what fits over a radio link.
            "publish_free_space": ParameterValue(
                LaunchConfiguration("octomap_free_space"), value_type=bool),
        }],
        remappings=[("cloud_in", "/hydrone/map/cloud_filtered")],  # absolute: escapes the namespace
    )

    # Joins TF's two disconnected trees, by MEASURING map -> odom rather than
    # assuming identity: map_T_odom = map_T_base . (odom_T_base)^-1. The full
    # argument, including the 2026-08-20 measurement that showed identity to be
    # wrong by 90 degrees, is in landing_sites.launch.py — it is the same node
    # doing the same job here.
    map_odom_tf = Node(
        package="hydrone_localization",
        executable="map_odom_node",
        name="map_odom",
        output="screen",
        condition=IfCondition(LaunchConfiguration("map_odom_tf")),
    )

    mission = Node(
        package="hydrone_mission",
        executable="phase1_mission_node",
        name="phase1_mission",
        output="screen",
        parameters=[{
            "takeoff_alt": ParameterValue(takeoff_alt, value_type=float),
            "target_bases": ParameterValue(
                LaunchConfiguration("target_bases"), value_type=int),
            "settle_s": ParameterValue(
                LaunchConfiguration("settle_s"), value_type=float),
            # The U's geometry. See the arguments for what 0 means.
            "u_side_x_m": ParameterValue(
                LaunchConfiguration("u_side_x_m"), value_type=float),
            "u_side_y_m": ParameterValue(
                LaunchConfiguration("u_side_y_m"), value_type=float),
            "survey_inset_m": ParameterValue(
                LaunchConfiguration("survey_inset_m"), value_type=float),
            "ground_z": ParameterValue(ground_z, value_type=float),
            "confirm_detections": ParameterValue(
                LaunchConfiguration("confirm_detections"), value_type=int),
            "confirm_confidence": ParameterValue(
                LaunchConfiguration("confirm_confidence"), value_type=float),
            "confirm_timeout_s": ParameterValue(
                LaunchConfiguration("confirm_timeout_s"), value_type=float),
            "auto_start": ParameterValue(
                LaunchConfiguration("auto_start"), value_type=bool),
            "dry_run": ParameterValue(
                LaunchConfiguration("dry_run"), value_type=bool),
            # Confirmation comes off the belly camera's own topic. The mission
            # reads only `confidence` from it; there is no position to read.
            "detections_topic": DOWN_DETECTIONS,
        }],
    )

    return LaunchDescription(args + [
        forward_detector,
        down_detector,
        pad_map,
        feature_map,
        cloud_filter,
        octomap,
        map_odom_tf,
        mission,
    ])
