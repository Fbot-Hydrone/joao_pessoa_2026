---
tags: [hydrone, phase3, mission]
---
# Phase 3 Gesture Mission

Back to [[Hydrone]]. The drone takes off, flies 1 m forward, turns 90° right to face the operator, and from then on flies what the operator's arms say. It uses the Phase 4 airframe and stack (Kopis X8, Mid-360, FAST-LIO into the EKF, GUIDED velocity setpoints; see [[Phase 4 Pipeline]]) plus one forward camera.

## Status (2026-09-30)
**Written and unit-tested, never flown.** Not in the sim, not on the drone. The pure cores have headless tests (`test_gestures.py`, `test_gesture_mission.py`) that fly the whole mission against a point-mass drone. The ROS nodes and launch files have not run yet.

## Pieces
| Piece | What it does |
|---|---|
| `hydrone_vision/gestures.py` | Pure: COCO-17 keypoints → gesture (frl_core's classifier), MediaPipe → COCO mapping, operator position in the image |
| `hydrone_vision/gesture_detector_node.py` | Camera → MediaPipe Pose → `/hydrone/vision/human_gesture` for every frame. Also publishes `/hydrone/gesture/debug_image` |
| `hydrone_mission/gesture/mission.py` | Pure state machine: debounce, the opening legs, the safety limits |
| `hydrone_mission/phase3_gesture_node.py` | Thin ROS wrapper: LIO pose, lidar, gestures and MAVROS |
| `phase3_sim.launch.py` | `phase4_sim` with `config-KopisX8Cam.yaml` (the Kopis plus a `FrontCamera`) and `mission:=gesture` |
| `phase3_real.launch.py` | Raspberry Pi 5: FAST-LIO (`fast_lio_mid360_real.yaml`), MAVROS, the USB camera read directly, `auto_start:=false` |

## Gestures
The arms must be straight. Angles are measured from the hanging arm.

| Arms | Gesture | Drone |
|---|---|---|
| both down | HOVER | holds its position (LIO) |
| both horizontal (T) | STOP | holds its position |
| both up (Y) | SUBIR | climbs at 0.25 m/s (held 1.5 s on the ground: takes off again) |
| both diagonal down (A) | DESCER | descends at 0.25 m/s |
| one horizontal | DIREITA / ESQUERDA | moves sideways at 0.3 m/s toward the side the arm points to |
| one diagonal up, the other down | APROXIMAR | moves toward the operator |
| one up, the other down | AFASTAR | backs away from the operator |
| one up, the other horizontal | POUSAR | lands (hold it 1.2 s) |

A gesture has to be held before it counts. The default is 0.5 s. HOVER and STOP take 0.25 s, APROXIMAR and AFASTAR 0.7 s, POUSAR 1.2 s. APROXIMAR and AFASTAR are slower because forming POUSAR passes through both of them. If nothing is recognised (bent arm, dead zone, nobody in view, camera silent for more than 0.5 s), the drone treats it as HOVER. The judges' terminal output is the `[PROVA3] GESTO: ...` lines.

## Mission
TAKEOFF (1 m) → FORWARD 1 m (P on the LIO position, ≤0.3 m/s) → TURN −90° → FIND → GESTURE → LAND → LANDED.

- **FIND:** holds position and looks for a person. A person seen for 0.8 s moves the mission to GESTURE. After 3 s with nobody in view it sweeps ±45° around the heading. After 60 s with nobody it lands.
- **GESTURE, safety checks every tick:**
  - altitude stays within 0.5–2.0 m above the takeoff ground;
  - the drone stays within 3 m of where the gestures began;
  - no horizontal motion toward anything the Mid-360 sees within 1.2 m along a 0.9 m-wide corridor (the operator counts);
  - no horizontal motion if the lidar is more than 1 s stale;
  - the yaw keeps the operator centred in the image;
  - an operator lost for 4 s sends the mission back to FIND.
- **Pilot override:** a mode change the node didn't request (the RC switched to LOITER) makes the node stop sending anything for good.

All limits are ROS parameters named like the `MissionParams` fields.

## Running it
Sim (the sim has no person to gesture, so type the gestures in):

    scripts/docker_up.sh --phase3
    ros2 topic pub -r 10 /hydrone/gesture/inject std_msgs/String "data: DIREITA"

or from the keyboard, in a second terminal inside the container (`docker compose exec -it hydrone bash`):

    ros2 run hydrone_mission gesture_keys     # w/s/a/d, r/f, t, l, space; q quits

`HumanGesture.msg` gained fields, and there are new entry points. On a `--dev` container that means running `scripts/dev_rebuild.sh` once.

Real drone: start the Livox driver (`ros2 launch livox_ros_driver2 msg_MID360_launch.py`; the repo only carries its messages), then:

    ros2 launch hydrone_bringup phase3_real.launch.py fcu_url:=/dev/ttyAMA0:921600 lidar_mount:="[0.0, 0.0, 0.1]"
    ros2 service call /hydrone/phase3/start std_srvs/srv/Trigger

## Before the first flight
1. **MediaPipe version.** The Dockerfile installs `mediapipe` unpinned, and the node uses `mp.solutions.pose`, which recent releases dropped. Check with `python3 -c "import mediapipe as mp; mp.solutions.pose"`. If it fails, pin `mediapipe<0.10.22` in the Dockerfile: MediaPipe is the only pose model the detector has.
2. **Frame rate on the Pi 5.** The detector logs frames/s every 10 s. It needs ≥10 Hz. `model_complexity:=0` is the lite model.
3. **Props off:**
   - move the drone by hand and check that `/hydrone/lio/odom` follows it;
   - wave at the camera and watch the debug image and the `[PROVA3]` lines;
   - check that `/hydrone/gesture/state` shows the blocks (walk up close with APROXIMAR).
4. **Sign of DIREITA.** Check it with the real camera: image right must be the drone's right. A mirrored UVC camera flips it; fix that with `camera_mirror:=true` (or the camera's own setting).
5. **Frame check.** On the first real flight, check the odom → local ENU rotation the same way [[Phase 4 Maze Mission]] does. The velocity goes out through the same `odom_to_enu`.
