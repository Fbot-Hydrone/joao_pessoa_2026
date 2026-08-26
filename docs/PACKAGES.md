# The packages, and the line between them

Seven packages. The split is by *what a thing is*, not by which phase happens
to use it — a Phase 2 mission should be able to reuse Phase 1's planning
without importing a mission node.

```
    sensors ──> hydrone_bringup ──> hydrone_localization ──> hydrone_map
                                            │                    │
                                            └──> hydrone_vision ─┘
                                                        │
                                    hydrone_nav <───────┘
                                            │
                                    hydrone_mission
```

| package | what belongs in it |
|---|---|
| `hydrone_bringup` | **Where data comes from**, and every launch file. The sim/real pairs live side by side on purpose: `zed_mimic` ↔ `zed_sdk`, `down_cam_mimic` ↔ `down_cam_usb`. Also `rangefinder_bridge` and the sim-only `odom_error`. |
| `hydrone_localization` | **Where the drone thinks it is.** `visual_odometry` (the real VO), `map_odom` (the `map`→`odom` edge), `vision_odom_bridge` (the pose ArduPilot flies on). SLAM, if it ever earns its place, lands here. |
| `hydrone_map` | **What the drone remembers about the world.** `pad_map` (the pads, and which were landed on) and `feature_map` (the accumulated point cloud). An occupancy map would sit next to them. |
| `hydrone_nav` | **How to get there.** `route` is a plain-Python library — no rclpy, no message imports — so any phase can ask it which pad is next. `nav_node` is the previous generation. |
| `hydrone_vision` | **What the camera sees.** `pad_detector` is a library; `pad_detector_node` is its ROS skin. New detectors (kit markers, gestures, Phase 4 targets) belong beside it, same shape. |
| `hydrone_mission` | **What to do, in what order.** One state machine per phase. |
| `hydrone_controller` | Setpoint plumbing. Not in the Phase 1 path today. |

## Two rules that keep it honest

**Libraries must not import ROS.** `hydrone_nav.route` and
`hydrone_vision.pad_detector` are plain Python that a test can drive with fakes
and a node can wrap. When a rule starts living inside a node's method, it stops
being reusable — that is exactly how `phase1_mission_node` reached 1441 lines,
and why `route` was pulled back out of it.

**Launch files stay in `hydrone_bringup`.** Scattering them per package looks
tidier but breaks the composed ones: `phase1_sim.launch.py` includes bringup,
vision, map, localization and mission, so it needs a package that depends on
all of them. That package is bringup.

## Adding a package

Create it under `src/`, with a `package.xml`. Nothing else needs editing: the
Dockerfile and `scripts/dev_rebuild.sh` both discover packages from
`src/hydrone_*` and `src/biguasim-ros2` rather than from a hand-written list.
The one exception is `docker-compose.dev.yml`, where each bind mount must be
spelled out — YAML has no globs.

## The previous generation

`vision_node`, `mission_node`, `pad_mission_node`, `nav_node` and
`controller_node` (~2500 lines) are not in the `phase1_sim` path. They still
build and still have their launch files. Deleting them is a decision about what
you still want to be able to run, not a refactor.
