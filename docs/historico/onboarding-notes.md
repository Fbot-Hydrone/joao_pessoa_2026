# Project Notes — joao_pessoa_2026 (Hydrone ROS2 drone dev environment)

> ⚠️ **OUTDATED — pre-refactor. Read `README.md` instead.**
> These notes predate the `refactor/workspace-cleanup` refactor and describe the
> archived direct ROS2↔BiguaSim path (`hydrone_biguasim`, `sim.launch.py`) and
> tell you to work on `develop_bridge_ros2` — none of that is valid anymore.
> Kept only for historical context. The single source of truth is `README.md`.

> Onboarding notes written while exploring the repo mid-project. Goal: explain
> what this is, the two ways to run it, and the gotchas I found. Treat the
> "Open questions / risks" section as things to confirm with the team.

## TL;DR

- This repo (`joao_pessoa_2026`) is a **ROS 2 Humble workspace** (Python) for the
  RoboCup Brasil **Flying Robot League / CBR 2026** competition ("Hydrone" stack).
- The sibling folder `../bs-drone-competition` is **BiguaSim**, the simulator
  (Unreal Engine 5 based, forked from BYU Holodeck). It provides the physics +
  sensors and a Python package `biguasim` that the ROS nodes import. No real
  drone hardware needed to develop.
- You build with `colcon build` and run with `ros2 launch`. There are **two run
  paths** (see below).

## ⚠️ Branch situation (read this first)

- **Work on `develop_bridge_ros2`. `main` is abandoned/outdated** — the team is
  refining `develop_bridge_ros2` before merging it. This checkout is now on
  `develop_bridge_ros2`.
- `develop_bridge_ros2` was 2 commits ahead of `main`; the meaningful code diff
  (ignoring `build/`, `install/`, `log/`, `.tlog`, `.BIN` noise) is just:
  - `src/biguasim-ros2/biguasim_main/biguasim_main/ardubridge_node.py` (heavily changed)
  - `src/biguasim-ros2/biguasim_main/biguasim_main/interface.py` (1 bugfix — a missing
    `sensor_name` assignment in `create_sensor_list`)
  - `src/biguasim-ros2/biguasim_main/config/config.yaml`
  - `src/biguasim-ros2/biguasim_main/launch/ardubridge.launch.py` (**new file**, only on develop)
  - `src/hydrone_bringup/launch/hydrone_sim.launch.py` (on develop it now includes
    `ardubridge.launch.py` instead of launching `ardubridge_node` directly)
- Switch with: `git switch develop_bridge_ros2`.

## Repo layout (`src/`)

| Package | Role |
|---|---|
| `hydrone_msgs` | Custom msgs/srvs (`LandingBase`, `MissionState`, `HumanGesture`, `QRCode`, `SetPhase.srv`) |
| `hydrone_bringup` | Launch files: `hydrone.launch.py` (the stack), `hydrone_sim.launch.py` (SITL path) |
| `hydrone_vision` | ZED2 + ArUco + MediaPipe + pyzbar perception |
| `hydrone_controller` | "MAVROS bridge" — consumes pose, emits setpoints |
| `hydrone_nav` | Route planning + precision landing |
| `hydrone_mission` | State machine / orchestrator |
| `hydrone_biguasim` | **Topic adapter** between BiguaSim and the Hydrone stack + its own `sim.launch.py` |
| `biguasim-ros2/biguasim_main` | BiguaSim ROS 2 wrapper: `biguasim_node` (direct sim) and `ardubridge_node` (SITL bridge) |
| `biguasim-ros2/biguasim_interfaces` | BiguaSim custom msgs (sonar, DVL...) |
| `ardupilot` | Full ArduPilot source (provides `ardupilot_sitl` launch pkg). Flattened into the repo as a normal folder — there is NO `.gitmodules` (last commit literally: "Transforma tudo em pasta normal e unifica o codigo"). |
| `micro_ros_agent`, `micro_ros_msgs` | micro-ROS / DDS agent used by the SITL path |

`install/` and `build/` are committed but were produced on the original author's
machine (`/home/lh/...`) and are broken here — you must run `colcon build` fresh
(see the build guide below).

## The two ways to run it

### Path A — Direct ROS2 ↔ BiguaSim (no ArduPilot, no SITL)
This is the simpler "ROS2 straight into the simulator" path.

Entry point: **`hydrone_biguasim/launch/sim.launch.py`**
```bash
ros2 launch hydrone_biguasim sim.launch.py phase:=1
ros2 launch hydrone_biguasim sim.launch.py phase:=1 open_hardware:=true
```
It starts, in order (with TimerActions to stagger init):
1. `biguasim_main/biguasim_node` — runs BiguaSim, publishes sensor topics,
   subscribes `<agent>/command_control` (a `Float64MultiArray`). Ticks the sim
   on a timer.
2. `hydrone_biguasim/biguasim_bridge` — **topic adapter**. Translates BiguaSim
   topics ↔ the MAVROS-style topics the Hydrone stack expects:
   - `drone_id0/odom/Odom` → `/mavros/local_position/pose`
   - `drone_id0/odom/IMU`  → `/mavros/imu/data`
   - `drone_id0/camera`    → `/zed2/zed_node/rgb/image_rect_color`
   - `drone_id0/depth`     → `/zed2/zed_node/depth/depth_registered`
   - `/mavros/setpoint_position/local` (PoseStamped) → `drone_id0/command_control` (`[x,y,z,yaw]`)
   - Also publishes a **fake `/mavros/state`** (armed=true, mode=GUIDED) so the
     controller doesn't block waiting for a MAVLink handshake.
3. `hydrone_bringup/hydrone.launch.py` — the full stack (vision, controller,
   nav, mission).

Config used by this path: `hydrone_biguasim/config/biguasim_drone.yaml`
→ agent `drone-id0`, type `DjiMatrice`, world `InfiniteFloor`, package `SkyDive`,
50 Hz. (Note: bridge expects `drone_id0` with underscore; BiguaSim replaces
`-`→`_`, so `drone-id0` becomes `drone_id0`. Consistent.)

### Path B — ArduPilot SITL → bridge → BiguaSim
This runs a real ArduCopter SITL flight stack; the bridge feeds BiguaSim as the
physics/sensor backend and relays back. More realistic (real ArduPilot firmware,
MAVLink/DDS), heavier to set up.

Entry point: **`hydrone_bringup/launch/hydrone_sim.launch.py`**
```bash
ros2 launch hydrone_bringup hydrone_sim.launch.py
```
It starts:
1. `micro_ros_agent` on `udp4 --port 2019` (DDS bridge for ArduPilot's native ROS2).
2. `biguasim_main/ardubridge_node` — the **physics bridge**:
   - Builds a BiguaSim scenario for the ArduPilot vehicle profile.
   - In a background thread: receives **motor PWM** from ArduPilot SITL over UDP
     (JSON model), converts PWM→motor cmds, `env.step()`s BiguaSim, then sends
     the simulated state (`build_json_state`) back to ArduPilot. Also publishes
     BiguaSim sensors to ROS2.
   - **Hardcoded** in the node: `PACKAGE_NAME="Competition"`, `WORLD="CompetionMap"`,
     `GPS_ORIGIN=(33.810313, -118.393867)`.
3. ArduPilot SITL via `ardupilot_sitl/launch/sitl_dds_udp.launch.py` with
   `model:=JSON` (so SITL talks to the bridge as its physics), DDS over udp4,
   `master:=tcp:127.0.0.1:5760`, `sitl:=127.0.0.1:5501`.

Config used by this path: `biguasim-ros2/biguasim_main/config/config.yaml`
→ agent `auv0`, type `HolybroX500`, world `CompetionMap`, package `Competition`,
720 ticks/s. (Different agent/world/package than Path A.)

On `develop_bridge_ros2` there's also a standalone launch for just the bridge:
`biguasim_main/launch/ardubridge.launch.py` (namespaced `biguasim`).

### Docker (Path B, partial)
`init.sh` → `xhost +local:docker` then `docker compose up --build`.
`docker-compose.yml` builds the Dockerfile (`osrf/ros:humble-desktop`, builds
the Micro-XRCE-DDS-Gen, colcon builds the ws) and runs **only the ArduPilot SITL
launch** (`sitl_dds_udp.launch.py`). It does NOT start `ardubridge_node` or the
hydrone stack, so the container alone is incomplete for a full sim — the bridge
+ stack appear to be expected to run separately (likely on host). Confirm intended
Docker workflow with the team.

## How to build and run (start here)

### 0. One-time prerequisites
```bash
# ROS 2 Humble + deps
sudo apt install ros-humble-mavros ros-humble-mavros-extras \
                 ros-humble-cv-bridge ros-humble-image-transport
pip install mediapipe pyzbar opencv-python numpy

# BiguaSim engine package (the ROS nodes do `import biguasim`)
cd ../bs-drone-competition && pip install .   # needs Python >= 3.10
#   (or use its conda env: conda env create -f environment.yml)
```
The first time you run a given config, BiguaSim downloads the UE5 world package
it references (`biguasim.install('SkyDive')` / `biguasim.install('Competition')`).

### 1. Build the ROS 2 workspace
```bash
cd ~/work/competition/joao_pessoa_2026
git switch develop_bridge_ros2          # make sure you're on the active branch
colcon build --symlink-install          # ALWAYS use --symlink-install here
source install/setup.bash               # source in EVERY new terminal
```
> ⚠️ The `install/`/`build/` committed in the repo are from another machine
> (`/home/lh/...`) and are broken here — your first `colcon build` is mandatory.
> `source install/setup.bash` must be run in every new terminal before any
> `ros2 ...` command, and again after every rebuild.

Build a single package faster (after the first full build):
```bash
colcon build --symlink-install --packages-select hydrone_nav
```

### 2. Run — pick ONE of the two paths

**Path A — direct ROS2 ↔ BiguaSim (simpler, no ArduPilot):**
```bash
source install/setup.bash
ros2 launch hydrone_biguasim sim.launch.py phase:=1
#   add open_hardware:=true / use_two_drones:=true as needed
```

**Path B — ArduPilot SITL → bridge → BiguaSim (what develop is refining):**
```bash
source install/setup.bash
ros2 launch hydrone_bringup hydrone_sim.launch.py
```
(On develop this single launch brings up `micro_ros_agent` + `ardubridge.launch.py`
+ ArduPilot SITL together.)

### 3. Start the mission (after the launch is up ~5 s)
```bash
ros2 service call /hydrone/mission/start hydrone_msgs/srv/SetPhase \
  "{phase: 1, open_hardware: false, use_two_drones: false}"
# emergency abort:
ros2 service call /hydrone/mission/abort std_srvs/srv/Trigger "{}"
```

### When do I need to rebuild (`colcon build`)?
- **Edited a Python node's `.py`** (e.g. `*_node.py`) and you built with
  `--symlink-install`: usually **NO rebuild** — the install dir symlinks back to
  source, so just restart the launch/node. (A rebuild is only needed if symlinks
  aren't in place yet.)
- **Added/removed/renamed a file, changed `setup.py`/`package.xml`/entry points,
  or added a new launch/config/`data_files`:** **YES, rebuild** that package
  (`--packages-select <pkg>`), because the install manifest changes.
- **Changed a `.msg`/`.srv` (in `hydrone_msgs` or `biguasim_interfaces`):**
  **YES, rebuild** (interfaces are code-generated), and rebuild anything that
  depends on them.
- **Changed a `config/*.yaml`:** if it's installed via `data_files` and you read
  it from `share/` (most nodes here do), **rebuild** so the copy in `install/`
  updates — OR just point `params_file` at the `src/` path directly while
  iterating.
- **First checkout / switched branches / pulled new commits:** **YES, rebuild**
  (the committed `install/` is stale/foreign — see warning above).
- After ANY rebuild: re-run `source install/setup.bash` in open terminals.

### Monitoring while it runs
```bash
ros2 topic echo /mavros/local_position/pose      # drone pose
ros2 topic echo /hydrone/mission_state           # state + score
ros2 topic echo /hydrone/vision/landing_bases    # detections
ros2 run rqt_image_view rqt_image_view /hydrone/vision/debug_image
ros2 node list ; ros2 topic list                 # sanity check what's alive
```

## Open questions / risks to confirm with the team

1. **Which path is "the" path right now?** The branch name `develop_bridge_ros2`
   and the active changes are all in `ardubridge_node.py` → **Path B (SITL bridge)**
   is the current focus (confirmed: `main` is abandoned, develop is being refined).
2. **Hardcoded param path bug — FIXED.** `holybro.parm` DOES ship with the repo
   (git-tracked at the **repo root**). The launch hardcoded
   `~/Documents/joao_pessoa_2026/holybro.parm` (original author's layout) which
   doesn't exist on this clone (`~/work/competition/joao_pessoa_2026/`). Changed
   `hydrone_sim.launch.py` to resolve the workspace root relative to the launch
   file via `os.path.realpath(__file__)` (realpath needed because
   `--symlink-install` makes the launch reachable through a symlink in `install/`).
   Now machine-independent. Assumes the symlink-install workflow this project
   uses; if you ever switch to `--merge-install`/plain build, prefer shipping the
   parm in the package `share/` + `get_package_share_directory`.

3. **The committed `install/` and `build/` are from another machine and are
   broken here.** The installed launch symlink points to
   `/home/lh/Documents/joao_pessoa_2026/build/...` (user `lh`). So before
   anything runs you MUST rebuild on this machine:
   `colcon build --symlink-install && source install/setup.bash`. This is also
   why the old hardcoded `~/Documents/...` path existed.
4. **Config divergence between paths:** Path A uses `DjiMatrice`/`SkyDive`/
   `InfiniteFloor`; Path B uses `HolybroX500`/`Competition`/`CompetionMap`
   (note the world name is spelled "Competion" — possible typo, but it's
   hardcoded in `ardubridge_node.py` too, so leave as-is unless the UE5 package
   says otherwise).
5. **Tracked build artifacts:** `build/`, `install/`, `log/`, `*.tlog`, `*.BIN`,
   `eeprom.bin`, dump files are committed and dirty. This is why the branch diff
   is ~2500 files / 470k lines of mostly noise. Consider `.gitignore`-ing them
   (don't delete blindly — confirm first).
6. **`ardupilot` is a flattened copy**, not a submodule. Updating ArduPilot will
   be manual. Big folder; the SITL build is the slow part of `colcon build`.
7. The `hydrone_controller` is described as a "MAVROS bridge" but in Path A the
   real MAVROS is faked by `biguasim_bridge`. So in sim it never talks to real
   MAVLink — good to keep in mind when debugging arming/mode issues.

## Next steps (suggested)
- On `develop_bridge_ros2`: `colcon build --symlink-install`, source, try
  **Path B** end to end.
- ~~Fix the `holybro.parm` path in `hydrone_sim.launch.py`~~ — done on
  `develop_bridge_ros2` (now resolved relative to the workspace root instead of
  an absolute home path).
- Decide on gitignoring `build/install/log` to make diffs readable.
