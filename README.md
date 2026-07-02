# Hydrone — CBR Flying Robot League 2026

ROS 2 Humble workspace for the RoboCup Brasil **Flying Robot League** (CBR 2026,
João Pessoa). An ArduCopter (SITL or real Holybro X500) flies missions inside
the **BiguaSim** UE5 simulator, driven by the Hydrone autonomy stack.

```
                 ┌──────────────────────── this repo ────────────────────────┐
                 │                                                            │
 BiguaSim (UE5)  │  ardubridge_node ◀──UDP JSON──▶ ArduPilot SITL             │
 physics/sensors ◀──shared memory──┐               │        │                 │
 (bs-drone-      │                 │               │ DDS    │ MAVLink         │
  competition)   │        ROS 2 sensor topics      ▼        ▼                 │
                 │                 │        micro_ros_agent  MAVProxy         │
                 │                 ▼               │                          │
                 │   hydrone stack: vision · controller · nav · mission      │
                 └────────────────────────────────────────────────────────────┘
```

The **simulation bringup** (`hydrone_sim.launch.py`) is the core of the
project: it starts the BiguaSim⇄ArduPilot physics bridge and ArduPilot SITL
(with its micro-ROS DDS agent and MAVProxy) in one launch.

## Repository layout

| Path | Role |
|---|---|
| `src/hydrone_bringup` | **Entry point.** Launch files + flight parameters (`config/params/`) |
| `src/biguasim-ros2/biguasim_main` | `ardubridge_node` — BiguaSim as ArduPilot's physics/sensor backend |
| `src/biguasim-ros2/biguasim_interfaces` | BiguaSim sensor messages (sonar, DVL, …) |
| `src/hydrone_mission` | Mission state machine (phases 1–4, scoring) |
| `src/hydrone_nav` | Route planning + precision landing |
| `src/hydrone_controller` | Setpoint/offboard control |
| `src/hydrone_vision` | Camera perception: bases (ArUco), gestures (MediaPipe), QR (pyzbar) |
| `src/hydrone_msgs` | Custom messages/services (`MissionState`, `SetPhase`, …) |
| `deps.repos` | Pinned source dependencies (ArduPilot, micro-ROS agent/msgs, XRCE-Gen) |
| `docker/`, `Dockerfile`, `docker-compose.yml` | Reproducible containerized bringup |
| `docs/` | Onboarding notes and historical docs |

Third-party sources (`src/ardupilot`, `src/micro_ros_agent`, `src/micro_ros_msgs`,
`tools/Micro-XRCE-DDS-Gen`) are **not committed** — they are pinned in
`deps.repos` and imported with `vcs`. Project-specific ArduPilot tuning lives in
`src/hydrone_bringup/config/params/` (`holybro_sitl.parm` for SITL,
`holybro_x500_tuned.parm` for the real frame).

> The old direct ROS2↔BiguaSim path (`hydrone_biguasim`, no ArduPilot) was
> archived; it exists only in git history.

## Option 1 — run with Docker (recommended)

Requirements: Docker + compose, an X server, and the simulator repo cloned as a
**sibling** directory (`../bs-competition/bs-drone-competition`).

```bash
./scripts/docker_up.sh
```

That's it — the script allows X access, builds the image (first build compiles
ArduPilot SITL: expect ~20+ min once, cached afterwards) and starts the full
simulation bringup. The container:

- runs `ros2 launch hydrone_bringup hydrone_sim.launch.py`;
- installs the `biguasim` package from the mounted simulator repo on first start;
- reuses worlds already downloaded on the host (`~/.local/share/biguasim`);
- uses host networking, so `ros2 topic list` from the host (same `ROS_DOMAIN_ID`)
  sees everything.

**GPU:** by default the container renders on the integrated GPU (`/dev/dri`
via mesa). To use an NVIDIA dGPU, install the container toolkit once on the
host — `scripts/docker_up.sh` then detects it and enables
`docker-compose.nvidia.yml` automatically:

```bash
sudo apt install nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

(If the package is missing, add NVIDIA's repo:
<https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html>.)

## Option 2 — run on the host

### 1. One-time setup

```bash
# ROS 2 Humble + tools
sudo apt install python3-vcstool python3-pip default-jdk \
                 ros-humble-cv-bridge ros-humble-image-transport libzbar0
pip3 install empy==3.3.4 pexpect pymavlink dronecan future lxml MAVProxy \
             mediapipe pyzbar opencv-python numpy

# Simulator client (from the sibling simulator repo)
pip3 install ../bs-competition/bs-drone-competition

# Pinned source dependencies
vcs import --recursive . < deps.repos

# DDS IDL generator (needed by ArduPilot's build)
( cd tools/Micro-XRCE-DDS-Gen && ./gradlew assemble -x submodulesUpdate )
export MICROXRCEDDSGEN_DIR=$PWD/tools/Micro-XRCE-DDS-Gen
export PATH=$MICROXRCEDDSGEN_DIR/scripts:$PATH
```

### 2. Build

```bash
colcon build --symlink-install
source install/setup.bash        # in every new terminal, after every rebuild
```

### 3. Run

```bash
# Simulation: bridge + SITL + DDS agent + MAVProxy
ros2 launch hydrone_bringup hydrone_sim.launch.py

# Autonomy stack (separate terminal)
ros2 launch hydrone_bringup hydrone.launch.py phase:=1
```

`hydrone.launch.py` arguments: `phase:=1..4`, `open_hardware:=true|false`
(2× score), `use_two_drones:=true|false` (phase 3), `camera_topic`, `depth_topic`.

### 4. Start a mission

```bash
ros2 service call /hydrone/mission/start hydrone_msgs/srv/SetPhase \
  "{phase: 1, open_hardware: false, use_two_drones: false}"

# emergency abort
ros2 service call /hydrone/mission/abort std_srvs/srv/Trigger "{}"
```

## Monitoring

```bash
ros2 topic echo /hydrone/mission_state           # mission state + score
ros2 topic echo /hydrone/vision/landing_bases    # detected bases
ros2 run rqt_image_view rqt_image_view /hydrone/vision/debug_image
ros2 node list                                   # what's alive
```

## Configuration

- **Bridge/sim scenario**: `src/biguasim-ros2/biguasim_main/config/config.yaml`
  (agent `auv0`, `HolybroX500`, world `CompetionMap` from the `Competition`
  package). The world/package names and GPS origin are currently also hardcoded
  in `ardubridge_node.py`.
- **SITL flight params**: `src/hydrone_bringup/config/params/holybro_sitl.parm`.
- **Real-frame tuning** (from the flattened ArduPilot tree, preserved):
  `src/hydrone_bringup/config/params/holybro_x500_tuned.parm`.

## Gotchas

- **Don't pass `refs:=dds_xrce_profile.xml`** to the SITL launch: this
  ArduPilot version creates DDS entities client-side and the profile file no
  longer exists. Passing a missing path kills the micro-ROS agent, which shows
  up as endless `sequence size exceeds remaining buffer` errors.
- The first run of a new scenario downloads the UE5 world package
  (`biguasim.install(...)` → set `BS_WORLDS_URL` if you use the team backend).
- Rebuild (`colcon build --symlink-install`) after changing msgs/srvs,
  `setup.py`, or installed config files; plain `.py` edits are picked up via
  symlink-install.
- Run git on the host, not inside containers (root-owned `.git` files break
  the repo for your user).

## Scoring (CBR 2026 rules)

| Phase | Action | Points |
|------|--------|--------|
| 1 | First visit to a base | +20 |
| 1 | Repeated visit | −5 |
| 2 | Kit picked up | +40 |
| 2 | Kit delivered correctly | +20 |
| 2 | Kit dropped in wrong place | −5 |
| 3 | First visit to a base | +20 |
| 3 | Repeated visit (1 drone / 2 drones) | −5 / −10 |
| 4 | Maze traversed | +50 |
| 4 | Unique QR code detected | +20 |
| All | Autonomous return | ×2 |
| All | Open hardware | ×2 |
