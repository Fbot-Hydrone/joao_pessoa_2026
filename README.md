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
| `src/hydrone_vision` | Camera perception: landing pads (classic CV), bases (ArUco), gestures (MediaPipe), QR (pyzbar) |
| `src/hydrone_msgs` | Custom messages/services (`MissionState`, `SetPhase`, …) |
| `deps.repos` | Pinned source dependencies (ArduPilot, micro-ROS agent/msgs, XRCE-Gen) |
| `docker/`, `Dockerfile`, `docker-compose.yml` | Reproducible containerized bringup |
| `docker-compose.dev.yml` | Dev override: bind-mounts `src/` so code edits need no image rebuild (`docker_up.sh --dev`) |
| `docs/` | Onboarding notes and historical docs. Start with [`LANDING-SITES.md`](docs/LANDING-SITES.md) and [`DEVELOP-PIPELINES.md`](docs/DEVELOP-PIPELINES.md) |

Third-party sources (`src/ardupilot`, `src/micro_ros_agent`, `src/micro_ros_msgs`,
`tools/Micro-XRCE-DDS-Gen`) are **not committed** — they are pinned in
`deps.repos` and imported with `vcs`. Project-specific ArduPilot tuning lives in
`src/hydrone_bringup/config/params/` (`holybro_sitl.parm` for SITL,
`holybro_x500_tuned.parm` for the real frame).

> The old direct ROS2↔BiguaSim path (`hydrone_biguasim`, no ArduPilot) was
> archived; it exists only in git history.

## Option 1 — run with Docker (recommended)

Requirements: Docker + compose, an X server, and the simulator repo cloned
somewhere on the machine (auto-detected in `../bs-competition/` or `../`;
anywhere else, point `BS_SIM_DIR` at it).

```bash
./scripts/docker_up.sh
# sim repo in a custom location:
BS_SIM_DIR=~/Documents/bs-drone-competition ./scripts/docker_up.sh
# autonomous landing-site mission (find a pad, land, take off, keep going):
./scripts/docker_up.sh --landing-sites      # docs/LANDING-SITES.md
```

That's it — the script allows X access, builds the image (first build compiles
ArduPilot SITL: expect ~20+ min once, cached afterwards) and starts the full
simulation bringup. The container:

- runs `ros2 launch hydrone_bringup hydrone_sim.launch.py`;
- installs the `biguasim` package from the mounted simulator repo on first start;
- reuses worlds already downloaded on the host (`~/.local/share/biguasim`);
- uses host networking, so `ros2 topic list` from the host (same `ROS_DOMAIN_ID`)
  sees everything.

### Iterating on ROS code without rebuilding the image

`docker_up.sh` rebuilds the image by default, which is right for a clean run
but far too slow to sit in an edit-run loop. Use `--dev` instead:

```bash
./scripts/docker_up.sh --dev                # bind-mounts ./src, no image build
# edit a node / launch file / config yaml on the host, then:
docker compose restart hydrone
```

`docker-compose.dev.yml` mounts the project packages over `/ws/src/<pkg>`, and
because the image is built with `colcon build --symlink-install` (which chains
`install/` → `build/` → `src/`), your working tree is what the container runs.
Node code, existing launch files and existing config YAML need no build at all.

Four kinds of change *do* need a build, but only an in-container colcon build
(seconds), not an image rebuild:

```bash
./scripts/dev_rebuild.sh --restart                 # all project packages
./scripts/dev_rebuild.sh hydrone_msgs --restart    # or just one
```

- `.msg` changes in `hydrone_msgs` / `biguasim_interfaces`
- new `entry_points` / `data_files` in a `setup.py`
- **new** launch or config files (the symlinks are made per file at build time,
  so a file that didn't exist at build time has nothing pointing at it)
- new `package.xml` dependencies

That build lives in the container's writable layer: it survives
`docker compose stop/start` and `restart`, and is lost on `docker compose down`.
After a `down`, just run `dev_rebuild.sh` once more.

Rebuild the image (`./scripts/docker_up.sh`, no `--dev`) when you change the
`Dockerfile`, `deps.repos`, or want a from-scratch verification — and always
before flying anything for real, so the image and the source agree.

> **Dockerfile layer order.** Everything in the image that doesn't depend on
> this repo (torch, MAVROS, Vulkan — about 1 GB of downloads) sits *above*
> `COPY src/`, so a source edit can't invalidate it. If you add a pip or apt
> dependency, add it above the marked line in the `Dockerfile`, not below it.

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

Ubuntu 22.04 only. One command installs everything (ROS 2 Humble included, if
missing), clones the simulator repo if it isn't found, imports the pinned
dependencies and builds the workspace — safe to re-run if anything fails
midway:

```bash
./scripts/host_setup.sh          # one-time; asks for sudo; ~30 min first run
```

Then, **in every terminal you work in**:

```bash
source scripts/env.sh            # ROS + workspace + build env, conda-safe
```

That single line replaces the manual `source /opt/ros/...`, `source
install/setup.bash` and the `MICROXRCEDDSGEN_DIR` exports — and it strips
conda from the shell (see Gotchas). Rebuild after changing msgs/srvs or
`setup.py`:

```bash
colcon build --symlink-install
```

<details>
<summary>What the script does (manual equivalent)</summary>

1. apt: build tools, `python3-vcstool`, `python3-colcon-common-extensions`,
   `default-jdk`, `ros-humble-cv-bridge`, `ros-humble-image-transport`, `libzbar0`.
2. pip (**system** pip, `--user` — never conda): upgrade `pip setuptools wheel`,
   then `empy==3.3.4 pexpect pymavlink dronecan future lxml MAVProxy mediapipe
   pyzbar opencv-python numpy`, CPU `torch` (explicitly, or pip pulls the CUDA
   multi-GB build), `roma matplotlib`.
3. Clones <https://github.com/HenriqueReichow/bs-drone-competition> next to
   this repo (unless `BS_SIM_DIR` points elsewhere) and `pip install --user -e`'s it.
4. `vcs import --recursive . < deps.repos`
5. `( cd tools/Micro-XRCE-DDS-Gen && ./gradlew assemble -x submodulesUpdate )`
6. `colcon build` — third-party packages first with `--executor sequential`
   (a parallel ardupilot+agent build can starve the IDL generator), then the rest.

</details>

### Run

```bash
# Simulation: bridge + SITL + DDS agent + MAVProxy
ros2 launch hydrone_bringup hydrone_sim.launch.py

# Autonomy stack (separate terminal)
ros2 launch hydrone_bringup hydrone.launch.py phase:=1
```

`hydrone.launch.py` arguments: `phase:=1..4`, `open_hardware:=true|false`
(2× score), `use_two_drones:=true|false` (phase 3), `camera_topic`, `depth_topic`.

**Landing-site mission** — the autonomous find-a-pad / land / take-off / continue
behaviour, sim and autonomy in one command:

```bash
ros2 launch hydrone_bringup landing_sites_sim.launch.py
```

It is an **alternative** to `hydrone.launch.py`, not an addition: both publish
position setpoints and must not run together. Full description, tuning and
limitations in [`docs/LANDING-SITES.md`](docs/LANDING-SITES.md).

### Start a mission

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

Landing-site mission (`landing_sites_sim.launch.py`):

```bash
ros2 topic echo /hydrone/mission/status          # state machine, 1 Hz
ros2 topic echo /hydrone/pads/map                # the pad map (incl. `visited`)
ros2 run rqt_image_view rqt_image_view /hydrone/pads/down/debug_image
# RViz, fixed frame `map`: /hydrone/pads/markers, /hydrone/map/features
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

- **Deactivate conda before building on the host.** With a conda env active
  (including the auto-activated `base`), CMake resolves libraries from
  `~/miniconda3` instead of the system — the micro-ROS agent then fails with
  `fmt`/`spdlog` template errors. Run `conda deactivate` until `which python3`
  says `/usr/bin/python3`, then build in that shell (delete `build/ install/`
  first if a poisoned build already happened).
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
