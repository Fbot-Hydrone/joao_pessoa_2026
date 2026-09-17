---
tags: [hydrone, scripts, infra]
---
# Scripts

Back to [[Hydrone]]. What each script in `scripts/` is for. Everything there
is a wrapper around something you would otherwise type wrong. There are three entry points (`docker_up.sh` for the simulator,
`jetson_up.sh` for the drone, `host_setup.sh`+`env.sh` for a native host build)
and the rest either support those or answer one specific question.

**Nothing here is dead.** Three files are one-off or machine-specific rather
than part of a daily loop, and two *routes through* a live script have been
superseded — both are called out below.

| Script | What it does | Status |
|---|---|---|
| `docker_up.sh` | Simulation stack in Docker: BiguaSim ⇄ ArduPilot SITL ⇄ the hydrone stack | **Active** — main sim entry point |
| `dev_rebuild.sh` | `colcon build` inside the already-running sim container | **Active** — companion to `docker_up.sh --dev` |
| `dev_shell.sh` | Shell inside the sim container, with `rviz2` + `rqt_image_view` alongside | **Active** |
| `jetson_up.sh` | The whole real-hardware stack on the drone's Jetson | **Active** — main drone entry point |
| `dds_iface.sh` | Sourced library: pins Fast DDS to one network interface | **Active** — used by the three below |
| `view_remote.sh` | One of the drone's image topics, in a window on the workstation | **Active** |
| `rviz_remote.sh` | `rviz2` on the workstation, looking at the drone's topics | **Active** |
| `view_topic.py` | The cv2-only image viewer the two `*_remote.sh` scripts run | **Active** |
| `env.sh` | Per-terminal environment for the native host build | **Active** (host route) |
| `host_setup.sh` | One-shot native install of the whole host toolchain | **Active**, secondary — Docker is the recommended path |
| `capture_charuco.py` | Captures ChArUco frames on the drone, with coverage binning | **Active** — current calibration route |
| `calibrate_offline.py` | Solves a calibration from those frames, on a fast machine | **Active** — current calibration route |
| `charuco_probe.py` | Works out the board's parameters from what the camera sees | **Active** pre-check, phrased for the superseded GUI |
| `build_pyzed.sh` | Builds the `pyzed` wheel against the image's own SDK and numpy | **Situational** — once per image/SDK/numpy bump |
| `host_perf.sh` | Toggles the CPU power cap that throttles BiguaSim | **Situational** — one specific laptop |
| `depth_probe.py` | Prints per-pixel depth in metres from a depth topic | **Situational** — written against the sim's topics |
| `gen.py` | Renders ChArUco views through a *known* camera matrix | **Orphan** — a one-off ground-truth fixture |

---

## The simulator, on the workstation

### `docker_up.sh`
Brings up `hydrone_sim.launch.py` (or a mission) in Docker. It finds the
BiguaSim repo (`BS_SIM_DIR`, else the usual siblings), allows X access, and
picks the dGPU when the NVIDIA container runtime is registered — otherwise the
iGPU, with a printed note.

Flags: `--dev` bind-mounts `src/` so code edits need no image rebuild (implies
`--no-build`), `--no-build` reuses the current image, `--phase1` /
`--landing-sites` / `--phase4` pick what runs, `--ground-truth` flies the EKF on
BiguaSim ground truth (a debugging aid — a green run on it proves nothing about
the real drone), `--no-odom-print` silences the VO-drift line. Any argument
containing `:=` is forwarded to `ros2 launch`; anything else goes to
`docker compose up`.

`--landing-sites` is the **earlier** mission, kept working but no longer the one
being developed — `--phase1` is. See [[Phase 1 Mission]] and [[Landing Sites]].

`--phase4` is not a mission on the same aircraft: it brings up the **Kopis X8
with a Livox Mid-360** (`phase4_sim.launch.py`), which shares nothing above
SITL and MAVROS with the two above — no ZED, no belly camera, no rangefinder,
no autonomy at all. It publishes the lidar's topics and flies on ground truth
so the vehicle holds still while that pipeline is built, so `--ground-truth`
and `--no-odom-print` do not apply and are not passed to it.

The Mid-360 is a depth camera the bridge spins between simulation steps, which
is gated on the `phase` launch argument (3 and 4 fly the Kopis; 1 and 2 fly the
Holybro, whose DepthCamera is the ZED's and must never be turned). `--world`
cannot spin it — the sensor lives in the world process — and the remote bridge
says so on startup.

### `dev_rebuild.sh`
Rebuilds project packages in the running container, for the changes the
bind-mount cannot make live: `.msg`/`.srv`, `entry_points`/`data_files` in
`setup.py`, and files that did not exist at image-build time. Writes to the
container's writable layer, so it survives `restart` but not `down`.

### `dev_shell.sh`
Turns the terminal into a shell in the sim container, with `rviz2` and
`rqt_image_view` started detached beside it and torn down on exit. It streams a
host-side rviz config in (nothing useful is bind-mounted) and copies the GUI
logs back out to `/tmp/hydrone-dev-shell` — including the exit *signal*, which
is what tells an OOM kill apart from a segfault when rviz dies mid-session.
Overridable: `CONTAINER`, `IMAGE_TOPIC`, `LOG_DIR`, `RVIZ_CONFIG`.

Sim-only: it assumes `joao_pessoa_2026-hydrone-1` and `rqt_image_view`, and the
drone image has neither. The drone equivalent is `view_remote.sh`.

### `host_perf.sh`
`status` / `on` / `off` for the platform profile, governor and EPP. The default
quiet/powersave policy pins this laptop's CPU at ~1.0 GHz and cost the simulator
3.1× (17.0 vs 52.8 tick/s, measured 2026-08-19); the governor never ramps on its
own because BiguaSim is a closed lockstep loop, so low CPU% is the symptom, not
spare headroom. `on` snapshots the current settings to `/run` (tmpfs, wiped by
the same reboot that resets the knobs) and `off` restores exactly those.

Hard-coded to this machine's knobs and observed defaults — it degrades safely
elsewhere (unknown knobs are skipped) but the numbers in its header are not
yours.

---

## The native host build

### `host_setup.sh`
The one-shot for README "Option 2": installs ROS 2 Humble if missing, the apt
and pip layers, clones/locates BiguaSim, `vcs import`s the pinned deps, builds
Micro-XRCE-DDS-Gen, then builds the workspace — third-party sequentially first
(a parallel build starves the IDL generator's JVM), everything else after. Every
step is idempotent.

The oldest file here (last touched 2026-07-02) and the one most exposed to
drift: Docker is the recommended path, so this route gets exercised least. Its
pinned pip list is a snapshot of what the stack needed then.

### `env.sh`
Sourced, not executed, in every terminal on the host route. Strips conda from
`PATH` (an active conda env makes CMake resolve `fmt`/`spdlog` out of
`~/miniconda3`, and hijacks `python3` so rclpy looks for a 3.11 `.so` that does
not exist), sources ROS and the workspace overlay, sets `ROS_DOMAIN_ID=42` to
match `docker-compose.yml`, and puts Micro-XRCE-DDS-Gen on `PATH`. Works in bash
and zsh.

---

## The drone

### `jetson_up.sh`
The Jetson's answer to `docker_up.sh` — a plain `docker run`, because the board
has no `docker compose`. Project packages are bind-mounted over `/ws/src/<pkg>`
onto the image's `--symlink-install` chain, so node code, existing launch files
and existing config YAML are live; the script also recreates launch/config
symlinks itself, so a *new* file no longer needs a rebuild.

Modes: default `--phase1` (`phase1_real.launch.py`), `--dry` (the same mission
with **no command ever reaching the FCU** — it never creates its arm/mode/
takeoff clients or setpoint publisher, and you are the actuator), `--sources`
(cameras + MAVROS only, a hardware check), `--map` (ZED point cloud + coverage
map), `--calibrate`, `--shell`. `--rebuild` runs colcon inside the container
(does not persist — the container is `--rm`); `--build` rebuilds the image, for
Dockerfile-level changes only.

The `--dry` guarantee is about the *stack*, not the vehicle: MAVROS still offers
`/mavros/cmd/arming` and the transmitter never went through MAVROS at all. Set
an arming check the vehicle cannot pass, and take the props off.

### `build_pyzed.sh`
Builds the ZED Python binding inside the drone image and drops a wheel that
`docker/Dockerfile.jetson` installs. Stereolabs' own cp310 wheel is built
against numpy 2.x, which this image cannot have (its apt `cv2` 4.5.4 is built
against numpy 1.x, and the mismatch is silent at import and segfaults on the
first array conversion). It cannot be a Dockerfile step: linking needs
`libcuda.so.1`, which `nvidia-container-runtime` injects at container *start*.

Run it once per image, SDK or numpy change — not part of any loop. The wheel is
gitignored.

---

## Watching the drone from the workstation

### `dds_iface.sh`
Sourced by the three scripts below and by `jetson_up.sh`. Both machines sit on
two networks — shared wifi and a direct 10.10.0.0/24 gigabit cable — and Fast
DDS announces a locator for every interface it can see, so an image stream
(~110 Mbit/s for 640×480 BGR at 15 Hz) happily goes out over the wifi the drone
is flying MAVLink on. It writes a Fast DDS profile with an `interfaceWhiteList`
pinning UDPv4 to one address, and re-declares the SHM transport by hand
(`useBuiltinTransports=false` is what makes the whitelist non-decorative, but it
also drops shared memory, which is how the ZED feeds the detectors in-container).

Modes: `auto` (cable if it is up *and* the peer answers on it, wifi otherwise),
`cable`, `wifi`, `any` (no pinning — the behaviour before the cable existed).

> **Uncommitted:** the `auto` mode and the switch of all four callers to it are
> working-tree changes, not yet committed. `auto` exists because `--cable` on
> one end and `--wifi` on the other fails *silently* — an empty window, exactly
> like a domain-id mismatch — so both ends measure the fact instead of two
> people typing matching flags.

### `view_remote.sh`
Runs the viewer here and lets DDS carry the images, rather than `docker exec`ing
into the drone (exec cannot add the X socket or mounts to a running container).
Takes a topic and the link flags. `ROS_DOMAIN_ID` must match the drone — the
default is **0**, because `jetson_up.sh` leaves it unset, while the simulator's
compose file uses 42.

### `rviz_remote.sh`
`rviz2` here, on the drone's topics. rviz is deliberately not in the drone image:
a Tegra X1 renders it badly and `ssh -X` pushes a whole framebuffer over the
flight link. Markers, TF and pose are tiny — do not add image or point-cloud
displays over wifi (over the cable the bandwidth warning largely lifts; the
point cloud is still a lot for rviz to draw).

### `view_topic.py`
The window `view_remote.sh` actually opens. A stand-in for `rqt_image_view`,
which is not in the drone image — it pulls in most of rqt and Qt for one window,
where this needs only `cv2`, already there for the detectors. Handles `bgr8`,
`rgb8`, `mono8`, and colormaps `32FC1`/`16UC1` depth so it does not just show a
black frame. `q`/`Esc` quit, `s` saves to `/tmp`.

---

## Camera calibration

Full write-up in [[Calibration]]. The current route is
**capture on the drone, solve on a desktop** ([[Calibration#The fast route: capture here, solve there|§3b]]).

### `charuco_probe.py`
Detects the board and reports which dictionary matches and how many squares it
sees, before you spend twenty minutes waving it. `--size` counts **squares**,
not interior corners, and a wrong value produces no error — just samples that
never register, which looks exactly like a camera or lighting problem.

Still the right first step, but its output is phrased as a `cameracalibrator`
command line, and that GUI is the superseded route — read the numbers off it and
pass them to `capture_charuco.py` instead.

### `capture_charuco.py`
Captures frames only: no GUI, no solve, no ROS. Bins each detected view by
position, fill and tilt, and keeps a frame only if its bin is still short —
the GUI's X/Y/Size/Skew bars, printed as a table. `--show` adds a live window
with the tilt in degrees, which the GUI does not show and which is the axis that
silently stays thin. V4L2 capture is exclusive, so `--from-topic` subscribes
instead of opening the device when something else already holds the camera.

### `calibrate_offline.py`
Solves from a directory of frames and prints the numbers in the form
`sources_real.launch.py` wants. Two things it works out rather than trusting
you: the `columns × rows` order (scored by homography residual — corner count
cannot tell 9×11 from 11×9) and the **legacy pattern** flag (OpenCV 4.6 changed
which squares carry markers; the Jetson is on 4.5.4 and a desktop likely on
4.11, and getting it wrong still calibrates, plausibly and wrongly).

**Why this replaced the in-GUI solve:** `cameracalibrator` does both halves in
one process on the machine holding the camera, calling `do_calibration()` from
the mouse callback on the `imshow` thread. Measured 2026-08-22 on the Tegra X1:
26+ minutes and still running, GUI frozen throughout. The same solve on a
desktop i7: 0.9 s. Splitting them also *keeps the images* — a directory of
frames can be re-solved as often as you like.

**`jetson_up.sh --calibrate` still starts the GUI**, and [[Calibration#Run it|Calibration §3]]
still documents it. It works; it is just the slow route.

### `gen.py`
Renders ChArUco views through a known `K` by exact homography (planar target,
pinhole camera, zero distortion — an earlier version pushed points through a
distortion model first, which is not a consistent image of anything and looked
like a solver bug). This is what validated `calibrate_offline.py` against ground
truth: `fx` 560 → 558.91, `cx` 322.0 → 322.10.

No callers, no CLI beyond `python3 scripts/gen.py <outdir> [legacy]`. It did its
job; keep it as the regression fixture or delete it, but it is not part of any
workflow.

---

## Other

### `depth_probe.py`
Prints per-pixel depth in metres from a `32FC1` depth topic — centre pixel,
named pixels, or a grid; `--once --dump` saves the frame as `.npy`. Values are
already metres; do not scale again.

Written against the simulator: the default topic is
`/biguasim/uav0_id0/DepthCamera`, and its header documents the two invalid-pixel
conventions the sim's mimic nodes use (655.04 m vs NaN). `--topic` points it
anywhere, so it works on the real ZED too, but the defaults and the reasoning
are sim-era.
