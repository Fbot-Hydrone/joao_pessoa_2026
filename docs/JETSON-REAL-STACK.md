# Running the stack on the drone — legacy Jetson, ZED 1, USB belly camera

How the autonomy stack runs on real hardware: what the hardware is, why the
software combination is awkward, how the image is built, what publishes what,
and everything that is not ready yet.

Companion to [`PHASE1-MISSION.md`](PHASE1-MISSION.md), which documents the
mission itself. **Nothing in the mission changes between sim and drone** — that
is the whole design, and this document is about the layer underneath it.

---

## 1. The hardware

| | what | notes |
|---|---|---|
| computer | Jetson, `t210ref`, **L4T R32.6.1**, Ubuntu 20.04, Python 3.8 | Tegra X1 — Nano/TX1 class, 4 GB. Native ROS is **Noetic** (ROS 1) |
| forward camera | **ZED, first generation**, serial 23876 | USB 3. `2b03:f582` on `/dev/video1` |
| belly camera | **Logitech C270** | `046d:0825` on `/dev/video0`, YUYV 640×480 |
| flight controller | Pixhawk over UART | `/dev/ttyTHS1` |
| CUDA | 10.2.300 | `/usr/local/cuda-10.2` |
| ZED SDK (host) | 4.0.1 | `/usr/local/zed`, `pyzed` for **cp38** |

Deliberately old: the plan is to demonstrate on legacy hardware first so that
better hardware can be justified. Read every performance choice below in that
light.

**The ZED 1 has no IMU.** Its positional tracking is visual-only, not
stereo-inertial. That is a property of the camera, not of anything here, and it
is the single biggest difference from the ZED 2i the code was originally aimed
at.

---

## 2. The version problem, and why it is solvable

ROS 2 Humble requires **Ubuntu 22.04**. The ZED SDK for L4T 32.7 is built for
**Ubuntu 18.04/20.04 with CUDA 10.2**. Those do not obviously go together, and
the first instinct — that they cannot — is wrong.

They reconcile because on Jetson, CUDA is **not in the container**. It is
injected from the host by `nvidia-container-runtime`, driven by
`/etc/nvidia-container-runtime/host-files-for-container.d/cuda.csv`, which says
literally `dir, /usr/local/cuda-10.2`. `libcuda.so.1` arrives the same way, from
`/usr/lib/aarch64-linux-gnu/tegra/`.

Measured, not assumed — `ldd libsl_zed.so` inside the jammy Humble container
with `--runtime nvidia`:

```
libEGL.so.1       => not found
libGLESv2.so.2    => not found
libturbojpeg.so.0 => not found
```

Three libraries, all ordinary jammy apt packages. Everything CUDA-related
resolved. That is what makes the image in §3 possible.

**The image must be run with `--runtime nvidia`.** Without it the injection does
not happen and the SDK fails at `dlopen` with a message about `libcuda` that
reads like a driver fault rather than a missing flag. `entrypoint_jetson.sh`
checks and says so.

### Why SDK 4.0.8 and not the host's 4.0.1

Stereolabs' generic `pyzed-4.0-cp310` wheel — the only cp310 build they publish
— is compiled against the **last** 4.0 patch. Loaded against 4.0.1 it dies:

```
undefined symbol: _ZN2sl6Camera19getRegionOfInterestERNS_3MatENS_10ResolutionE
```

`getRegionOfInterest` was added after 4.0.1. So the SDK in the image is
**4.0.8**, the last 4.0.x for L4T 32.7, and the stock wheel matches.

The installer has to be downloaded **by hand from the Stereolabs website**:
`download.stereolabs.com/zedsdk/4.0/l4t32.7/jetson` and its 4.1/4.2 siblings now
return a 309 KB HTML page rather than the installer, so a plain `curl` in a
Dockerfile cannot fetch it. It is not committed either — 81 MB of binary. Put it
at `docker/ZED_SDK_Tegra_L4T32.7_v4.0.8.zstd.run` before building.

### The fallback, if the patch level ever has to differ

`~/cbr2026/build/build_pyzed.sh` on the Jetson builds the binding **from source**
against the SDK and numpy installed in the image, which removes both version
skews by construction. **The image uses its output**, because Stereolabs' own
cp310 wheel is compiled against numpy 2 and this image needs numpy 1 — see §7.

Five things in that script are non-obvious, each of which failed a build first:

  * tag **`v4.0.8`**, matching the SDK exactly. `master` targets SDK 5.4 and
    aborts outright; plain `v4.0` compiles for 20 minutes and then fails on
    `'BODY_70' is not a member of 'sl::BODY_FORMAT'`.
  * create `/usr/local/cuda` by hand — the runtime injects only the versioned
    directory and `setup.py` looks for the unversioned symlink.
  * rewrite `from distutils.core import setup` to setuptools; distutils has no
    `bdist_wheel` and importing setuptools first does not add one.
  * install `libusb-1.0-0-dev` — without it the long compile succeeds and only
    the final link fails.
  * the checkout is left in the bind-mounted build directory and is **root
    owned**, so a later `rm -rf` as the normal user fails silently and the next
    run rebuilds the stale tag. Remove it from inside a container.

---

## 3. The image

```bash
# on the Jetson, in the repo root, with the .run in docker/
docker build -f docker/Dockerfile.jetson -t hydrone-jetson:humble .
```

`scripts/jetson_up.sh --build` does the same thing and checks the inputs first.

### The two build inputs, and which one is committed

| file | size | in git? | how to get it back |
|---|---|---|---|
| `docker/pyzed-4.0-cp310-cp310-linux_aarch64.whl` | 3.7 MB | **yes** | it is in the repo |
| `docker/ZED_SDK_Tegra_L4T32.7_v4.0.8.zstd.run` | 84 MB | no | download *ZED SDK for JetPack 4.6.X (L4T 32.7) 4.0.8* from [stereolabs.com/developers](https://www.stereolabs.com/developers/release/4.0) |

The wheel is committed even though committing binaries is normally wrong,
because it is the one input that is **not** reproducible from a URL. It is a
from-source Cython build against SDK 4.0.8 and numpy 1.21, and rebuilding it
(`scripts/build_pyzed.sh`) means re-walking the tag hunt — `master` targets SDK
5.4, `v4.0` fails on `'BODY_70' is not a member of 'sl::BODY_FORMAT'`, only the
exact `v4.0.8` tag builds — plus the `setup.py` distutils→setuptools rewrite and
the `libusb-1.0-0-dev` dependency. That is an hour. The SDK installer is a
plain download, so it stays out.

**Neither file is needed to RUN anything.** Both are `COPY`ed into the image at
build time; a container started from an existing image needs neither, and
neither does the dev loop in §4.

Built `FROM ros2-jetson:humble`. What it adds, and why:

| addition | why |
|---|---|
| `libegl1 libgles2 libturbojpeg libusb-1.0-0` | the four the SDK needs and jammy lacks — from `ldd`, §2 |
| `ros-humble-mavros`, `-msgs`, `-extras` | the stack **imports** `mavros_msgs`: `phase1_mission_node` for `/mavros/state` and the arm/mode/takeoff services, `pad_map_node` for the pre-arm mapping gate. Without it both fail at import, which is what the base image did |
| `install_geographiclib_datasets.sh` | not optional for ArduPilot — MAVROS converts AMSL to ellipsoid height with GeographicLib's geoid grid and throws at startup without the dataset |
| ZED SDK 4.0.8, `skip_cuda skip_tools skip_python skip_od_module skip_hub` | CUDA comes from the host; the tools are GUI programs; the binding is installed explicitly below. The installer WARNS and ignores an unknown flag rather than failing, so check the build log for `unknown parameter` |
| a stubbed `/etc/nv_tegra_release` | the installer probes it to decide it is on a Jetson, and it is a host file absent during `docker build` |
| a **locally built** `pyzed-4.0-cp310` wheel, `numpy<2` | Stereolabs' wheel is built against numpy 2 and fails on numpy 1 with `ndarray size changed ... Expected 96, got 88`; numpy 1 is required because the image's cv2 is compiled against it, and that mismatch is silent at import and fatal at first use ([`LANDING-SITES.md`](LANDING-SITES.md) §9). `--no-deps` plus an assertion keeps pip from quietly reinstating numpy 2 |
| the workspace, `--symlink-install` | so the dev bind mount can put the host working tree at the end of the `install/ → build/ → src/` chain, exactly as on x86 |

CUDA is deliberately **not** baked. It must match the host driver, and it
already exists on the host.

---

## 4. Running it

```bash
./scripts/jetson_up.sh                    # the phase 1 mission
./scripts/jetson_up.sh --sources          # cameras + MAVROS only, no flight
./scripts/jetson_up.sh --shell            # a shell inside the image
```

`scripts/jetson_up.sh` is the Jetson's `docker_up.sh`. It is a plain
`docker run` and not compose, because the board has neither `docker compose`
nor `docker-compose` and installing one to launch a single container is not
worth the disk. What it puts on the command line:

| flag | why |
|---|---|
| `--runtime nvidia` | injects the host's CUDA 10.2, `libcuda.so.1` and the Tegra EGL vendor driver. Without it the SDK fails at `dlopen` with something that reads like a driver fault |
| `--network host` | DDS discovery, and reaching a ground station |
| `--ipc host` | DDS shared-memory transport |
| `-v /dev:/dev --privileged` | the ZED needs **both** `/dev/video*` (V4L2 capture) and `/dev/bus/usb` (libusb control transfers), and MAVROS needs `/dev/ttyTHS1`. Per-device flags are possible but USB node numbers move on replug |
| `-it` only when there is a terminal | `docker run -t` aborts with *the input device is not a TTY* under `ssh` without a tty, `nohup`, or a systemd unit — which is how this gets started on a drone nobody is sitting next to |
| `--init` + `exec ros2 launch …` | so that a stop signal actually stops it — see below |
| `--stop-signal SIGINT` | `ros2 launch` shuts its nodes down gracefully on SIGINT; the default SIGTERM-then-SIGKILL is a 10 s wait and a hard kill |

### Why stopping it needs two flags, not one

Getting Ctrl-C or `docker stop` to actually stop the stack took two tries, and
the first one looked right:

1. `bash -lc "ros2 launch …"` makes **bash** PID 1, and bash does not forward
   signals to a child it is waiting on. Adding `exec` fixes that — `ps` inside
   the container then genuinely shows `ros2` as PID 1.
2. That was still not enough. **The kernel does not apply default signal
   actions to PID 1**: a signal with no installed handler is discarded rather
   than killing the process. `ros2 launch` installs a SIGINT handler but not a
   SIGTERM one, so as PID 1 it ignored SIGTERM completely.

Measured on the board, between the two fixes: `timeout 30` around the script
returned on schedule, and both cameras and MAVROS were still running three
minutes later. `--init` puts tini at PID 1 — an ordinary process that forwards
signals to the real one — and the same test now leaves zero containers.

This matters more on a drone than on a desk. The failure mode is a stack that
keeps holding the cameras and the FCU's UART after you think you stopped it,
so the next run fails to open `/dev/ttyTHS1` for reasons that have nothing to
do with the code you just changed.

Raw equivalent, if you want it without the script:

```bash
docker run --rm -it --runtime nvidia --network host --ipc host \
  -v /dev:/dev --privileged \
  hydrone-jetson:humble \
  ros2 launch hydrone_bringup phase1_real.launch.py
```

### The dev loop: why you almost never rebuild

`jetson_up.sh` bind-mounts the six project packages from the checkout over
`/ws/src/<pkg>`, the same trick as `docker-compose.dev.yml` on x86. The image
was built with `colcon build --symlink-install`, which chains

```
install/<pkg>/…/<pkg>  ->  /ws/build/<pkg>  ->  /ws/src/<pkg>/<pkg>
```

so mounting the working tree at the end of that chain makes the code live.
Verified on the board: a comment appended to `phase1_mission_node.py` on the
host was visible through `hydrone_mission.phase1_mission_node.__file__` inside a
container started one second later, with no build of any kind.

| what changed | what it costs |
|---|---|
| node code, an **existing** launch file, an **existing** config YAML | nothing — re-run `jetson_up.sh` |
| `.msg`/`.srv`, `entry_points` in `setup.py`, a **new** launch/config file, a new `package.xml` dep | `jetson_up.sh --rebuild` — colcon inside the container. `--symlink-install` creates one symlink **per file at build time**, so a file that did not exist then has nothing pointing at it |
| a new apt/pip dependency, a new SDK, anything in `Dockerfile.jetson` | `jetson_up.sh --build` |

`--rebuild` does not persist: the container is `--rm`, so its writable layer
goes with it. Pass it on each run until the change earns a `--build`.

A `--build` is not the whole image either. Every expensive layer — apt, MAVROS,
the ZED SDK, pyzed — sits **above** the `COPY src/` line in the Dockerfile, so
a code change invalidates only the colcon layer and those stay cached.

`--no-dev` drops the mounts and runs purely what is baked in the image. That is
what should fly: it is the only mode where the running code cannot be changed
by editing a file on disk.

Useful arguments — all of them inherited straight through the wrapper, see §6:

```bash
./scripts/jetson_up.sh \
    takeoff_alt:=1.0 target_bases:=1 \
    fcu_url:=/dev/ttyTHS1:921600 \
    zed_resolution:=VGA zed_fps:=15 zed_point_cloud:=false \
    down_cam_device:=/dev/video0
```

Anything containing `:=` is passed through to the launch file untouched; every
other argument is a flag of the script, and an unrecognised one is an error
rather than a silently ignored typo.

Cameras only, no flight — the sensible first bench run:

```bash
./scripts/jetson_up.sh --sources
# in a second terminal
docker exec -it hydrone-jetson bash -lc \
  '. /ws/install/setup.sh && ros2 run rqt_image_view rqt_image_view'
```

---

## 5. What produces the contract

| bus | sim | drone |
|---|---|---|
| `/zed/zed_node/rgb/*`, `depth/*`, `point_cloud/*`, `odom` | `zed_mimic_node` + `visual_odometry_node` | **`zed_sdk_node`** |
| `/down_cam/image_raw`, `camera_info`, mount TF | `down_cam_mimic_node` | **`down_cam_usb_node`** |
| `/mavros/*` | MAVROS → SITL via MAVProxy | MAVROS → Pixhawk on `/dev/ttyTHS1` |
| `/mavros/distance_sensor/rangefinder` | `rangefinder_bridge` mimics it | MAVROS **publishes** it from the natively-read VL53L1X |
| `/mavros/vision_pose/pose` | `vision_odom_bridge` | `vision_odom_bridge` — the *same node*, it is agnostic |

`sources_real.launch.py` deliberately omits two config files the sim loads:
`mavros_distance_sensor.yaml` (puts the rangefinder plugin in *subscriber* mode
so the bridge can feed SITL — on the drone the plugin publishes) and
`timeouts.yaml` (widens MAVROS's steady-clock timeouts because BiguaSim runs
below real time; real time does not).

### Why `zed_sdk_node` and not `zed_wrapper`

`zed-ros2-wrapper` is the right answer on hardware that can run it, and the
topic names above are **the wrapper's**, chosen so swapping it in is a one-line
change in `sources_real.launch.py`. It is not usable here: it would have to be
built from source against a 4.0-era SDK inside a jammy container on an L4T 32
host, which is a far larger surface than a few hundred lines of Python doing
exactly the six publications this stack consumes.

If the drone gets a board that runs a supported SDK + Humble pairing, delete
`zed_sdk_node` and launch the wrapper. Two settings it gets wrong for us: the
point cloud must be **on** (`feature_map_node` has no other geometry), and
`pos_tracking.publish_map_tf` must be **false** (it broadcasts `map → odom`
where `map` is the ZED's loop-closed frame; here `map` is the FCU's frame and
`map_odom_node` owns that edge — two broadcasters of one edge meaning two
different things is a corrupt tree).

---

## 6. Arguments: one file owns each default

`phase1_real.launch.py` declares **nothing**. Hardware arguments are declared by
`sources_real.launch.py`, mission arguments by `phase1.launch.py`, and both
reach their file by inheritance — including anything given on the wrapper's
command line.

This is not stylistic. A wrapper that re-declares an argument carries its own
default, and forwarding it overwrites the inner file's, so editing the file that
*documents* an argument silently does nothing. That cost a session on
2026-08-22: `takeoff_alt` edited to 4 m in `phase1.launch.py`, vehicle climbed to
the wrapper's 1.0 m, nothing warned.
`hydrone_bringup/test/test_launch_arguments.py` fails if it comes back, for
`phase1_real` against **both** of its includes.

Cost: `ros2 launch -s phase1_real.launch.py` lists nothing. Run it against
`sources_real.launch.py` or `phase1.launch.py` instead.

---

## 7. Performance, on a Tegra X1

Everything here is sized for a 4 GB Nano/TX1-class board, not for the ZED 2i +
Orin the code was drafted against.

- **`zed_resolution: VGA`** (672×376 per eye). The depth pass scales with
  pixels and the two detectors need what CPU is left. HD720 works; watch the
  frame rate before trusting it.
- **`zed_depth_mode: PERFORMANCE`**. The pads are metres away and metres wide.
- **`zed_point_cloud: false`**. Only `feature_map_node` reads the cloud, it is
  an observer, and it is the most expensive publication in the stack. If you
  turn it on, leave `feature_map` on too; if you leave it off, pass
  `feature_map:=false` so a node is not waiting on a topic nobody publishes.
- The cloud is **strided** (`point_cloud_stride`, default 4) when on:
  `feature_map_node` voxelises anyway, so a quarter of the points cost a quarter
  of the bandwidth and barely move the map.

### `msg.data` must be an `array.array`, never `bytes`

Worth its own heading because it cost a factor of twelve and looked exactly like
a hardware problem.

The first bench run published **1.2 Hz** on a camera the SDK was feeding at 15.
Depth off changed nothing, and the CPU was 73% idle — so it was neither the
depth pass nor compute. Timing the pieces on the Jetson:

| | ms per 672x376x3 frame |
|---|---|
| `numpy.tobytes()` | 0.2 |
| **`msg.data = <bytes>`** | **361.0** |
| `msg.data = array.array("B", <bytes>)` | 0.2 |
| `publish()` | 0.6 |

rclpy's `uint8[]` field converts a `bytes` object element by element in Python.
Every image publication in the repo was doing it. Fixed in `zed_sdk_node`,
`down_cam_usb_node`, `zed_mimic_node`, `feature_map_node` and — the one that
matters most — `hydrone_vision/image_convert.py`, which is shared, so **the
simulator was paying the same cost on every frame it published**.

Measured after, on this board, all three buses at once:

```
/zed/zed_node/rgb/image_rect_color   14.96 Hz
/zed/zed_node/depth/depth_registered 14.90 Hz
/down_cam/image_raw                  14.63 Hz
```

against a requested 15. If you add a publication that carries a buffer, wrap it
the same way. And note the shape of this bug when measuring: the first RGB
reading after start-up was 5 Hz purely because the SDK's positional tracking was
still in `SEARCHING`. Warm up before believing a rate.

---

## 8. What the first real launch found

Everything before this section was validated by starting the nodes
**individually**. The first run of `sources_real.launch.py` as a launch file —
2026-08-22, on the board — failed before a single node started:

```
[ERROR] [launch]: Caught exception in launch (...):
        Unrecognized data type: [<class 'float'>]
```

`value_type=list([float])` is `[float]`, a list containing the *type*; launch
wants the typing generic `List[float]`. It is invisible to import, to
`generate_launch_description()`, and to every test that existed — the type is
not examined until launch **evaluates** the parameter. Fixed, and pinned by
`test_every_node_parameter_evaluates` in
`src/hydrone_bringup/test/test_launch_arguments.py`, which walks every launch
file and forces each `ParameterValue` through launch's own `extract_type`. The
test was checked by reintroducing the bug: it fails naming
`camera_offset_xyz`, and passes again when reverted.

Measured after the fix, whole launch, all four nodes up:

| bus | rate |
|---|---|
| `/zed/zed_node/rgb/image_rect_color` | 13.58 Hz |
| `/down_cam/image_raw` | 14.93 Hz |
| `/mavros/state` | `connected: false` — no FCU on the other end yet |

The lesson generalises: a launch file that has never been *run* is untested, no
matter how many of its nodes have been.

---

## 9. Not ready yet — read before flying

Ordered by how much damage each does.

1. **The belly camera is not calibrated.** Its `camera_info` is a nominal 60°
   pinhole derived from the C270's published FOV — zero distortion, principal
   point assumed dead centre. The belly camera has no depth, so a pad's position
   comes *entirely* from `fx/fy/cx/cy`, and an error there is a **lateral
   landing error**, not a detection failure: the pad is found, confidently, in
   the wrong place. `down_cam_usb_node` warns every 30 s. Fix it:
   ```bash
   ros2 run camera_calibration cameracalibrator \
       --size 8x6 --square 0.025 image:=/down_cam/image_raw camera:=/down_cam
   ```
   then pass `down_cam_calibrated:=true` with the numbers.
2. **The belly camera's mount is the simulated one.** `down_cam_mount_xyz`
   defaults to `[0, 0, -0.12]` and `down_cam_mount_rpy_deg` to `[0, 90, 0]` —
   the numbers from BiguaSim's virtual airframe. Measure the real one. Getting
   it wrong offsets every mapped pad in the direction the camera really points.
3. **The slow-flight limits are simulator-only.** `WP_SPD`, `WP_ACC` and
   `ATC_RATE_WPY_MAX` live in `holybro_sitl.parm`; `holybro_x500_tuned.parm`
   carries none of them. On ArduCopter 4.6.x they are also named differently —
   `WPNAV_SPEED` (cm/s), `WPNAV_ACCEL`, `ATC_SLEW_YAW` (cdeg/s). Check what the
   FCU is actually running; nothing in this stack constrains the real vehicle's
   speed.
4. **The ZED 1's tracking is visual-only.** No IMU means no stereo-inertial
   fusion, and the arena is texture-poor ([`LANDING-SITES.md`](LANDING-SITES.md)
   §10). Watch `/zed/zed_node/odom` against reality on the bench, moving the
   drone by hand, before letting the EKF fly on it.
5. **None of this has been flown.** The image builds and the nodes publish; that
   is all that has been demonstrated. Bench first: cameras only, then the map,
   then MAVROS connected but disarmed.
