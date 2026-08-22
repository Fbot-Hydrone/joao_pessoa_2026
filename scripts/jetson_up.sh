#!/usr/bin/env bash
# Run the stack on the drone's Jetson.
#
#   ./scripts/jetson_up.sh                          # phase 1 mission, live code
#   ./scripts/jetson_up.sh --sources                # cameras + MAVROS only
#
# (There is no --landing-sites here: landing_sites has no _real wrapper. Its
# sim launch drives BiguaSim's mimic nodes, which have no hardware counterpart.)
#   ./scripts/jetson_up.sh --shell                  # a shell inside the image
#   ./scripts/jetson_up.sh --calibrate              # belly camera + the
#                                                   # calibration GUI
#   ./scripts/jetson_up.sh takeoff_alt:=1.0 target_bases:=1
#   ./scripts/jetson_up.sh --rebuild                # after a .msg/setup.py edit
#   ./scripts/jetson_up.sh --build                  # rebuild the image first
#
# This is the Jetson's answer to scripts/docker_up.sh. It is a plain `docker
# run` and not compose because the board has neither `docker compose` nor
# `docker-compose` installed, and putting them there to launch one container is
# not worth the disk.
#
# ── WHY YOU ALMOST NEVER REBUILD ────────────────────────────────────────────
# The project packages are bind-mounted from the checkout over /ws/src/<pkg>,
# the same trick as docker-compose.dev.yml. The image was built with
# `colcon build --symlink-install`, which chains
#
#   install/<pkg>/.../<pkg>  ->  /ws/build/<pkg>  ->  /ws/src/<pkg>/<pkg>
#
# so mounting the working tree at the end of that chain makes node code,
# existing launch files and existing config YAML live. Edit on the Jetson (or
# rsync to it), re-run this script, and the new code runs. No image build, no
# colcon, no layer cache to think about.
#
# WHAT IS *NOT* LIVE, and needs --rebuild (colcon inside the container):
#   * .msg / .srv changes in hydrone_msgs      (code generation)
#   * new entry_points in setup.py             (console-script wrappers)
#   * a launch or config file that did NOT EXIST at image-build time
#     (--symlink-install creates one symlink per file, at build time, so a new
#      file has nothing pointing at it)
#   * new package.xml dependencies
#
# --rebuild does not persist: the container is --rm, so its writable layer goes
# away with it. Pass --rebuild on each run until the change is worth a --build.
#
# WHAT NEEDS --build (a real image build): a new apt/pip dependency, a new SDK,
# or anything else in Dockerfile.jetson. The expensive layers (apt, MAVROS, the
# ZED SDK, pyzed) all sit ABOVE the `COPY src/` line, so they stay cached and
# only the colcon layer is redone.
#
# Overridable by environment: IMAGE, CONTAINER, FCU_URL.
set -euo pipefail
cd "$(dirname "$0")/.."

IMAGE="${IMAGE:-hydrone-jetson:humble}"
CONTAINER="${CONTAINER:-hydrone-jetson}"

LAUNCH=phase1_real.launch.py
DO_BUILD=false
DO_REBUILD=false
SHELL_MODE=false
CALIBRATE=false
NO_DEV=false
launch_args=()

# The belly camera's ChArUco target. Defaults are the calib.io board in use
# here, whose own footer reads "11x9 | Checker 22 mm | Marker 16 mm | DICT_4X4".
# Note --size is COLUMNS x ROWS and counts SQUARES, not interior corners, which
# is the opposite of what -p chessboard wants; scripts/charuco_probe.py checks
# it against the camera. docs/CALIBRATION.md has the why.
CAL_SIZE="${CAL_SIZE:-9x11}"
CAL_SQUARE="${CAL_SQUARE:-0.022}"
CAL_MARKER="${CAL_MARKER:-0.016}"
CAL_DICT="${CAL_DICT:-4x4_250}"

for arg in "$@"; do
    case "$arg" in
        --phase1)        LAUNCH=phase1_real.launch.py ;;
        --sources)       LAUNCH=sources_real.launch.py ;;
        --shell)         SHELL_MODE=true ;;
        --calibrate)     CALIBRATE=true ;;
        --build)         DO_BUILD=true ;;
        --rebuild)       DO_REBUILD=true ;;
        --no-dev)        NO_DEV=true ;;
        *:=*)            launch_args+=("$arg") ;;
        -h|--help)       sed -n '2,50p' "$0"; exit 0 ;;
        *) echo "unknown argument: $arg" >&2; exit 2 ;;
    esac
done

# ── Preflight ───────────────────────────────────────────────────────────────
# Each of these turns a confusing runtime failure into a sentence. The
# entrypoint re-checks the first two from inside, but by then the error is
# buried in ROS output.
if ! docker info 2>/dev/null | grep -qi 'runtimes:.*nvidia'; then
    echo "ERROR: the nvidia container runtime is not registered with docker." >&2
    echo "  Without it CUDA and libcuda.so.1 are never injected and the ZED" >&2
    echo "  SDK fails at dlopen with what looks like a driver problem." >&2
    exit 1
fi
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Image $IMAGE not found — building it." >&2
    DO_BUILD=true
fi
if ! ls /dev/video* >/dev/null 2>&1; then
    echo "WARNING: no /dev/video* on this host. Is the ZED plugged in?" >&2
fi

if [ "$DO_BUILD" = true ]; then
    if ! ls docker/ZED_SDK_*.run >/dev/null 2>&1; then
        echo "ERROR: no docker/ZED_SDK_*.run in the checkout." >&2
        echo "  It is deliberately not committed (84 MB, and Stereolabs no" >&2
        echo "  longer serves it to a plain GET). Download 'ZED SDK for" >&2
        echo "  JetPack 4.6.X (L4T 32.7) 4.0.8' from stereolabs.com/developers" >&2
        echo "  and drop it in docker/.  See docs/JETSON-REAL-STACK.md §3." >&2
        exit 1
    fi
    docker build -f docker/Dockerfile.jetson -t "$IMAGE" .
fi

# ── The run ─────────────────────────────────────────────────────────────────
#   --runtime nvidia : injects the host's CUDA 10.2, libcuda and the Tegra EGL
#                      vendor driver. Not optional; see Dockerfile.jetson.
#   --network host   : DDS discovery, and reaching a ground station.
#   --ipc host       : DDS shared-memory transport between containers.
#   -v /dev:/dev --privileged : the ZED needs BOTH /dev/video* (V4L2 capture)
#                      and /dev/bus/usb (libusb control transfers for its
#                      settings), and MAVROS needs /dev/ttyTHS1. Enumerating
#                      every --device is possible but the USB node numbers
#                      change on replug, so this is the honest option.
# -t only when there IS a terminal: `docker run -t` aborts with "the input
# device is not a TTY" under ssh-without-tty, nohup or a systemd unit, which is
# exactly how this gets started on a drone that nobody is sitting next to.
# NB: written as `if`, not `[ -t 1 ] && tty_args=(-it)`. Under `set -e` a
# bare `test && assign` that tests FALSE is a failing command at the end of a
# list, and the script exits — silently, at the one moment there is no
# terminal to print to. scripts/docker_up.sh had exactly this bug.
if [ -t 1 ]; then
    tty_args=(-it)
else
    tty_args=(-i)
fi

run_args=(
    --rm "${tty_args[@]}"
    --name "$CONTAINER"
    # --init puts tini at PID 1 and it forwards signals to the real process.
    # It is not optional. `exec ros2 launch` below makes the launch PID 1, and
    # the KERNEL DOES NOT APPLY DEFAULT SIGNAL ACTIONS TO PID 1: a signal with
    # no installed handler is discarded. ros2 launch (Python) handles SIGINT
    # but not SIGTERM, so `docker stop` and a supervisor's SIGTERM were both
    # silently ignored. Measured on the board: `timeout 30` around this script
    # returned, and the stack — both cameras and MAVROS — was still running
    # three minutes later. tini takes PID 1 instead and passes the signal on.
    --init
    # ...and make `docker stop` send the signal ros2 launch shuts down
    # GRACEFULLY on, rather than SIGTERM followed by SIGKILL 10 s later.
    --stop-signal SIGINT
    --runtime nvidia
    --network host
    --ipc host
    --privileged
    -v /dev:/dev
)

# The dev bind mounts. Per-package and not a blanket ./src:/ws/src, for the
# same reason as docker-compose.dev.yml: a whole-src mount would shadow
# anything the image built in place under /ws/src.
if [ "$NO_DEV" = false ]; then
    for pkg in hydrone_bringup hydrone_controller hydrone_mission \
               hydrone_msgs hydrone_nav hydrone_vision; do
        run_args+=(-v "$PWD/src/$pkg:/ws/src/$pkg")
    done
fi

# PYTHONDONTWRITEBYTECODE: the container is root and the mounts are the host's
# checkout, so without this every run salts the working tree with root-owned
# __pycache__ directories that the normal user then cannot delete.
run_args+=(-e PYTHONDONTWRITEBYTECODE=1)

# ── X, for the calibration GUI ──────────────────────────────────────────────
# Two display routes, and they need different plumbing:
#   the Jetson's own screen  -> DISPLAY=:0, a unix socket in /tmp/.X11-unix
#   ssh -X from your desk    -> DISPLAY=localhost:10.0, a TCP port that
#                               --network host already reaches
# Both need the auth cookie, or the connection is refused with "Authorization
# required". The container runs as root and the cookie belongs to the login
# user, so it is mounted explicitly rather than relying on $HOME.
if [ "$CALIBRATE" = true ]; then
    run_args+=(-e "DISPLAY=${DISPLAY:-:0}")
    [ -d /tmp/.X11-unix ] && run_args+=(-v /tmp/.X11-unix:/tmp/.X11-unix)
    # Finding the cookie is the fiddly part. $HOME/.Xauthority is very often
    # STALE -- on this board it was two months old and held cookies for a
    # display that no longer exists, which produces
    #     Invalid MIT-MAGIC-COOKIE-1 key
    # and then a GTK failure that reads like missing X libraries rather than
    # an auth problem. The authoritative answer is the -auth file the running
    # X server was started with, so ask it: gdm keeps it under /run/user.
    xauth_file="${XAUTHORITY:-}"
    if [ -z "$xauth_file" ]; then
        xauth_file=$(ps -o args= -C Xorg 2>/dev/null |
                     sed -n 's/.*-auth \([^ ]*\).*/\1/p' | head -1)
    fi
    [ -z "$xauth_file" ] && xauth_file="$HOME/.Xauthority"

    if [ -r "$xauth_file" ]; then
        echo "X cookie: $xauth_file"
        run_args+=(-v "$xauth_file:/root/.Xauthority:ro"
                   -e XAUTHORITY=/root/.Xauthority)
    else
        echo "NOTE: no readable X cookie (tried $xauth_file). If the GUI is" >&2
        echo "      refused, run 'xhost +local:' on the machine that owns" >&2
        echo "      the display." >&2
    fi
fi

if [ "$SHELL_MODE" = true ]; then
    exec docker run "${run_args[@]}" "$IMAGE" bash
fi

# ── Calibration: the belly camera, and the GUI, and nothing else ────────────
# Deliberately NOT sources_real.launch.py. That would also start the ZED, its
# depth computation and MAVROS -- on a Tegra X1 that is most of the CPU, and
# the calibrator wants frames delivered promptly or the GUI feels broken.
if [ "$CALIBRATE" = true ]; then
    echo "+ belly camera + cameracalibrator"
    echo "  board: --size $CAL_SIZE --square $CAL_SQUARE"
    echo "         --charuco_marker_size $CAL_MARKER --aruco_dict $CAL_DICT"
    echo "  DISPLAY=${DISPLAY:-:0}"
    echo
    echo "  Fill all four coverage bars (X, Y, Size, Skew), then CALIBRATE,"
    echo "  then SAVE. Do NOT use COMMIT: down_cam_usb_node takes its"
    echo "  intrinsics as ROS parameters and offers no set_camera_info"
    echo "  service, so COMMIT has nothing to talk to. SAVE writes"
    echo "  /tmp/calibrationdata.tar.gz inside the container -- copy it out"
    echo "  with 'docker cp $CONTAINER:/tmp/calibrationdata.tar.gz .' from"
    echo "  another terminal BEFORE you stop this, because --rm discards the"
    echo "  container's filesystem on exit."
    echo
    exec docker run "${run_args[@]}" -w /ws "$IMAGE" bash -lc "
        set -e
        . /ws/install/setup.sh
        ros2 run hydrone_bringup down_cam_usb_node --ros-args             -p device:=/dev/v4l/by-path/platform-70090000.xusb-usb-0:2.2:1.0-video-index0             &
        camera_pid=\$!
        trap 'kill \$camera_pid 2>/dev/null' EXIT
        # Wait for the topic rather than sleeping: the calibrator subscribes
        # once at startup and does not retry, so racing it produces a window
        # that never updates and looks like a broken camera.
        for _ in \$(seq 1 30); do
            ros2 topic list 2>/dev/null | grep -q /down_cam/image_raw && break
            sleep 1
        done
        exec ros2 run camera_calibration cameracalibrator             --pattern charuco --size $CAL_SIZE --square $CAL_SQUARE             --charuco_marker_size $CAL_MARKER --aruco_dict $CAL_DICT             --no-service-check             image:=/down_cam/image_raw camera:=/down_cam"
fi

inner=""
if [ "$DO_REBUILD" = true ]; then
    # --symlink-install to KEEP the chain the bind mounts depend on; a plain
    # colcon build would replace the symlinks with copies and silently freeze
    # the code at its current state.
    inner+=". /opt/ros/humble/setup.sh && colcon build --symlink-install "
    inner+="--packages-select hydrone_msgs hydrone_controller hydrone_bringup "
    inner+="hydrone_vision hydrone_nav hydrone_mission && . /ws/install/setup.sh && "
fi
# `exec` so that ros2 launch becomes PID 1 in the container. Without it PID 1
# is this bash, and bash does NOT forward signals to a child it is waiting on:
# Ctrl-C, `docker stop`, or a supervisor's SIGTERM would kill the docker CLIENT
# and leave the container — cameras, MAVROS and all — running unattended.
# Measured: a `timeout 40` around this script left the stack up for minutes.
inner+="exec ros2 launch hydrone_bringup $LAUNCH ${launch_args[*]:-}"

echo "+ $LAUNCH ${launch_args[*]:-}"
exec docker run "${run_args[@]}" -w /ws "$IMAGE" bash -lc "$inner"
