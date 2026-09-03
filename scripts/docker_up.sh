#!/usr/bin/env bash
# Bring up the full simulation stack in Docker.
#
# Usage: scripts/docker_up.sh [--dev] [--no-build] [--phase1] [--landing-sites]
#                             [--ground-truth] [--no-odom-print]
#                             [name:=value ...] [docker compose up args...]
#   --dev             mount the project packages from the host into the
#                     container (docker-compose.dev.yml) and skip the image
#                     build. Node code, launch files and config YAML are then
#                     live — edit, `docker compose restart hydrone`, done.
#                     Use scripts/dev_rebuild.sh for .msg / setup.py / new-file
#                     changes. Implies --no-build.
#   --no-build        don't pass --build to compose (reuse the current image).
#   --phase1          run the Phase 1 mission: take off, MAP the arena with a
#                     closed perimeter, mow it with the belly camera, land on
#                     every base found, come home. The ZED does odometry and
#                     mapping only; the belly camera is the detector AND the
#                     position source, placing each pad by casting its pixel
#                     into the occupancy map. See docs/PHASE1-MISSION.md and
#                     docs/MAP-SWEEP-2026-09-02.md.
#   --landing-sites   run the earlier landing-site mission (fly forward and land
#                     on whatever the belly camera sees). See docs/LANDING-SITES.md.
#   --zed-detect      run the OLDER division of labour, where the forward ZED
#                     both finds a base across the arena and says where it is,
#                     the belly camera only votes yes/no, and the search is a
#                     three-sided U flown twice. This was --phase1 until
#                     2026-09-02. Kept because it places pads better (2-16 cm
#                     against 45 cm) while finding fewer of them, so it is both
#                     the fallback and the comparison. See
#                     src/hydrone_bringup/launch/phase1_zed_detect.launch.py.
#   --ground-truth    fly the EKF on BiguaSim ground truth instead of the real
#                     visual odometry (odom_source:=ground_truth). A DEBUGGING
#                     AID for separating autonomy bugs from localization bugs —
#                     a green run on ground truth proves nothing about the real
#                     drone, which has none. See docs/LANDING-SITES.md §10.
#   --debug           open the windows: rviz2 preloaded with the mission's
#                     layout (octomap, vehicle pose, the pad map, the belly
#                     camera's coverage/footprint/trajectory, the planned
#                     route) and rqt_image_view on the belly detector's
#                     annotated view. Both run INSIDE the container and draw on
#                     this machine's X server, so there is no ROS_DOMAIN_ID to
#                     match and nothing else to start. Needs a display.
#   --no-odom-print   silence odom_error_node's 1 Hz VO-drift line (the CSV is
#                     still written either way). On by default.
#
# Any argument containing ':=' is a LAUNCH argument and is appended to the
# ros2 launch command inside the container, so the mission can be tuned without
# editing a file:
#
#   scripts/docker_up.sh --phase1 target_bases:=2 takeoff_alt:=1.5
#
# --phase1, --landing-sites and --zed-detect are mutually exclusive; the last
# one given wins.
# Anything else is forwarded untouched to `docker compose up` (-d, --force-recreate, ...).
set -e
cd "$(dirname "$0")/.."

# Consume our own flag and pass the rest through. Only an exact match is
# intercepted, so compose's own flags are never swallowed by accident.
ODOM_ERROR_PRINT=true
HYDRONE_LAUNCH=hydrone_sim.launch.py
ODOM_SOURCE=vo
DEV_MODE=false
DO_BUILD=true
launch_args=()
compose_args=()
for arg in "$@"; do
    case "$arg" in
        --no-odom-print) ODOM_ERROR_PRINT=false ;;
        --debug)         launch_args+=("debug:=true") ;;
        --phase1)        HYDRONE_LAUNCH=phase1_sim.launch.py ;;
        --landing-sites) HYDRONE_LAUNCH=landing_sites_sim.launch.py ;;
        --zed-detect)    HYDRONE_LAUNCH=phase1_zed_detect_sim.launch.py ;;
        --ground-truth)  ODOM_SOURCE=ground_truth ;;
        --dev)           DEV_MODE=true; DO_BUILD=false ;;
        --no-build)      DO_BUILD=false ;;
        # A ros2 launch argument, not a compose one. Matched on the ':=' rather
        # than on a list of known names so a new launch argument needs no change
        # here — the launch file is the one place that knows what it accepts,
        # and it errors clearly on a name it does not.
        *:=*)            launch_args+=("$arg") ;;
        *)               compose_args+=("$arg") ;;
    esac
done
# Joined with spaces because docker compose interpolates this into the `command:`
# STRING and then splits it shell-style. That also means a launch argument whose
# value contains a space would not survive; none of ours do.
HYDRONE_LAUNCH_ARGS="${launch_args[*]}"
export ODOM_ERROR_PRINT     # interpolated into `command:` in docker-compose.yml
export HYDRONE_LAUNCH       # ditto — selects which launch file the container runs
export ODOM_SOURCE          # ditto — what the EKF navigates on (vo|ground_truth)
export HYDRONE_LAUNCH_ARGS  # ditto — extra name:=value pairs, possibly empty

# Let the containerized UE5 viewport open on the host X server
xhost +local:docker

# Make sure the shared asset dir exists so the bind mount doesn't create it root-owned
mkdir -p "$HOME/.local/share/biguasim"

# Locate the BiguaSim repo (mounted into the container). Set BS_SIM_DIR to
# override; otherwise try the common sibling locations.
if [ -z "${BS_SIM_DIR:-}" ]; then
    for candidate in ../bs-competition/bs-drone-competition ../bs-drone-competition; do
        if [ -d "$candidate" ]; then
            BS_SIM_DIR=$candidate
            break
        fi
    done
fi
if [ ! -d "${BS_SIM_DIR:-}" ]; then
    echo "ERROR: BiguaSim repo not found. Clone it and/or set BS_SIM_DIR, e.g.:" >&2
    echo "  BS_SIM_DIR=~/Documents/bs-drone-competition $0" >&2
    exit 1
fi
export BS_SIM_DIR
echo "BiguaSim repo: $BS_SIM_DIR"

# Use the NVIDIA dGPU when the container runtime is available (see
# docker-compose.nvidia.yml for the host setup), otherwise fall back to
# the integrated GPU via /dev/dri.
compose_files=(-f docker-compose.yml)
if docker info 2>/dev/null | grep -qi 'runtimes:.*nvidia'; then
    echo "NVIDIA container runtime detected — rendering on the dGPU"
    compose_files+=(-f docker-compose.nvidia.yml)
else
    echo "No NVIDIA container runtime — rendering on the iGPU (see README for dGPU setup)"
fi

if [ "$DEV_MODE" = true ]; then
    echo "Dev mode — project packages bind-mounted from ./src (no image build)"
    compose_files+=(-f docker-compose.dev.yml)
fi

echo "Launch file  : $HYDRONE_LAUNCH"
if [ -n "$HYDRONE_LAUNCH_ARGS" ]; then
    echo "Launch args  : $HYDRONE_LAUNCH_ARGS"
fi
echo "Odom source  : $ODOM_SOURCE$([ "$ODOM_SOURCE" = ground_truth ] && echo ' (DEBUGGING AID — proves nothing about the real drone)')"
echo "VO drift print: $ODOM_ERROR_PRINT (CSV is written either way)"

build_arg=(--build)
[ "$DO_BUILD" = true ] || build_arg=()

docker compose "${compose_files[@]}" up "${build_arg[@]}" "${compose_args[@]}"
