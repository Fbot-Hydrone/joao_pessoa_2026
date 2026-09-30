#!/usr/bin/env bash
# Bring up the full simulation stack in Docker.
#
# Usage: scripts/docker_up.sh [--dev] [--no-build] [--phase1] [--landing-sites]
#                             [--phase3] [--phase4] [--ground-truth] [--no-odom-print]
#                             [--world HOST[:PORT]] [--world-port PORT]
#                             [name:=value ...] [docker compose up args...]
#   --dev             mount the project packages from the host into the
#                     container (docker-compose.dev.yml) and skip the image
#                     build. Node code, launch files and config YAML are then
#                     live — edit, `docker compose restart hydrone`, done.
#                     Use scripts/dev_rebuild.sh for .msg / setup.py / new-file
#                     changes. Implies --no-build.
#   --no-build        don't pass --build to compose (reuse the current image).
#   --phase1          run the Phase 1 mission (take off, turn on the spot until
#                     a landing base is found, fly over it, confirm it on the
#                     belly camera, land, repeat, come home) instead of the bare
#                     sim bring-up. See docs/Phase 1 Mission.md.
#   --landing-sites   run the earlier landing-site mission (fly forward and land
#                     on whatever the belly camera sees). See docs/Landing Sites.md.
#   --phase3          the Phase 3 gesture mission on the Kopis X8: the Phase 4
#                     LIO stack plus a forward camera (config-KopisX8Cam.yaml),
#                     gesture_detector_node and phase3_gesture_node (takeoff,
#                     1 m forward, 90 deg right, obey the operator's arms).
#                     The sim has no person: type gestures on
#                     /hydrone/gesture/inject. See docs/Phase 3 Gesture Mission.md.
#   --phase4          bring up the OTHER AIRCRAFT: the Kopis X8 flying on its
#                     Livox Mid-360 (config-KopisX8.yaml, an engine raycast
#                     lidar). FAST-LIO odometry is the EKF's external nav; the
#                     persistent map node runs too. Local simulator only.
#                     --ground-truth and --no-odom-print don't apply.
#                     See phase4_sim.launch.py and docs/LIO Odometry.md.
#   --ground-truth    fly the EKF on BiguaSim ground truth instead of the real
#                     visual odometry (odom_source:=ground_truth). A DEBUGGING
#                     AID for separating autonomy bugs from localization bugs —
#                     a green run on ground truth proves nothing about the real
#                     drone, which has none. See docs/Landing Sites.md.
#   --no-odom-print   silence odom_error_node's 1 Hz VO-drift line (the CSV is
#                     still written either way). On by default.
#   --world HOST[:PORT]
#                     fly against a BiguaSim world running in ANOTHER process,
#                     instead of starting the simulator in this container.
#                     SITL and the whole autonomy stack still run here; only the
#                     physics, the sensor rendering and the collision checking
#                     move. Several machines can then share one simulation and
#                     SEE each other in it, which a local sim cannot do.
#                     PORT defaults to 8770 (the world's request port; it also
#                     uses PORT+1 to publish state).
#                     Start the world with, on the other machine:
#                       python tools/serve_world.py --package Competition \
#                              --world CompetionMap --port 8770
#   --world-port PORT the same port, given separately. Handy for an IPv6
#                     literal, where HOST:PORT is ambiguous.
#
# Running against a world needs the world PACKAGE installed here too
# (~/.local/share/biguasim, already bind-mounted): the client refuses to connect
# unless its copy of the world matches the server's, because mismatched
# collision geometry looks like broken physics rather than a version problem.
#
# Any argument containing ':=' is a LAUNCH argument and is appended to the
# ros2 launch command inside the container, so the mission can be tuned without
# editing a file:
#
#   scripts/docker_up.sh --phase1 target_bases:=2 takeoff_alt:=1.5
#
# --phase1, --landing-sites, --phase3 and --phase4 all pick the launch file, so they are
# mutually exclusive; the last one given wins.
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
WORLD_ADDRESS="${WORLD_ADDRESS:-}"
WORLD_PORT="${WORLD_PORT:-8770}"
launch_args=()
compose_args=()
# --world and --world-port take a value, so the next argument belongs to them
# rather than to compose. Tracked with a flag instead of shift/getopts to leave
# the existing pass-everything-else-through behaviour exactly as it was.
want_value=
for arg in "$@"; do
    if [ -n "$want_value" ]; then
        case "$want_value" in
            world)
                # host:port, but only when the colon is unambiguous. An IPv6
                # literal is full of colons and is left alone -- use
                # --world-port for those, or bracket the address.
                case "$arg" in
                    \[*\]:*) WORLD_ADDRESS="${arg%:*}"; WORLD_PORT="${arg##*:}" ;;
                    *:*:*)    WORLD_ADDRESS="$arg" ;;
                    *:*)      WORLD_ADDRESS="${arg%:*}"; WORLD_PORT="${arg##*:}" ;;
                    *)        WORLD_ADDRESS="$arg" ;;
                esac
                ;;
            world-port) WORLD_PORT="$arg" ;;
        esac
        want_value=
        continue
    fi
    case "$arg" in
        --no-odom-print) ODOM_ERROR_PRINT=false ;;
        --world)         want_value=world ;;
        --world-port)    want_value=world-port ;;
        --phase1)        HYDRONE_LAUNCH=phase1_sim.launch.py ;;
        --landing-sites) HYDRONE_LAUNCH=landing_sites_sim.launch.py ;;
        --phase3)        HYDRONE_LAUNCH=phase3_sim.launch.py ;;
        --phase4)        HYDRONE_LAUNCH=phase4_sim.launch.py ;;
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

# odom_source and odom_error_print belong to the Holybro launches, which choose
# between two estimators and measure one against the other. phase4_sim declares
# neither: that aircraft carries one sensor and has no estimator yet. Launch
# accepts an undeclared argument silently (it just becomes a launch
# configuration nobody reads), so passing them would not error — it would only
# leave the command line, and this script's summary, describing a vehicle this
# is not. Empty for phase 4, and UNSET is what docker-compose.yml falls back on
# for a plain `docker compose up`.
HYDRONE_ODOM_ARGS="odom_error_print:=$ODOM_ERROR_PRINT odom_source:=$ODOM_SOURCE"
case "$HYDRONE_LAUNCH" in phase3_sim.launch.py|phase4_sim.launch.py) HYDRONE_ODOM_ARGS= ;; esac

export HYDRONE_LAUNCH       # interpolated into `command:` in docker-compose.yml
export HYDRONE_ODOM_ARGS    # ditto — the odom pair above, or empty for phase 4
export HYDRONE_LAUNCH_ARGS  # ditto — extra name:=value pairs, possibly empty
export WORLD_ADDRESS        # empty = simulate here; set = use a world elsewhere
export WORLD_PORT           # its request port; state is published on PORT+1

if [ -n "$want_value" ]; then
    echo "ERROR: --$want_value needs a value" >&2
    exit 1
fi

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

if [ -n "$WORLD_ADDRESS" ]; then
    # Bracket a bare IPv6 literal for display only; ZeroMQ's endpoint builder
    # does the same thing to the real address. Without it the port looks like
    # one more group of the address.
    shown="$WORLD_ADDRESS"
    case "$shown" in
        \[*\]) ;;                      # already bracketed
        *:*:*) shown="[$shown]" ;;     # a bare IPv6 literal
    esac
    echo "World        : $shown:$WORLD_PORT (remote — physics and sensors run there)"
else
    echo "World        : local (this container runs the simulator)"
fi
echo "Launch file  : $HYDRONE_LAUNCH"
if [ -n "$HYDRONE_LAUNCH_ARGS" ]; then
    echo "Launch args  : $HYDRONE_LAUNCH_ARGS"
fi
if [ -z "$HYDRONE_ODOM_ARGS" ]; then
    echo "Aircraft     : Kopis X8 + Livox Mid-360"
    echo "Nav          : FAST-LIO on the Mid-360 as external nav (ground truth"
    echo "               only feeds the drift log). See docs/LIO Odometry.md."
else
    echo "Odom source  : $ODOM_SOURCE$([ "$ODOM_SOURCE" = ground_truth ] && echo ' (DEBUGGING AID — proves nothing about the real drone)')"
    echo "VO drift print: $ODOM_ERROR_PRINT (CSV is written either way)"
fi

build_arg=(--build)
[ "$DO_BUILD" = true ] || build_arg=()

docker compose "${compose_files[@]}" up "${build_arg[@]}" "${compose_args[@]}"
