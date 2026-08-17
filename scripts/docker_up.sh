#!/usr/bin/env bash
# Bring up the full simulation stack in Docker.
#
# Usage: scripts/docker_up.sh [--dev] [--no-build] [--landing-sites]
#                             [--no-odom-print] [docker compose up args...]
#   --dev             mount the project packages from the host into the
#                     container (docker-compose.dev.yml) and skip the image
#                     build. Node code, launch files and config YAML are then
#                     live — edit, `docker compose restart hydrone`, done.
#                     Use scripts/dev_rebuild.sh for .msg / setup.py / new-file
#                     changes. Implies --no-build.
#   --no-build        don't pass --build to compose (reuse the current image).
#   --landing-sites   run the autonomous landing-site mission (pad detectors +
#                     pad/feature maps + search-land-take-off state machine)
#                     instead of the bare sim bring-up. See docs/LANDING-SITES.md.
#   --no-odom-print   silence odom_error_node's 1 Hz VO-drift line (the CSV is
#                     still written either way). On by default.
# Anything else is forwarded untouched to `docker compose up` (-d, --force-recreate, ...).
set -e
cd "$(dirname "$0")/.."

# Consume our own flag and pass the rest through. Only an exact match is
# intercepted, so compose's own flags are never swallowed by accident.
ODOM_ERROR_PRINT=true
HYDRONE_LAUNCH=hydrone_sim.launch.py
DEV_MODE=false
DO_BUILD=true
compose_args=()
for arg in "$@"; do
    case "$arg" in
        --no-odom-print) ODOM_ERROR_PRINT=false ;;
        --landing-sites) HYDRONE_LAUNCH=landing_sites_sim.launch.py ;;
        --dev)           DEV_MODE=true; DO_BUILD=false ;;
        --no-build)      DO_BUILD=false ;;
        *)               compose_args+=("$arg") ;;
    esac
done
export ODOM_ERROR_PRINT   # interpolated into `command:` in docker-compose.yml
export HYDRONE_LAUNCH     # ditto — selects which launch file the container runs

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
echo "VO drift print: $ODOM_ERROR_PRINT (CSV is written either way)"

build_arg=(--build)
[ "$DO_BUILD" = true ] || build_arg=()

docker compose "${compose_files[@]}" up "${build_arg[@]}" "${compose_args[@]}"
