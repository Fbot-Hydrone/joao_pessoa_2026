#!/usr/bin/env bash
# Rebuild the project packages inside the ALREADY RUNNING container.
#
# Usage: scripts/dev_rebuild.sh [--restart] [package ...]
#   --restart   restart the hydrone service afterwards so the new code runs
#               (needed for anything but a pure edit to an already-loaded file)
#   package...  packages to rebuild; default is all project packages
#
# This is the companion to `scripts/docker_up.sh --dev`. With the dev bind
# mounts in place, most edits (node code, existing launch files, existing
# config YAML) are already live and need nothing but a restart. Run this when
# you changed something the symlink chain can't pick up:
#
#   * .msg files in hydrone_msgs / biguasim_interfaces
#   * entry_points / data_files in setup.py
#   * files that did not exist at image-build time (new launch or config file)
#
# It writes to the container's writable layer, so the result survives
# `docker compose stop/start` and `restart` but NOT `docker compose down`.
# After a `down`, either rerun this once or rebuild the image normally.
set -e
cd "$(dirname "$0")/.."

ALL_PKGS=(hydrone_msgs biguasim_interfaces biguasim_main hydrone_bringup
          hydrone_vision hydrone_controller hydrone_nav hydrone_mission)

RESTART=false
pkgs=()
for arg in "$@"; do
    case "$arg" in
        --restart) RESTART=true ;;
        *)         pkgs+=("$arg") ;;
    esac
done
[ ${#pkgs[@]} -gt 0 ] || pkgs=("${ALL_PKGS[@]}")

if ! docker compose ps --status running --services 2>/dev/null | grep -qx hydrone; then
    echo "ERROR: the 'hydrone' service isn't running." >&2
    echo "  Start it first:  ./scripts/docker_up.sh --dev" >&2
    exit 1
fi

echo "Rebuilding in-container: ${pkgs[*]}"
# --symlink-install to keep the install/ -> build/ -> src/ chain that makes the
# bind mounts live; without it the rebuilt packages would go back to copies.
#
# `exec` runs as root (it bypasses the entrypoint's drop to uid 1000), so
# PYTHONDONTWRITEBYTECODE keeps stray root-owned __pycache__ out of the
# bind-mounted host tree; the chown afterwards cleans up anything else that
# slipped through. colcon itself writes only to /ws/build inside the container.
docker compose exec -w /ws -e PYTHONDONTWRITEBYTECODE=1 hydrone bash -c \
    ". /opt/ros/humble/setup.sh && colcon build --symlink-install --packages-select ${pkgs[*]}"
# Only the bind-mounted project trees — never /ws/src/ardupilot & friends,
# which are image-internal and legitimately root-owned.
MOUNTED_SRC=(/ws/src/hydrone_bringup /ws/src/hydrone_controller
             /ws/src/hydrone_mission /ws/src/hydrone_msgs /ws/src/hydrone_nav
             /ws/src/hydrone_vision /ws/src/biguasim-ros2)
docker compose exec -w /ws hydrone bash -c \
    "find ${MOUNTED_SRC[*]} ! -uid $(id -u) -exec chown $(id -u):$(id -g) {} + 2>/dev/null; true"

if [ "$RESTART" = true ]; then
    echo "Restarting hydrone ..."
    docker compose restart hydrone
else
    echo "Done. Restart to pick it up:  docker compose restart hydrone"
fi
