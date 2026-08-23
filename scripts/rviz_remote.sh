#!/usr/bin/env bash
# rviz2 on THIS machine, looking at the topics the drone is publishing.
#
#   ./scripts/rviz_remote.sh                 # markers, TF, pose
#   ./scripts/rviz_remote.sh -d my.rviz      # with a saved config
#
# WHY NOT ON THE JETSON
# rviz2 is deliberately not in the drone's image. A Tegra X1 renders it badly,
# and forwarding its window over ssh -X pushes a whole framebuffer across the
# same wifi link as everything else. Running it here instead sends only the
# messages, and the ones you want from a drone -- markers, TF, pose, the pad
# map -- are tiny.
#
# DO NOT add /zed/.../image_rect_color or point_cloud/cloud_registered to an
# rviz display over wifi. 640x480 BGR at 15 Hz is ~110 Mbit/s and the point
# cloud is worse; it will starve the link the drone is also using for MAVLink.
# Use rqt_image_view on a single topic if you need to see a camera, or better,
# look at the debug images on the drone's own screen.
#
# DOMAIN IDS HAVE TO MATCH, and the two halves of this project do not agree by
# default: docker-compose.yml sets ROS_DOMAIN_ID=42 for the simulator, while
# scripts/jetson_up.sh leaves it unset, which DDS reads as 0. Set ROS_DOMAIN_ID
# the same on both sides -- here and in jetson_up.sh's environment -- or rviz
# will sit there showing nothing with no error at all.
#
# Verified 2026-08-23: PC 192.168.0.103 received a topic published by a
# container on the Jetson 192.168.0.102 over wifi, on domain 0.
set -euo pipefail
cd "$(dirname "$0")/.."

IMAGE="${IMAGE:-joao_pessoa_2026-hydrone:latest}"
DOMAIN="${ROS_DOMAIN_ID:-0}"
CONFIG=""
while [ $# -gt 0 ]; do
    case "$1" in
        -d|--config) CONFIG="${2:-}"; shift 2 ;;
        -h|--help)   sed -n '2,30p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

if [ -z "${DISPLAY:-}" ]; then
    echo "ERROR: DISPLAY is not set; there is no screen to draw on." >&2
    exit 1
fi

# The container draws on this X server, so it needs to be allowed to connect.
# Narrow grant: local connections only, not `xhost +`.
xhost +local: >/dev/null 2>&1 || true

# -t only when there is a terminal. `docker run -t` aborts with "the input
# device is not a TTY" under ssh-without-tty, a script, or a CI runner. Written
# as an `if` and not `[ -t 1 ] && ...` because under `set -e` a bare test that
# fails at the end of a list exits the script. Both traps were already hit and
# fixed in scripts/jetson_up.sh; this file repeated them.
if [ -t 1 ]; then
    tty_args=(-it)
else
    tty_args=(-i)
fi

run_args=(
    --rm "${tty_args[@]}"
    --network host        # DDS discovery has to reach the drone's subnet
    --ipc host
    -e "DISPLAY=$DISPLAY"
    -e "ROS_DOMAIN_ID=$DOMAIN"
    -v /tmp/.X11-unix:/tmp/.X11-unix
)
xauth_file="${XAUTHORITY:-$HOME/.Xauthority}"
if [ -r "$xauth_file" ]; then
    run_args+=(-v "$xauth_file:/tmp/.xauth:ro" -e XAUTHORITY=/tmp/.xauth)
fi

inner=". /opt/ros/humble/setup.sh; [ -f /ws/install/setup.sh ] && "
inner+=". /ws/install/setup.sh; exec rviz2"
if [ -n "$CONFIG" ]; then
    [ -f "$CONFIG" ] || { echo "no such config: $CONFIG" >&2; exit 1; }
    run_args+=(-v "$(realpath "$CONFIG"):/tmp/rviz.rviz:ro")
    inner+=" -d /tmp/rviz.rviz"
fi

echo "rviz2 on ROS_DOMAIN_ID=$DOMAIN  (the drone must match)"
echo "add:  /hydrone/pads/markers   TF   /mavros/local_position/pose"
echo "avoid: raw images and point clouds -- this is a wifi link"
exec docker run "${run_args[@]}" "$IMAGE" bash -lc "$inner"
