#!/usr/bin/env bash
# View one of the drone's image topics on THIS machine.
#
#   ./scripts/view_remote.sh /hydrone/pads/down/debug_image
#   ./scripts/view_remote.sh /down_cam/image_raw
#
# Not `docker exec` into the drone's container: exec cannot add mounts or
# devices to something already running (`unknown shorthand flag: 'v'`), so the
# X socket and scripts/ cannot be attached after the fact. This runs the viewer
# HERE and lets DDS carry the images across.
#
# ROS_DOMAIN_ID must match the drone. jetson_up.sh leaves it unset, which DDS
# reads as 0, while docker-compose.yml sets 42 for the simulator -- so the
# default here is 0, not 42. Verified: this machine received a topic published
# from the Jetson over wifi on domain 0.
#
# A 640x480 BGR image at 15 Hz is ~110 Mbit/s over the same wifi the drone is
# using. Fine for a look; do not leave it open during a flight.
set -euo pipefail
cd "$(dirname "$0")/.."
TOPIC="${1:?usage: view_remote.sh <topic> [--scale N]}"
shift || true
[ -n "${DISPLAY:-}" ] || { echo "DISPLAY is not set" >&2; exit 1; }
xhost +local: >/dev/null 2>&1 || true

if [ -t 1 ]; then tty_args=(-it); else tty_args=(-i); fi
args=(--rm "${tty_args[@]}" --network host --ipc host -u hydrone
      -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" -e "DISPLAY=$DISPLAY"
      -v /tmp/.X11-unix:/tmp/.X11-unix
      -v "$PWD/scripts:/scripts:ro")
xa="${XAUTHORITY:-$HOME/.Xauthority}"
[ -r "$xa" ] && args+=(-v "$xa:/tmp/.xauth:ro" -e XAUTHORITY=/tmp/.xauth)

echo "viewing $TOPIC on domain ${ROS_DOMAIN_ID:-0}  (q quits, s saves)"
exec docker run "${args[@]}" "${IMAGE:-joao_pessoa_2026-hydrone:latest}" \
    bash -lc ". /opt/ros/humble/setup.sh && exec python3 /scripts/view_topic.py $TOPIC $*"
