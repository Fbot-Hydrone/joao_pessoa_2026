#!/usr/bin/env bash
# View one of the drone's image topics on THIS machine.
#
#   ./scripts/view_remote.sh /hydrone/pads/down/debug_image
#   ./scripts/view_remote.sh /down_cam/image_raw
#   ./scripts/view_remote.sh --wifi /down_cam/image_raw    # if the cable is out
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
# ── WHICH LINK THE PIXELS TAKE ──────────────────────────────────────────────
# There are two paths to the drone and DDS does not choose sensibly between
# them on its own: the shared wifi, and a direct gigabit cable on 10.10.0.0/24.
# A 640x480 BGR image at 15 Hz is ~110 Mbit/s. Over wifi that competes with the
# MAVLink the drone is flying on, which is why this script used to carry a "do
# not leave it open during a flight" warning.
#
#   --cable   (default)  pin DDS to the direct cable. The wifi is left alone
#                        for MAVLink and you can watch images during a flight.
#   --wifi               pin DDS to the wireless interface. For when the cable
#                        is unplugged -- the old behaviour, old caveat.
#   --any                no pinning at all; whatever DDS negotiates.
#
# --cable needs jetson_up.sh to have been started with --cable too, or there is
# nothing publishing on that link. Mismatch looks like an empty window with no
# error, exactly like a domain mismatch does.
set -euo pipefail
cd "$(dirname "$0")/.."
. "$(dirname "$0")/dds_iface.sh"

MODE=cable
TOPIC=""
extra=()
while [ $# -gt 0 ]; do
    case "$1" in
        --cable) MODE=cable; shift ;;
        --wifi)  MODE=wifi;  shift ;;
        --any)   MODE=any;   shift ;;
        -h|--help) sed -n '2,36p' "$0"; exit 0 ;;
        *)
            if [ -z "$TOPIC" ]; then TOPIC="$1"; else extra+=("$1"); fi
            shift ;;
    esac
done
[ -n "$TOPIC" ] || { echo "usage: view_remote.sh [--cable|--wifi|--any] <topic> [--scale N]" >&2; exit 2; }
[ -n "${DISPLAY:-}" ] || { echo "DISPLAY is not set" >&2; exit 1; }
xhost +local: >/dev/null 2>&1 || true

dds_iface_setup "$MODE"

if [ -t 1 ]; then tty_args=(-it); else tty_args=(-i); fi
args=(--rm "${tty_args[@]}" --network host --ipc host -u hydrone
      -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" -e "DISPLAY=$DISPLAY"
      -v /tmp/.X11-unix:/tmp/.X11-unix
      -v "$PWD/scripts:/scripts:ro")
xa="${XAUTHORITY:-$HOME/.Xauthority}"
[ -r "$xa" ] && args+=(-v "$xa:/tmp/.xauth:ro" -e XAUTHORITY=/tmp/.xauth)

# The profile is generated on the host, so it has to be mounted in; Fast DDS
# reads it from the path this variable names.
if [ -n "$DDS_PROFILE" ]; then
    args+=(-v "$DDS_PROFILE:/tmp/dds_profile.xml:ro"
           -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/dds_profile.xml)
    echo "link: $MODE  ($DDS_IFACE $DDS_ADDR)"
else
    echo "link: any (DDS picks; may use the wifi)"
fi

echo "viewing $TOPIC on domain ${ROS_DOMAIN_ID:-0}  (q quits, s saves)"
exec docker run "${args[@]}" "${IMAGE:-joao_pessoa_2026-hydrone:latest}" \
    bash -lc ". /opt/ros/humble/setup.sh && exec python3 /scripts/view_topic.py $TOPIC ${extra[*]:-}"
