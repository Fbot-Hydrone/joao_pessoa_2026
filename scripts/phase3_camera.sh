#!/usr/bin/env bash
# Fly the Phase 3 mission on your own gestures, through the PC's webcam.
#
#   ./scripts/phase3_camera.sh                     # /dev/video0
#   ./scripts/phase3_camera.sh camera:=/dev/video2 flip:=true
#
# Runs gesture_camera (MediaPipe Pose -> frl_core) inside the running hydrone
# container. The container must have been started with the webcam:
#
#   ./scripts/docker_up.sh --phase3 --no-build --webcam
#
# Any name:=value is passed to the node as a ROS parameter. `q` in the preview
# window, or Ctrl+C here, stops it (the drone brakes and holds).
# Overridable: CONTAINER, WEBCAM_DEV.

set -euo pipefail

CONTAINER="${CONTAINER:-joao_pessoa_2026-hydrone-1}"
WEBCAM_DEV="${WEBCAM_DEV:-/dev/video0}"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "phase3_camera: container '$CONTAINER' is not running" \
         "(scripts/docker_up.sh --phase3 --no-build --webcam)" >&2
    exit 1
fi
if ! docker exec "$CONTAINER" test -e "$WEBCAM_DEV"; then
    echo "phase3_camera: $WEBCAM_DEV is not inside the container — it was started" \
         "without --webcam. Restart it: docker compose down, then docker_up.sh ... --webcam" >&2
    exit 1
fi

# The device keeps the host's group id; give the container user that group,
# so the node can open it without running as root.
docker exec -u root -e DEV="$WEBCAM_DEV" "$CONTAINER" bash -c '
    gid=$(stat -c %g "$DEV")
    name=$(getent group "$gid" | cut -d: -f1)
    [ -n "$name" ] || { name=hostvideo; groupadd -g "$gid" "$name"; }
    id -nG hydrone | grep -qw "$name" || usermod -aG "$name" hydrone'

params=(-p "camera:=$WEBCAM_DEV")
for arg in "$@"; do
    params+=(-p "$arg")
done

exec docker exec -it -u hydrone -e DISPLAY="${DISPLAY:-:0}" "$CONTAINER" bash -c \
    '. /opt/ros/humble/setup.sh && . /ws/install/setup.sh && exec ros2 run hydrone_mission gesture_camera --ros-args "$@"' \
    _ "${params[@]}"
