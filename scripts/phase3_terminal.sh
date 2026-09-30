#!/usr/bin/env bash
# Be the camera: type the operator's gestures for the Phase 3 mission.
#
#   ./scripts/phase3_terminal.sh            # into the running hydrone container
#   ./scripts/phase3_terminal.sh --host     # on the host (after source scripts/env.sh)
#
# Publishes hydrone_msgs/HumanGesture on /hydrone/vision/human_gesture, exactly
# what the gesture recogniser will publish on the drone. Start the mission
# first (scripts/docker_up.sh --phase3). Type `ajuda` for the commands.
# Overridable: CONTAINER.

set -euo pipefail

CONTAINER="${CONTAINER:-joao_pessoa_2026-hydrone-1}"

if [ "${1:-}" = "--host" ]; then
    exec ros2 run hydrone_mission gesture_terminal
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "phase3_terminal: container '$CONTAINER' is not running" \
         "(scripts/docker_up.sh --phase3, or CONTAINER=...)" >&2
    exit 1
fi

exec docker exec -it -u hydrone "$CONTAINER" bash -c \
    '. /opt/ros/humble/setup.sh && . /ws/install/setup.sh && exec ros2 run hydrone_mission gesture_terminal'
