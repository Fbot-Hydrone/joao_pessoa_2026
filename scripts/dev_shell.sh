#!/usr/bin/env bash
# Turn this terminal into a shell inside the hydrone container, with rviz2 and
# rqt_image_view running alongside it in the same container.
#
#   ./scripts/dev_shell.sh                    # attach + both GUIs
#   ./scripts/dev_shell.sh -d my_view.rviz    # ... with an rviz config
#
# The rviz config lives on the host (nothing useful is bind-mounted into the
# container), so it is streamed into the container at launch and removed on
# exit. Edits made through rviz "Save Config" land on that throwaway copy and
# are NOT written back to the host file.
#
# The GUIs are started detached (docker exec -d) and write their output to a
# file *inside* the container, which is copied to LOG_DIR on the host when this
# shell exits. Two reasons:
#
#   - Nothing on the host holds a stream open for them, so their lifetime
#     depends on nothing but themselves. (Killing a backgrounded `docker exec`
#     client does NOT kill its container process -- measured, not assumed -- but
#     there is no reason to keep the coupling around either.)
#   - The wrapper deliberately does not exec the GUI: it stays around to record
#     the exit status in a .status file. rviz2 has been dying mid-session with
#     an abruptly-truncated log and no error message, which means a signal
#     rather than an exception -- but nothing was recording *which* signal.
#     Now cleanup reports it ("rviz2 exited on its own: SIGKILL"), which is what
#     tells apart an OOM kill from a segfault from anything else.
#
# Leaving the shell (exit / Ctrl-D) kills both GUIs.
# Overridable: CONTAINER, IMAGE_TOPIC, LOG_DIR, RVIZ_CONFIG.

set -uo pipefail

CONTAINER="${CONTAINER:-joao_pessoa_2026-hydrone-1}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/hydrone/pads/down/debug_image}"
LOG_DIR="${LOG_DIR:-/tmp/hydrone-dev-shell}"
RVIZ_CONFIG="${RVIZ_CONFIG:-}"

while [ $# -gt 0 ]; do
    case "$1" in
        -d|--rviz-config) RVIZ_CONFIG="${2:-}"; shift 2 ;;
        -h|--help) sed -n '2,29p' "$0" | sed 's/^# \?//'; exit 0 ;;
        *) echo "dev_shell: unknown argument '$1'" >&2; exit 2 ;;
    esac
done

# No config asked for: fall back to the usual one if it is lying around.
if [ -z "$RVIZ_CONFIG" ]; then
    here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    for candidate in \
        "$PWD/rviz_pointc_map.rviz" \
        "$here/../rviz_pointc_map.rviz" \
        "$here/../../rviz_pointc_map.rviz"
    do
        [ -r "$candidate" ] && { RVIZ_CONFIG="$candidate"; break; }
    done
fi

if [ -n "$RVIZ_CONFIG" ] && [ ! -r "$RVIZ_CONFIG" ]; then
    echo "dev_shell: cannot read rviz config '$RVIZ_CONFIG'" >&2
    exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "dev_shell: container '$CONTAINER' is not running" >&2
    exit 1
fi

mkdir -p "$LOG_DIR"
tag="$$"
prefix="/tmp/hydrone-dev-shell.$tag"   # in-container pid/log/status files
guis=()
container_cfg=""

# start_gui <name> <command...>
# Runs the command detached in the container with ROS sourced, in its own
# session so the whole process group can be torn down later (ros2 run forks a
# child, so killing just the launcher would orphan it). The session leader
# writes its own PID before doing anything slow, so the pidfile holds the PID
# that is also the process-group id. It deliberately does NOT exec the command:
# staying around to record the exit status is the whole point.
start_gui() {
    local name="$1"; shift

    if ! docker exec -d -u hydrone "$CONTAINER" setsid bash -c \
        "echo \$\$ > $prefix.$name.pid
         { . /opt/ros/humble/setup.sh && . /ws/install/setup.sh && $*; } \
             > $prefix.$name.log 2>&1
         echo \$? > $prefix.$name.status"
    then
        echo "dev_shell: failed to start $name" >&2
        return 1
    fi

    guis+=("$name")
    printf '  %-16s log: %s\n' "$name" "$LOG_DIR/$name.$tag.log"
}

cleanup() {
    trap - EXIT INT TERM
    [ ${#guis[@]} -eq 0 ] && return
    echo
    echo "dev_shell: stopping GUIs..."

    # One pass inside the container: report anything that had already exited on
    # its own (read the status BEFORE killing, or our own TERM would mask it),
    # then TERM each GUI's whole process group and KILL whatever ignored it
    # (rqt_image_view does).
    docker exec -i -u hydrone -e PREFIX="$prefix" "$CONTAINER" \
        bash -s -- "${guis[@]}" <<'CLEAN' 2>/dev/null
pids=()
for name in "$@"; do
    status=$(cat "$PREFIX.$name.status" 2>/dev/null)
    if [ -n "$status" ]; then
        if [ "$status" -gt 128 ] 2>/dev/null; then
            sig=$((status - 128))
            echo "  $name exited on its own: SIG$(kill -l "$sig" 2>/dev/null || echo "$sig")"
        else
            echo "  $name exited on its own: status $status"
        fi
        continue
    fi
    # the pidfile is written first thing, but a very short session can outrun it
    for _ in $(seq 20); do [ -s "$PREFIX.$name.pid" ] && break; sleep 0.1; done
    pid=$(cat "$PREFIX.$name.pid" 2>/dev/null)
    if [ -z "$pid" ]; then
        echo "  $name never started"
    elif ! kill -0 "$pid" 2>/dev/null; then
        # gone, but the wrapper never got to write a status: it was killed too,
        # which points at something that takes out the whole process group
        echo "  $name died without recording an exit status (killed group?)"
    else
        kill -TERM -- "-$pid" 2>/dev/null
        pids+=("$pid")
    fi
done
sleep 2
for pid in "${pids[@]}"; do
    kill -KILL -- "-$pid" 2>/dev/null
done
CLEAN

    # Bring the logs back out to the host, then drop the container-side files.
    for name in "${guis[@]}"; do
        docker exec -u hydrone "$CONTAINER" cat "$prefix.$name.log" \
            > "$LOG_DIR/$name.$tag.log" 2>/dev/null
    done
    docker exec -u hydrone -e CFG="$container_cfg" -e PREFIX="$prefix" \
        "$CONTAINER" bash -c '[ -n "$PREFIX" ] && rm -f "$PREFIX".* ${CFG:+"$CFG"}' \
        >/dev/null 2>&1

    echo "dev_shell: logs in $LOG_DIR"
}
trap cleanup EXIT INT TERM

echo "dev_shell: starting GUIs in $CONTAINER"

rviz_args=()
if [ -n "$RVIZ_CONFIG" ]; then
    container_cfg="$prefix.rviz"
    # Written by hydrone itself, so no ownership surprises (docker cp would land
    # it as root).
    if ! docker exec -i -u hydrone "$CONTAINER" bash -c "cat > $container_cfg" \
            < "$RVIZ_CONFIG"; then
        echo "dev_shell: failed to copy rviz config into the container" >&2
        container_cfg=""
        exit 1
    fi
    rviz_args=(-d "$container_cfg")
    echo "  rviz config      $RVIZ_CONFIG -> $container_cfg"
fi

start_gui rviz2 rviz2 "${rviz_args[@]}"
start_gui rqt_image_view ros2 run rqt_image_view rqt_image_view "$IMAGE_TOPIC"
echo "  live log:        docker exec -u hydrone $CONTAINER tail -f $prefix.<gui>.log"
echo

# Foreground, so this terminal *is* the container shell. When it exits, the trap
# above tears the GUIs down.
docker exec -u hydrone -it "$CONTAINER" bash -c 'source /ws/install/setup.bash && exec bash'
