#!/bin/bash
# Phase 2 in ONE command on the Orin, like run_phase1.sh. Everything runs in
# tmux session "p2"; this script waits until the countdown is running, prints
# COUNTDOWN and exits. After that nothing depends on ssh: pull the cable.
#
#   ./run_phase2.sh            full Phase 2
#   NOGO=1 ./run_phase2.sh     rehearsal: everything but the start call
#
# SEMPRE SEM O SURFACE (2026-11-26): o Pixhawk usa o sensor de altura dele
# direto (PLAIN) e o surface_offset não sobe. Se o Pixhawk estiver no modo
# surface, o script volta ele pro PLAIN sozinho (2 reboots, ~2 min).
# O modo fica em .fcu_height_mode, o mesmo do p1.
#
# Code: cbr/phase2 in ~/Documents/jp_phase2 (deploy_phase2.sh). Settings in
# the block below, here AND in the laptop's copy, never on the command line.
# The gripper homes itself at boot: jaws EMPTY and free when this starts.
set -e

# ── arena ───────────────────────────────────────────────────────────────────
SURFACE=0.0                     # takeoff base top above the floor
ALT=3.0                         # cruise, m above the floor
Z_MIN=0.3; Z_MAX=3.5            # planner box z, m above the floor
WALLS="[6.6, 0.6, 0.8, 6.8]"    # [front, back, left, right] from the drone on the base
# Bases FROM THE TAKEOFF BASE, x right, y forward (nose at arming), m:
# "[x, y]" or "{grid: [col, row]}". Empty = unknown -> the Phase 1 scan first.
KITS=1                          # kits in the arena (1..3): only bases 1..KITS exist
PICKUP_1=""; PICKUP_2=""; PICKUP_3=""
DELIVERY_1=""; DELIVERY_2=""; DELIVERY_3=""
YOLO=cuda:0                     # GPU. NAO usar 0: o ROS le como numero e os detectores morrem (2026-11-26)
COUNTDOWN=${COUNTDOWN:-0}        # 0 = start assim que a missão fica pronta; COUNTDOWN=30 ./run_phase2.sh para dar tempo de se afastar
# ────────────────────────────────────────────────────────────────────────────

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="${WS:-$HOME/Documents/jp_phase2}"
ENV="source /opt/ros/humble/setup.bash; source ~/mavros_ws/install/setup.bash; source $WS/install/setup.bash"
LOG="$HERE/logs/p2_$(date +%Y%m%d_%H%M%S)"
EXTRA=""
NOSURF=1                          # sempre sem o surface (PLAIN)
MODE_FILE="$HERE/.fcu_height_mode"   # surface | plain: o que o Pixhawk tem agora
die() { echo "NOT STARTED: $*"; exit 1; }

[ -f "$WS/install/hydrone_bringup/share/hydrone_bringup/launch/phase2.launch.py" ] \
    || die "no Phase 2 build in $WS: run deploy_phase2.sh from the laptop"
grep -q fcu_range_topic "$WS/install/hydrone_bringup/share/hydrone_bringup/launch/phase2.launch.py" \
    || die "installed phase2 build has no fcu_range_topic: rebuild cbr/phase2"
grep -q '"kits"' "$WS/install/hydrone_bringup/share/hydrone_bringup/launch/phase2.launch.py" \
    || die "installed phase2 build has no kits argument: rebuild cbr/phase2"
ls /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_* >/dev/null 2>&1 \
    || die "no FTDI cable: the gripper's ArbotiX-M is not plugged in"
case "$KITS" in 1|2|3) ;; *) die "KITS=$KITS: has to be 1, 2 or 3" ;; esac
# a base above KITS would be silently ignored: refuse instead
for i in 1 2 3; do
    [ "$i" -le "$KITS" ] && continue
    for n in PICKUP_$i DELIVERY_$i; do
        [ -z "${!n}" ] || die "$n is set but KITS=$KITS"
    done
done

"$HERE/stop.sh" || exit 1
tmux kill-session -t p2 2>/dev/null || true

mkdir -p "$LOG"

# ── Pixhawk height mode: surface (normal) or plain (NOSURFACE=1) ───────────
want=surface; [ "$NOSURF" = 1 ] && want=plain
have=$(cat "$MODE_FILE" 2>/dev/null || true)
if [ "$have" != surface ] && [ "$have" != plain ]; then
    echo "Pixhawk: lendo o modo de altura (~30 s)..."
    m=$(bash -c "$ENV; cd $WS; timeout 90 python3 scripts/fcu_surface_offset.py status" 2>&1 | grep '^mode:' || true)
    case "$m" in
        "mode: SURFACE_OFFSET (Orin"*) have=surface ;;
        "mode: PLAIN"*) have=plain ;;
        *) die "Pixhawk em modo desconhecido ($m): scripts/fcu_surface_offset.py status" ;;
    esac
    echo "$have" > "$MODE_FILE"
fi
if [ "$have" != "$want" ]; then
    [ "$want" = plain ] && act=revert || act=apply
    echo "Pixhawk: $have -> $want ($act, 2 reboots, ~2 min). Drone parado e desarmado..."
    rm -f "$MODE_FILE"
    bash -c "$ENV; cd $WS; timeout 300 python3 scripts/fcu_surface_offset.py $act" > "$LOG/fcu_$act.log" 2>&1 || true
    m=$(grep '^mode:' "$LOG/fcu_$act.log" | tail -1 || true)
    case "$want:$m" in
        "surface:mode: SURFACE_OFFSET (Orin"*|"plain:mode: PLAIN"*) echo "$want" > "$MODE_FILE"; echo "Pixhawk: $m" ;;
        *) tail -5 "$LOG/fcu_$act.log"; die "troca do modo do Pixhawk falhou (log $LOG/fcu_$act.log)" ;;
    esac
fi
if [ "$NOSURF" = 1 ]; then
    SURF_ARGS="surface_offset:=false"
    FLOOR_CMD="python3 floor_plain.py"
    # in PLAIN there is no rangefinder_fcu: the FCU's own sensor is here, and
    # the takeoff climb has to be sent on top of it
    EXTRA="$EXTRA fcu_range_topic:=/mavros/distance_sensor/rangefinder"
else
    SURF_ARGS="takeoff_base_height_m:=$SURFACE calibrate_datum:=false"
    FLOOR_CMD="python3 floor_z.py --surface $SURFACE --log $LOG/floor.csv"
fi

# Only the given bases go on the command line: `ros2 launch` rejects an empty
# `name:=`, and a missing one means "unknown" anyway (the real layout is empty).
POSARGS=""
for n in PICKUP_1 PICKUP_2 PICKUP_3 DELIVERY_1 DELIVERY_2 DELIVERY_3; do
    [ -n "${!n}" ] && POSARGS+="${n,,}:=\"${!n}\" "
done
LAYOUT="$WS/install/hydrone_bringup/share/hydrone_bringup/config/phase2_bases_real.yaml"
echo "$EXTRA" > "$LOG/extra"
cat > "$LOG/settings" <<EOF
NOSURFACE=$NOSURF KITS=$KITS SURFACE=$SURFACE ALT=$ALT Z=[$Z_MIN,$Z_MAX] WALLS=$WALLS YOLO=$YOLO EXTRA=$EXTRA
pickups [$PICKUP_1] [$PICKUP_2] [$PICKUP_3] deliveries [$DELIVERY_1] [$DELIVERY_2] [$DELIVERY_3]
commit $(git -C "$WS" log -1 --format='%h %s' 2>/dev/null)
EOF

# 1. sensors + MAVROS + surface_offset + gripper (homes at boot). The ZED
#    cloud is off by default in sources_real: without it no octomap.
tmux new-session -d -s p2 -n stack "bash -c '$ENV; ros2 launch hydrone_bringup sources_real.launch.py $SURF_ARGS zed_point_cloud:=true gripper:=true 2>&1 | tee $LOG/stack.log; exec bash'"

# 2. ground checks + where the floor is in FCU z
tmux new-window -t p2 -n floor "bash -c '$ENV; cd $HERE; $FLOOR_CMD 2>&1 | tee $LOG/floor.log; exec bash'"

# 3. mission once the floor is known: ground_z and the plan z shifted by it;
#    cruise_alt is above the floor (the mission adds ground_z). The bag and
#    tasks.log go to $LOG/phase2_<date>/.
tmux new-window -t p2 -n mission "bash -c '$ENV; until grep -q \"^FLOOR_Z=\" $LOG/floor.log 2>/dev/null; do sleep 0.5; done; \
F=\$(grep \"^FLOOR_Z=\" $LOG/floor.log | cut -d= -f2); \
Z0=\$(python3 -c \"print(round($Z_MIN+\$F,3))\"); Z1=\$(python3 -c \"print(round($Z_MAX+\$F,3))\"); \
echo \"floor z \$F: cruise $ALT m above it, plan z \$Z0..\$Z1\" | tee $LOG/mission_args; \
ros2 launch hydrone_bringup phase2.launch.py layout_file:=$LAYOUT arena_yaw_deg:=0.0 belly_offset_xy:=\"[0.065, -0.071]\" \
ground_z:=\$F cruise_alt:=$ALT plan_bounds:=\"[0.0, 0.0, \$Z0, 0.0, 0.0, \$Z1]\" arena_walls_m:=\"$WALLS\" \
kits:=$KITS $POSARGS yolo_device:=$YOLO auto_start:=false log_dir:=$LOG $EXTRA 2>&1 | tee $LOG/mission.log; exec bash'"

# 4. countdown + start, on the drone
tmux new-window -t p2 -n go "bash -c '$ENV; until grep -q \"ready, waiting. Call /hydrone/mission/start\" $LOG/mission.log 2>/dev/null; do sleep 0.5; done; \
date +%s > $LOG/countdown; sleep $COUNTDOWN; if [ \"${NOGO:-0}\" = 1 ]; then echo NOGO: not starting | tee $LOG/start.log; else ros2 service call /hydrone/mission/start std_srvs/srv/Trigger 2>&1 | tee $LOG/start.log; fi; exec bash'"

echo "log: $LOG"
[ "$NOSURF" = 1 ] && echo "MODO NOSURFACE: altura direto do sensor do Pixhawk (voa ALT acima do que estiver embaixo)"
echo "waiting for the ground checks (~1 min, keep the drone still)..."
t0=$(date +%s)
until grep -q "^FLOOR_Z=" "$LOG/floor.log" 2>/dev/null; do
    if grep -q "ABORT" "$LOG/floor.log" 2>/dev/null; then
        grep "ABORT" "$LOG/floor.log"; die "ground checks failed (tmux attach -t p2, window floor)"
    fi
    [ $(( $(date +%s) - t0 )) -gt 240 ] && die "no floor result after 240 s"
    sleep 1
done
cat "$LOG/mission_args" 2>/dev/null || true
echo "waiting for the mission (gripper homing, YOLO, nav) to be ready..."
until [ -f "$LOG/countdown" ]; do
    [ $(( $(date +%s) - t0 )) -gt 420 ] && die "mission not ready after 420 s (tmux attach -t p2, window mission)"
    sleep 1
done
grep -m1 "auto_calibrate" "$LOG/stack.log" || echo "WARNING: no gripper homing line in stack.log"
go=$(( $(cat "$LOG/countdown") + COUNTDOWN ))
echo "COUNTDOWN RUNNING. Start at $(date -d @$go +%H:%M:%S), takeoff not before then. Pull the cable."
