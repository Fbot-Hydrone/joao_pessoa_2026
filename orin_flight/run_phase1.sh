#!/bin/bash
# Phase 1 in ONE command on the Orin. Everything runs in tmux session "p1";
# this script only waits until the countdown is running, prints COUNTDOWN and
# exits. After that nothing depends on ssh: pull the cable and leave.
#
#   ./run_phase1.sh            full Phase 1
#   LOW=1 ./run_phase1.sh      low bases only (max_pad_top_m 0.5)
#   NOGO=1 ./run_phase1.sh     rehearsal: everything but the start call
#   NOSURFACE=1 ./run_phase1.sh  SEM o surface_offset: Pixhawk volta ao sensor
#                              dele direto (PLAIN). Plano B se o surface der erro.
#                              Trocar de modo reinicia o Pixhawk 2x (~2 min);
#                              voltar ao normal (sem a flag) troca de novo.
#
# Arena settings are in the block below. Change them HERE (and in the copy
# on the laptop), not on the command line, so the run is the one we rehearsed.
#
# Height: feed = raw sensor + surface, no datum bias (the bias blocked every
# takeoff on 2026-11-23). The EKF's zero is off by a constant, so floor_z.py
# measures where the floor is in FCU z and the z values below are shifted by it.
set -e

# ── arena (2026-11-24) ──────────────────────────────────────────────────────
SURFACE=0.5                     # height of the takeoff base top above the floor
ALT=2.1                         # cruise, m above the floor
Z_MIN=0.3; Z_MAX=3.5            # planner box z, m above the floor
WALLS="[6.6, 0.6, 0.8, 6.8]"    # [front, back, left, right] from the drone on the base
SIDE=right                      # mapping starts on this side
U_X=5; U_Y=5                    # quadrado 5x5 m centrado na arena (~1,1-1,3 m das paredes); 0 = arena menos 1,2 m por lado; 6x6 bateu na parede da frente 2026-11-24
COUNTDOWN=${COUNTDOWN:-0}        # 0 = start assim que a missão fica pronta; COUNTDOWN=30 ./run_phase1.sh para dar tempo de se afastar
# ────────────────────────────────────────────────────────────────────────────

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="${WS:-$HOME/Documents/joao_pessoa_2026}"
ENV="source /opt/ros/humble/setup.bash; source ~/mavros_ws/install/setup.bash; source $WS/install/setup.bash"
LOG="$HERE/logs/p1_$(date +%Y%m%d_%H%M%S)"
EXTRA=""
[ "${LOW:-0}" = 1 ] && EXTRA="max_pad_top_m:=0.5"
NOSURF="${NOSURFACE:-0}"
MODE_FILE="$HERE/.fcu_height_mode"   # surface | plain: o que o Pixhawk tem agora

die() { echo "NOT STARTED: $*"; exit 1; }

# an installed build without calibrate_datum would run the datum (bias on)
grep -q calibrate_datum "$WS/install/hydrone_bringup/share/hydrone_bringup/launch/sources_real.launch.py" \
    || die "installed build has no calibrate_datum: rebuild cbr/phase1"

"$HERE/stop.sh" || exit 1

mkdir -p "$LOG" ~/warmup_bags

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
else
    SURF_ARGS="takeoff_base_height_m:=$SURFACE calibrate_datum:=false"
    FLOOR_CMD="python3 floor_z.py --surface $SURFACE --log $LOG/floor.csv"
fi
echo "$EXTRA" > "$LOG/extra"
cat > "$LOG/settings" <<EOF
NOSURFACE=$NOSURF SURFACE=$SURFACE ALT=$ALT Z=[$Z_MIN,$Z_MAX] WALLS=$WALLS SIDE=$SIDE U=${U_X}x$U_Y EXTRA=$EXTRA
commit $(git -C "$WS" log -1 --format='%h %s' 2>/dev/null)
EOF

# 1. sensors + MAVROS + surface_offset (feed = raw + SURFACE)
tmux new-session -d -s p1 -n stack "bash -c '$ENV; ros2 launch hydrone_bringup sources_real.launch.py $SURF_ARGS zed_point_cloud:=true 2>&1 | tee $LOG/stack.log; exec bash'"

# 2. ground checks + where the floor is in FCU z
tmux new-window -t p1 -n floor "bash -c '$ENV; cd $HERE; $FLOOR_CMD 2>&1 | tee $LOG/floor.log; exec bash'"

# 3. mission, once the floor is known (args shifted by FLOOR_Z)
tmux new-window -t p1 -n mission "bash -c '$ENV; until grep -q \"^FLOOR_Z=\" $LOG/floor.log 2>/dev/null; do sleep 0.5; done; \
F=\$(grep \"^FLOOR_Z=\" $LOG/floor.log | cut -d= -f2); \
A=\$(python3 -c \"print(round($ALT+\$F,3))\"); Z0=\$(python3 -c \"print(round($Z_MIN+\$F,3))\"); Z1=\$(python3 -c \"print(round($Z_MAX+\$F,3))\"); \
echo \"floor z \$F: takeoff_alt \$A, plan z \$Z0..\$Z1\" | tee $LOG/mission_args; \
ros2 launch hydrone_bringup phase1.launch.py field_mode:=dark_blue belly_offset_xy:=\"[0.065, -0.071]\" \
ground_z:=\$F takeoff_alt:=\$A plan_bounds:=\"[0.0, 0.0, \$Z0, 0.0, 0.0, \$Z1]\" arena_walls_m:=\"$WALLS\" \
sweep_side:=$SIDE u_side_x_m:=$U_X u_side_y_m:=$U_Y auto_start:=false $EXTRA 2>&1 | tee $LOG/mission.log; exec bash'"

# 4. rosbag, from when the mission is up
tmux new-window -t p1 -n rec "bash -c '$ENV; until [ -f $LOG/mission.log ]; do sleep 1; done; sleep 5; cd ~/warmup_bags; \
ros2 bag record -o p1_\$(date +%H%M%S) -e \"/(mavros/(local_position|imu|state|distance_sensor|vision_pose|rc).*|down_cam/.*|hydrone/.*|octomap/.*|tf|tf_static|rosout)\" -x \".*debug_image.*\" 2>&1 | tee $LOG/rec.log; exec bash'"

# 5. countdown + start, on the drone
tmux new-window -t p1 -n go "bash -c '$ENV; until grep -q \"Call /hydrone/mission/start\" $LOG/mission.log 2>/dev/null; do sleep 0.5; done; \
date +%s > $LOG/countdown; sleep $COUNTDOWN; if [ \"${NOGO:-0}\" = 1 ]; then echo NOGO: not starting | tee $LOG/start.log; else ros2 service call /hydrone/mission/start std_srvs/srv/Trigger 2>&1 | tee $LOG/start.log; fi; exec bash'"

echo "log: $LOG"
[ "$NOSURF" = 1 ] && echo "MODO NOSURFACE: altura direto do sensor do Pixhawk (voa ALT acima do que estiver embaixo)"
echo "waiting for the ground checks (~1 min, keep the drone still)..."
t0=$(date +%s)
until grep -q "^FLOOR_Z=" "$LOG/floor.log" 2>/dev/null; do
    if grep -q "ABORT" "$LOG/floor.log" 2>/dev/null; then
        grep "ABORT" "$LOG/floor.log"; die "ground checks failed (tmux attach -t p1, window floor)"
    fi
    [ $(( $(date +%s) - t0 )) -gt 240 ] && die "no floor result after 240 s"
    sleep 1
done
cat "$LOG/mission_args" 2>/dev/null || true
echo "waiting for the mission to be ready..."
until [ -f "$LOG/countdown" ]; do
    [ $(( $(date +%s) - t0 )) -gt 360 ] && die "mission not ready after 360 s (tmux attach -t p1, window mission)"
    sleep 1
done
go=$(( $(cat "$LOG/countdown") + COUNTDOWN ))
echo "COUNTDOWN RUNNING. Start at $(date -d @$go +%H:%M:%S), takeoff not before then. Pull the cable."
