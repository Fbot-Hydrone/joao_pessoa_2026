#!/bin/bash
# Phase 3 in ONE command on the mini's Jetson Nano, like orin_flight/run_phase1.sh.
# Everything runs in tmux session "p3"; this script waits until the countdown
# is running on the drone, prints COUNTDOWN and exits. After that nothing
# depends on ssh (FbotDrone can drop).
#
#   ./run_phase3.sh            full Phase 3
#   NOGO=1 ./run_phase3.sh     rehearsal: every check, never the start call
#   COUNTDOWN=20 ./run_phase3.sh
#
# The drone sits on its takeoff base before this runs. The operator and every
# command read off the gestures print in window "stack" (tmux attach -t p3):
# that is the terminal the judge looks at.
# Stack: jetson_up.sh --phase3 = phase4_real (Super-LIO coarse, MAVROS, LEDs,
# no QR) + phase3_hri_node + gesture_camera. No bag: the logs below and the .BIN.
#
# Code: cbr/phase3 in ~/cbr2026/jp_phase3, image hydrone-jetson-lio:phase3
# (deploy.sh 3 from the laptop). Settings in the block below, here AND in the
# laptop's copy, never on the command line.
set -e

# ── arena ───────────────────────────────────────────────────────────────────
TAKEOFF_ALT=1.5                 # m above the takeoff base; the camera has to frame the operator
APPROACH_M=2.0                  # straight ahead after takeoff
TURN_DEG=90                     # then turn this much to the RIGHT to face the operator
VEL=0.3                         # m/s na horizontal enquanto o gesto e mantido (era 0.5)
VEL_Z=0.2                       # m/s na vertical (SUBIR/DESCER; era 0.3)
TARGET_BASES=6                  # landings before flying home by itself
CAMERA=""                       # "" = the first /dev/v4l/by-id camera
CAMERA_FLIP=false               # true if DIREITA/ESQUERDA come out swapped
LEDS=white                      # one word (jetson_up splits on spaces); none = off
GPU=${GPU:-1}                   # 1 = YOLOv8n-pose na GPU (pose_gpu.py, ~20 fps, acerta braco p/ cima com o colete); 0 = MediaPipe lite no container (~7.6 fps)
VEST=${VEST:-1}                  # 1 = operador e so quem veste o colete (vest.json; calibrar: touch /tmp/vest_calibrate (GPU=1) ou ~/cbr2026/jp_phase3/maps/vest_calibrate (GPU=0)); 0 = qualquer pessoa
LIO_MAP=false                   # lio_map_node: nothing reads it, costs ~45% of a core and grows in memory
COUNTDOWN=${COUNTDOWN:-0}       # s after READY + the FCU checks
# ────────────────────────────────────────────────────────────────────────────

PH=3
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="${WS:-$HOME/cbr2026/jp_phase$PH}"
IMAGE="hydrone-jetson-lio:phase$PH"
export CONTAINER=hydrone-jetson COUNTDOWN NOGO
LOG="$HERE/logs/p${PH}_$(date +%Y%m%d_%H%M%S)"
die() { echo "NOT STARTED: $*"; exit 1; }

[ -f "$WS/src/hydrone_bringup/launch/phase3_real.launch.py" ] || die "no $WS: run deploy.sh $PH from the laptop"
grep -q '"qr": "false"' "$WS/src/hydrone_bringup/launch/phase3_real.launch.py" \
    || die "$WS phase3_real does not turn the QR reader off (it would take the camera): redeploy cbr/phase3"
docker image inspect "$IMAGE" >/dev/null 2>&1 || die "no $IMAGE: run deploy.sh $PH from the laptop"
grep -q "'lio_map'" "$WS/src/hydrone_bringup/launch/phase4_real.launch.py" || die "$WS phase4_real has no lio_map argument: redeploy"
[ -f ~/nano_observe/tools/preflight.py ] || die "no ~/nano_observe/tools (nano_observe/deploy.sh)"
[ -n "$CAMERA" ] || CAMERA=$(ls /dev/v4l/by-id/*-video-index0 2>/dev/null | head -1 || true)
[ -n "$CAMERA" ] && [ -e "$CAMERA" ] || die "no camera (/dev/v4l/by-id): the gestures need it"
[ -e /dev/ttyTHS1 ] || die "no /dev/ttyTHS1 (the Cube's UART)"
ping -c1 -W2 192.168.1.145 >/dev/null || die "the Livox (192.168.1.145) does not answer"

"$HERE/stop.sh" || exit 1
mkdir -p "$LOG"
ARGS="auto_start:=false takeoff_alt:=$TAKEOFF_ALT approach_forward_m:=$APPROACH_M approach_turn_deg:=$TURN_DEG"
CAM_ARG=$CAMERA; [ "$GPU" = 1 ] && CAM_ARG=udp:5600
ARGS="$ARGS vel_speed:=$VEL vel_speed_z:=$VEL_Z"
ARGS="$ARGS target_bases:=$TARGET_BASES camera:=$CAM_ARG camera_flip:=$CAMERA_FLIP leds:=$LEDS"
# MAVROS forwards the FCU to 14550 locally, for gyrocal.py / preflight.py on the host
VP=${VIEW:-0}; [ "$GPU" = 1 ] && VP=0   # com GPU a imagem sai do pose_gpu.py
ARGS="$ARGS view_port:=$VP pose_model:=0 vest:=$([ "$VEST" = 1 ] && echo true || echo false)"   # VIEW=8090 ./run_phase3.sh: imagem com esqueleto no navegador (bancada)
ARGS="$ARGS lio_map:=$LIO_MAP gcs_url:=udp://:14556@127.0.0.1:14550"
cat > "$LOG/settings" <<EOF
TAKEOFF_ALT=$TAKEOFF_ALT APPROACH_M=$APPROACH_M TURN_DEG=$TURN_DEG TARGET_BASES=$TARGET_BASES VEL=$VEL VEL_Z=$VEL_Z
CAMERA=$CAMERA GPU=$GPU VEST=$VEST FLIP=$CAMERA_FLIP LEDS=$LEDS LIO_MAP=$LIO_MAP COUNTDOWN=$COUNTDOWN NOGO=${NOGO:-0}
image $IMAGE $(docker image inspect -f '{{.Created}}' "$IMAGE")
commit $(git -C "$WS" log -1 --format='%h %s' 2>/dev/null)
EOF

# clocks de CPU/GPU/memoria travados no maximo (pose e LIO mais rapidos; esquenta mais)
sudo -n jetson_clocks 2>/dev/null && echo "jetson_clocks: ligado" || echo "WARNING: jetson_clocks falhou (sudo?)"

# camera: foco travado no infinito (o autofoco da C922 fica cacando) e sem cair o fps no escuro
CAMDEV=$(readlink -f "$CAMERA")
v4l2-ctl -d "$CAMDEV" -c focus_auto=0 -c focus_absolute=${FOCUS:-0} -c exposure_auto_priority=0 2>/dev/null && echo "camera: foco ${FOCUS:-0} travado" || echo "WARNING: nao travou o foco da camera (v4l2-ctl)"

# 0. pose do operador na GPU (fora do Docker), antes da stack
if [ "$GPU" = 1 ]; then
    tmux new-session -d -s p$PH -n pose "bash -c 'VEST=$VEST python3 -u $HERE/pose_gpu.py $CAMERA ${VIEW:-0} 2>&1 | tee $LOG/pose.log; exec bash'"
    t0=$(date +%s)
    until grep -q "pose_gpu\] camera" "$LOG/pose.log" 2>/dev/null; do
        [ $(( $(date +%s) - t0 )) -gt 90 ] && die "pose_gpu.py nao subiu em 90 s (tmux attach -t p$PH, janela pose; ou GPU=0)"
        sleep 1
    done
    echo "pose na GPU: $(grep -m1 "pose_gpu\] camera" $LOG/pose.log)"
    tmux new-window -t p$PH -n stack "bash -c 'cd $WS; IMAGE_LIO=$IMAGE ./scripts/jetson_up.sh --phase3 $ARGS 2>&1 | tee $LOG/stack.log; exec bash'"
else
# 1. the whole stack
tmux new-session -d -s p$PH -n stack "bash -c 'cd $WS; IMAGE_LIO=$IMAGE ./scripts/jetson_up.sh --phase3 $ARGS 2>&1 | tee $LOG/stack.log; exec bash'"
fi

# 2. READY (mission in WAIT_START), FCU checks, countdown, start: on the drone
tmux new-window -t p$PH -n go "bash -c '$HERE/go.sh $LOG /hydrone/phase3/start --status /hydrone/phase3/status --status-ready WAIT_START; exec bash'"

echo "log: $LOG"
echo "waiting for the LIO, the EKF home and the mission (~1-2 min, keep the drone still)..."
t0=$(date +%s)
until grep -q "COUNTDOWN" "$LOG/gate.log" 2>/dev/null; do
    grep -q "ABORT" "$LOG/gate.log" 2>/dev/null && { grep "ABORT\|waiting" "$LOG/gate.log" | tail -3; die "tmux attach -t p$PH"; }
    [ $(( $(date +%s) - t0 )) -gt 420 ] && die "no countdown after 420 s (tmux attach -t p$PH)"
    sleep 1
done
grep -m1 -E "camera '.*' open|keypoints da GPU" "$LOG/stack.log" || echo "WARNING: no 'camera open' from gesture_camera in stack.log"
grep -q "LIO LOST" "$LOG/stack.log" && echo "WARNING: LIO LOST in stack.log"
grep -q "no hydrone_leds" "$LOG/stack.log" && echo "WARNING: LEDs off (no hydrone_leds in $IMAGE)"
echo "COUNTDOWN RUNNING: start in $COUNTDOWN s$([ "${NOGO:-0}" = 1 ] && echo ' (NOGO: it will not start)'). tmux attach -t p$PH, window stack, for the judge."
