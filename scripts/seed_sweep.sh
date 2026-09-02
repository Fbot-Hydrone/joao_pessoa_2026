#!/usr/bin/env bash
# Fly the SAME mission over MANY base layouts, and score each one.
#
#   scripts/seed_sweep.sh                    # map_sweep, seeds 1..8
#   scripts/seed_sweep.sh 11 12 13           # those seeds
#   MISSION=--phase1 scripts/seed_sweep.sh   # the U mission instead
#
# WHY. Both missions have been measured on ONE arena, seed 10, and tuned while
# looking at it. A threshold that happens to suit six particular base positions
# is indistinguishable from one that generalises, until it is flown on layouts
# nobody looked at. This is what turns "it works" into "it works on 7 of 8".
#
# Each seed is a full bring-up: the sim spawns a different set of six bases
# (biguasim_main.bases.sample_bases, same function this script scores against,
# so the truth is generated rather than annotated), the mission flies, and the
# log is scored. Budget ~15 minutes of wall clock PER SEED — the simulator runs
# well below real time with the cameras on.
#
# Output lands in logs/seed_sweep/<timestamp>/:
#   seed_<n>.log      the filtered run log
#   summary.tsv       one row per seed
#   summary.txt       the same, readable
set -u
cd "$(dirname "$0")/.."

MISSION="${MISSION:---map-sweep}"
ODOM="${ODOM:---ground-truth}"
# Wall-clock ceiling per seed. A run that has not finished by then is recorded
# as TIMEOUT and the sweep moves on rather than stalling on one arena.
RUN_TIMEOUT="${RUN_TIMEOUT:-1500}"
: "${BS_SIM_DIR:=/home/lh/Documents/biguasim-competicao/bs-drone-competition}"
export BS_SIM_DIR

SEEDS=("$@")
[ ${#SEEDS[@]} -eq 0 ] && SEEDS=(1 2 3 4 5 6 7 8)

OUT="logs/seed_sweep/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
echo "seed	bases	detectadas	pousos	pousos_validos	erro_mapa_m	desfecho" > "$OUT/summary.tsv"

echo "Varredura: ${#SEEDS[@]} seed(s), missao $MISSION $ODOM"
echo "Saida: $OUT"
echo "Orcamento: ate $((RUN_TIMEOUT / 60)) min por seed, ~$(( ${#SEEDS[@]} * RUN_TIMEOUT / 60 )) min no total."
echo

for seed in "${SEEDS[@]}"; do
    log="$OUT/seed_${seed}.log"
    echo "── seed $seed ─────────────────────────────────────────────"
    docker compose down --remove-orphans >/dev/null 2>&1

    # The bring-up is backgrounded and killed by the timeout, rather than run
    # under `timeout`, because `docker compose up` attached to a killed shell
    # leaves the container in exit 137 and that LOOKS like a simulator crash.
    BASES_SEED="$seed" ./scripts/docker_up.sh $MISSION $ODOM --no-build 2>&1 \
        | stdbuf -oL grep --line-buffered -E \
          'MAP SWEEP|SEARCH LEVEL|pad_map|phase1_mission|bases spawnadas|Traceback' \
        > "$log" &
    up_pid=$!

    # Poll for a terminal line rather than a fixed sleep: a seed that finishes
    # early should not cost the whole budget.
    waited=0
    while [ $waited -lt "$RUN_TIMEOUT" ]; do
        if grep -qE 'mission complete|ABORTED|Traceback' "$log" 2>/dev/null; then
            break
        fi
        if ! docker ps --format '{{.Names}}' | grep -q joao_pessoa_2026-hydrone; then
            sleep 10                       # give the log a moment to flush
            [ $waited -gt 60 ] && break
        fi
        sleep 10
        waited=$((waited + 10))
    done

    kill "$up_pid" 2>/dev/null
    docker compose down --remove-orphans >/dev/null 2>&1

    python3 scripts/score_run.py --seed "$seed" --log "$log" \
        | tee -a "$OUT/summary.txt"
    python3 scripts/score_run.py --seed "$seed" --log "$log" --tsv \
        >> "$OUT/summary.tsv"
    echo
done

echo "════════════════════════════════════════════════════════════"
column -t -s'	' "$OUT/summary.tsv"
echo
echo "Detalhe por seed em $OUT/"
