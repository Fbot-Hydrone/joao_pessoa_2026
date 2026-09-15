#!/usr/bin/env bash
# Fly the SAME seeds under SEVERAL configurations, and put the scores side by
# side. seed_sweep.sh answers "does this work across arenas"; this answers
# "which of these variants should we keep".
#
#   scripts/param_sweep.sh                       # all configs, seeds 1..8
#   scripts/param_sweep.sh --configs base,fast   # a subset
#   SEEDS="1 2 3" scripts/param_sweep.sh         # fewer seeds
#
# WHY A SEPARATE SCRIPT. Comparing variants needs the SAME arenas under each
# one — a config that looks better because it drew easier layouts is worse than
# no measurement at all. Every config here flies the identical seed list, and
# the summary is per (config, seed) so the pairing survives into the analysis.
#
# BUDGET. ~15 min of wall clock per seed, so one config over 8 seeds is ~2 h.
# Four configs is most of a working day. It runs unattended and every run is
# scored as it finishes, so a sweep that is interrupted still leaves usable
# rows behind.
set -u
cd "$(dirname "$0")/.."

: "${BS_SIM_DIR:=/home/lh/Documents/biguasim-competicao/bs-drone-competition}"
export BS_SIM_DIR
SEEDS="${SEEDS:-1 2 3 4 5 6 7 8}"
RUN_TIMEOUT="${RUN_TIMEOUT:-1500}"
CONFIGS="base,fast,fast_acc,fast_settle,early_land"

while [ $# -gt 0 ]; do
    case "$1" in
        --configs) CONFIGS="$2"; shift 2 ;;
        *) echo "argumento desconhecido: $1" >&2; exit 2 ;;
    esac
done

# Each config is: the SITL parameter overlay, then the launch arguments.
# Keeping them in one place means the summary can name exactly what flew.
config_overlay() {
    case "$1" in
        base)         echo "" ;;
        fast)         echo "fast" ;;
        fast_acc)     echo "fast_acc" ;;
        fast_settle)  echo "fast_acc" ;;
        early_land)   echo "fast_acc" ;;
    esac
}
config_args() {
    case "$1" in
        base)         echo "" ;;
        fast)         echo "" ;;
        fast_acc)     echo "" ;;
        fast_settle)  echo "settle_still_speed:=0.15 dwell_s:=2.0" ;;
        early_land)   echo "settle_still_speed:=0.15 dwell_s:=2.0 land_during_survey_min_level:=1" ;;
    esac
}

OUT="logs/param_sweep/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
printf 'config\tseed\tbases\tdetectadas\tpousos\tpousos_validos\terro_mapa_m\tdesfecho\tsegundos\n' > "$OUT/summary.tsv"

n_cfg=$(echo "$CONFIGS" | tr ',' ' ' | wc -w)
n_seed=$(echo "$SEEDS" | wc -w)
echo "Benchmark: $n_cfg configuracao(oes) x $n_seed seed(s) = $((n_cfg * n_seed)) corridas"
echo "Orcamento: ate $(( n_cfg * n_seed * RUN_TIMEOUT / 3600 )) h"
echo "Saida: $OUT"
echo

for cfg in $(echo "$CONFIGS" | tr ',' ' '); do
    overlay=$(config_overlay "$cfg")
    args=$(config_args "$cfg")
    echo "════ config $cfg  (overlay='${overlay:-nenhum}' args='${args:-nenhum}')"
    for seed in $SEEDS; do
        log="$OUT/${cfg}_seed_${seed}.log"
        echo "── $cfg / seed $seed"
        docker compose down --remove-orphans >/dev/null 2>&1
        t0=$(date +%s)

        # Backgrounded rather than run under `timeout`, for the reason
        # seed_sweep.sh gives: a killed `compose up` leaves exit 137 behind and
        # that is indistinguishable from a simulator crash in the log.
        BASES_SEED="$seed" SITL_PARAMS_OVERLAY="$overlay" \
            ./scripts/docker_up.sh --phase1 --ground-truth --no-build $args 2>&1 \
            | stdbuf -oL grep --line-buffered -E \
              'MAP SWEEP|SEARCH LEVEL|pad_map|phase1_mission|bases spawnadas|Traceback|parameter overlay' \
            > "$log" &
        up_pid=$!

        waited=0
        while [ $waited -lt "$RUN_TIMEOUT" ]; do
            grep -qE 'mission complete|ABORTED|Traceback' "$log" 2>/dev/null && break
            if ! docker ps --format '{{.Names}}' | grep -q joao_pessoa_2026-hydrone; then
                sleep 10
                [ $waited -gt 60 ] && break
            fi
            sleep 10
            waited=$((waited + 10))
        done

        kill "$up_pid" 2>/dev/null
        docker compose down --remove-orphans >/dev/null 2>&1
        secs=$(( $(date +%s) - t0 ))

        row=$(python3 scripts/score_run.py --seed "$seed" --log "$log" --tsv)
        printf '%s\t%s\t%s\n' "$cfg" "$row" "$secs" >> "$OUT/summary.tsv"
        echo "   -> $row  (${secs}s de parede)"
    done
    echo
done

echo "════════════════════════════════════════════════════════════"
column -t -s"$(printf '\t')" "$OUT/summary.tsv"
echo
echo "Detalhe por corrida em $OUT/"
