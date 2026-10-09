#!/usr/bin/env bash
# Gera o dataset inteiro: uma condicao por processo (o BiguaSim nao libera
# bem o ambiente dentro do mesmo processo). Uso:
#   scripts/kit_dataset/run_all.sh ~/Documents/kit_dataset_sim/raw 150
set -u
OUT="${1:-$HOME/Documents/kit_dataset_sim/raw}"
VIEWS="${2:-150}"
source ~/anaconda3/etc/profile.d/conda.sh
conda activate biguasim-comp
cd "$(dirname "$0")"
for c in 0 1 2 3 4 5 6; do
    rm -f /dev/shm/sem.HOLODECK* /dev/shm/HOLODECK_MEM* 2>/dev/null
    timeout 1800 python -u generate.py --out "$OUT" --views "$VIEWS" \
        --conditions "$c" 2>&1 | grep -E --line-buffered "^\[c|^c0|Error|Trace"
    pkill -x Holodeck 2>/dev/null; sleep 5
done
rm -f /dev/shm/sem.HOLODECK* /dev/shm/HOLODECK_MEM* 2>/dev/null
echo "ALL DONE"
