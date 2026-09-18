#!/usr/bin/env python3
"""Onde os 10 minutos da Fase 1 foram gastos.

    scripts/time_breakdown.py logs/seed_sweep/<data>/seed_1.log

Le as transicoes `[A -> B]` que o phase1_mission_node ja imprime e soma quanto
tempo o veiculo passou em cada estado. Nao precisa de instrumentacao nova: a
maquina de estados ja diz tudo, so nunca ninguem somou.

POR QUE ISSO E O NUMERO CERTO. A missao encosta no teto de 600 s e a discussao
sobre como acelera-la sempre foi sobre o que PARECE lento (a descida, o giro).
Dois resultados negativos ja custaram uma sessao cada por terem sido afinados
sem esta tabela na mao: `WP_SPD` para 1,5 PIOROU o perimetro (commit 37db0169) e
quatro parametros de descida nao moveram o LAND (holybro_sitl.parm). Medir onde
o tempo esta antes de mexer em numero e mais barato que as duas juntas.

O corte por CICLO importa tanto quanto o corte por estado: seis pousos de 35 s
sao 210 s, e isso nao aparece como um estado caro, aparece como seis baratos.
"""

import argparse
import re
import sys
from collections import defaultdict

RE_TRANS = re.compile(
    r"\[(\d{10}\.\d+)\].*phase1_mission\]: \[(\w+) -> (\w+)\]")
# Marcos que nao sao transicao de estado mas delimitam um ciclo de pouso.
RE_TOUCH = re.compile(r"\[(\d{10}\.\d+)\].*phase1_mission\]: touchdown at")
RE_LANDED = re.compile(
    r"\[(\d{10}\.\d+)\].*phase1_mission\]: LANDED .*\(([^)]*)\)")
RE_AIRBORNE = re.compile(r"\[(\d{10}\.\d+)\].*phase1_mission\]: airborne —")


def parse(path):
    trans, touch, landed, airborne = [], [], [], []
    for line in open(path, errors="replace"):
        m = RE_TRANS.search(line)
        if m:
            trans.append((float(m.group(1)), m.group(2), m.group(3)))
            continue
        m = RE_TOUCH.search(line)
        if m:
            touch.append(float(m.group(1)))
            continue
        m = RE_LANDED.search(line)
        if m:
            landed.append((float(m.group(1)), m.group(2)))
            continue
        m = RE_AIRBORNE.search(line)
        if m:
            airborne.append(float(m.group(1)))
    return trans, touch, landed, airborne


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    a = ap.parse_args()

    trans, touch, landed, airborne = parse(a.log)
    if len(trans) < 2:
        print(f"{a.log}: sem transicoes de estado — a corrida nem comecou.")
        return 1

    # Tempo por estado: cada transicao fecha o estado anterior.
    spent = defaultdict(float)
    visits = defaultdict(int)
    for (t0, _, s), (t1, _, _) in zip(trans, trans[1:]):
        spent[s] += t1 - t0
        visits[s] += 1
    # O primeiro estado do log tambem contou algum tempo antes da 1a transicao,
    # mas nao sabemos quando a corrida comecou; ignorado de proposito.

    total = trans[-1][0] - trans[0][0]
    print(f"\n{a.log}")
    print(f"voo medido: {total:.0f} s "
          f"({trans[0][0]:.0f} -> {trans[-1][0]:.0f})\n")

    print(f"{'estado':<10} {'total':>8} {'%':>6} {'visitas':>8} {'media':>8}")
    print("-" * 44)
    for s in sorted(spent, key=lambda k: -spent[k]):
        print(f"{s:<10} {spent[s]:>7.1f}s {100*spent[s]/total:>5.1f}% "
              f"{visits[s]:>8} {spent[s]/visits[s]:>7.1f}s")
    print("-" * 44)
    print(f"{'TOTAL':<10} {sum(spent.values()):>7.1f}s")

    # O ciclo de pouso, que e o que seis bases multiplicam.
    if touch:
        print(f"\nciclo de pouso ({len(touch)} toque(s)):")
        print(f"  {'#':<3} {'disarm':>8} {'prova':>26} {'toque->no ar':>14}")
        for i, t in enumerate(touch, 1):
            ld = next((x for x in landed if x[0] >= t), None)
            up = next((x for x in airborne if x >= t), None)
            d = f"{ld[0]-t:.1f}s" if ld else "-"
            how = ld[1] if ld else "-"
            cyc = f"{up-t:.1f}s" if up else "- (fim)"
            print(f"  {i:<3} {d:>8} {how:>26} {cyc:>14}")

    # Quanto do voo foi gasto ENTRE bases, que e o alvo real de otimizacao.
    ground = sum(spent[s] for s in ("LAND", "DISARM", "DWELL", "ARMING",
                                    "TAKEOFF") if s in spent)
    print(f"\npousar/rearmar/subir: {ground:.0f}s ({100*ground/total:.0f}% do voo)")
    search = sum(spent[s] for s in ("SETTLE", "TRAVEL", "CONFIRM", "SELECT")
                 if s in spent)
    print(f"procurar/viajar:      {search:.0f}s ({100*search/total:.0f}% do voo)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
