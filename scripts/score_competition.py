#!/usr/bin/env python3
"""Score a Phase 1 run the way the CBR 2026 rules score it.

WHY THIS EXISTS. scripts/score_run.py counts landings and how many were on a
real base. Useful for debugging, and the WRONG objective to tune against,
because it treats a run's landings as a bag. The rules do not:

  +20   each base visited for the FIRST time
   -5   each repeat landing on a base already visited
   x2   returning autonomously to the takeoff base and landing on it
        (only if the score is positive)
   x2   open-hardware drone
  MAX   480 = 6 bases x 20, doubled, doubled

  * landing OUTSIDE a base ENDS THE ATTEMPT *

That last line is what makes this a different problem. An off-base landing is
not one lost base out of six — it is the end of the run, and it also forfeits
the x2 for coming home, which is worth as much as all six bases together. A
cautious run that visits four bases and flies home scores 4*20*2*2 = 320. A
greedy run that visits five and then puts a leg off the edge scores at most
5*20*2 = 200 with no way home, and 0 if it never scored before going off.

So the order of the landings matters, and "how many were valid" does not
describe the run at all.

MATCHING, and its limit: a landing is attributed to a base by RESTING HEIGHT,
the same way score_run.py does it — the arena's bases sit at known distinct
heights, so z identifies which one. A landing on the FLOOR beside a base whose
top is near ground level is therefore indistinguishable from a landing on it.
That makes this scorer OPTIMISTIC on ground-level bases, and it is the reason
`--strict` exists: it treats any landing it cannot attribute confidently as
off-base, which brackets the true score from below.
"""

import argparse
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# The heights, the frame conversion and the landing regex all come from
# score_run, deliberately. A base's height in sample_bases is measured from the
# arena FLOOR while the mission's z is measured from the TAKEOFF BASE, and
# score_run.truth() is where that conversion lives. Re-deriving it here is how
# the first version of this file scored every run as an off-base landing.
from score_run import truth, LAND_CLEARANCE, LAND_TOL, RE_LANDED  # noqa: E402
RE_FINAL = re.compile(r"LANDED \(final\)")
RE_COMPLETE = re.compile(r"mission complete")


def score(seed, path, strict=False, open_hardware=True):
    text = open(path, errors="replace").read() if os.path.exists(path) else ""
    if not text.strip():
        return dict(seed=seed, pontos=0, bases=0, nota="sem log")

    bases = truth(seed)
    landings = [float(z) for z in RE_LANDED.findall(text)]

    pontos, visited, ended, why = 0, set(), False, "fim da busca"
    for z in landings:
        b = min(range(len(bases)),
                key=lambda i: abs(z - (bases[i][2] + LAND_CLEARANCE)))
        d = abs(z - (bases[b][2] + LAND_CLEARANCE))
        if d > (LAND_TOL / 2 if strict else LAND_TOL):
            ended, why = True, f"POUSOU FORA (z={z:.2f}, nada a {LAND_TOL} m)"
            break
        if b in visited:
            pontos -= 5
        else:
            pontos += 20
            visited.add(b)

    # The x2 for coming home is only earned if the run actually got there.
    home = bool(RE_COMPLETE.search(text)) and bool(RE_FINAL.search(text))
    if home and not ended and pontos > 0:
        pontos *= 2
    if open_hardware:
        pontos *= 2
    return dict(seed=seed, pontos=pontos, bases=len(visited), casa=home,
                nota=why if ended else ("voltou" if home else "nao voltou"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--strict", action="store_true")
    ap.add_argument("--tsv", action="store_true")
    a = ap.parse_args()
    r = score(a.seed, a.log, a.strict)
    if a.tsv:
        print(f"{r['seed']}\t{r['pontos']}\t{r['bases']}\t{r.get('casa','')}\t{r['nota']}")
    else:
        print(f"seed {r['seed']}: {r['pontos']} pontos "
              f"({r['bases']} base(s), {r['nota']})")


if __name__ == "__main__":
    main()
