#!/usr/bin/env python3
"""
score_run — turn one mission log into a row of numbers, scored against truth.

    python3 scripts/score_run.py --seed 10 --log logs/seed_sweep/.../seed_10.log

WHERE THE TRUTH COMES FROM. Not annotation: `biguasim_main.bases.sample_bases`
is the same function the simulator calls to place the bases, so for a given
seed it reproduces the arena exactly. That is what makes "found 5 of 6" a
measurement instead of an impression.

WHAT COUNTS AS A LANDING ON A BASE. Not the mission saying it landed — the
mission cannot tell a base from the floor beside it. A base of height `h` has
its top at `h + ground_z`, and the vehicle comes to rest LAND_CLEARANCE above
that, a signature measured across many runs at 0.12-0.14 m. A landing whose
resting altitude matches some base's top to within a few centimetres is on that
base; one that does not is on the floor, and earlier runs that reported six
landings had erred by 1.30 m on some of them and were indistinguishable by any
other criterion.

Frames: the simulator places bases in WORLD coordinates and the mission flies
in the FCU's local ENU, which is that world turned 90 degrees —
`map = (-y_world, x_world)`, and z measured from the top of the takeoff base
rather than the floor.
"""

import argparse
import math
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src",
                                "biguasim-ros2", "biguasim_main"))

from biguasim_main.bases import sample_bases            # noqa: E402

# Where the arena floor sits in the mission's frame. The map's origin is the
# TOP of the base the drone armed on, so the floor is below zero.
GROUND_Z = -0.7
# How far above a base's top the airframe comes to rest. Measured 0.12-0.14 m.
LAND_CLEARANCE = 0.13
# How close a resting altitude has to be to count as that base.
LAND_TOL = 0.08

RE_LANDED = re.compile(r"LANDED on base #\d+ of \d+ — resting at z=([-\d.]+)")
RE_CONFIRMED = re.compile(r"pad (\d+): CONFIRMED at \(([-\d.]+), ([-\d.]+)\)")
RE_SPAWNED = re.compile(r"(\d+) bases spawnadas \(seed (\d+)\)")


def truth(seed, count=6):
    """The real bases for this seed, in the MISSION's frame."""
    return [(-y, x, z + GROUND_Z) for x, y, z in sample_bases(count, seed)]


def score(seed, path):
    text = open(path, errors="replace").read() if os.path.exists(path) else ""
    bases = truth(seed)

    # The sim's own line, so a seed that never spawned is not scored as a miss.
    spawned = RE_SPAWNED.search(text)
    n_spawned = int(spawned.group(1)) if spawned else 0
    if spawned and int(spawned.group(2)) != seed:
        return dict(seed=seed, note=f"log is seed {spawned.group(2)}, not {seed}")

    # Last CONFIRMED position per pad id — the map's final answer.
    confirmed = {}
    for pid, x, y in RE_CONFIRMED.findall(text):
        confirmed[pid] = (float(x), float(y))

    # Which real bases the map found. The takeoff base is registered rather
    # than detected, so a pad sitting on it is not a find.
    found, errs = set(), []
    for x, y in confirmed.values():
        b = min(range(len(bases)),
                key=lambda i: math.hypot(x - bases[i][0], y - bases[i][1]))
        d = math.hypot(x - bases[b][0], y - bases[b][1])
        if d <= 1.0:                      # same base, however coarsely placed
            found.add(b)
            errs.append(d)

    landings = [float(z) for z in RE_LANDED.findall(text)]
    valid = []
    for z in landings:
        b = min(range(len(bases)),
                key=lambda i: abs(z - (bases[i][2] + LAND_CLEARANCE)))
        if abs(z - (bases[b][2] + LAND_CLEARANCE)) <= LAND_TOL:
            valid.append(b)

    if "Traceback" in text:
        outcome = "CRASH"
    elif "mission complete" in text:
        outcome = "completa"
    elif "ABORTED" in text:
        outcome = "abortada"
    elif not text.strip():
        outcome = "sem log"
    else:
        outcome = "timeout"

    return dict(
        seed=seed, spawned=n_spawned, found=len(found), landings=len(landings),
        valid=len(set(valid)), err=(sorted(errs)[len(errs) // 2] if errs else None),
        outcome=outcome, heights=[round(b[2], 2) for b in bases])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--tsv", action="store_true")
    a = ap.parse_args()
    r = score(a.seed, a.log)

    if "note" in r:
        print(f"seed {a.seed}: {r['note']}")
        return
    err = "-" if r["err"] is None else f"{r['err']:.2f}"
    if a.tsv:
        print(f"{r['seed']}\t{r['spawned']}\t{r['found']}\t{r['landings']}\t"
              f"{r['valid']}\t{err}\t{r['outcome']}")
    else:
        print(f"seed {r['seed']:>3}: detectou {r['found']}/6, pousou "
              f"{r['landings']} ({r['valid']} em base de verdade), erro do mapa "
              f"{err} m, {r['outcome']}")
        print(f"          alturas dos topos: {r['heights']}")


if __name__ == "__main__":
    main()
