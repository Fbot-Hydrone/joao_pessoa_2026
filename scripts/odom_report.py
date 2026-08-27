#!/usr/bin/env python3
"""Read an odom_error CSV and say how the localisation actually did.

    scripts/odom_report.py                 # the newest CSV in ./logs
    scripts/odom_report.py logs/a.csv logs/b.csv    # compare two runs

Plain stdlib, so it runs on the host with no ROS and no conda env.

The four numbers that matter, and what each one catches:

  erro mediano   how wrong the estimate is for most of the flight. The final
                 error alone hides a run that was 12 m off in the middle.
  yaw pico       the failure mode this arena produces. Blank walls stop
                 matching mid-turn, the VO holds its pose, and the rotation
                 that really happened is never recorded. It shows up as JUMPS,
                 not drift — which is why the timeline below prints path_len:
                 a yaw step while path_len is frozen is a lost turn.
  escala VO/GT   distance reported over distance flown. Above 1 the VO invents
                 travel (noise integrated while parked); below 1 it loses it
                 (bad depth, so every step is scaled down).
  parado         inflation while the vehicle was still. The ZUPT should hold
                 this at 1.00x; anything above means it is leaking again.
"""

import csv
import glob
import math
import os
import sys


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def num(row, key):
    try:
        return float(row[key])
    except (TypeError, ValueError):
        return 0.0


def report(path):
    rows = read(path)
    if len(rows) < 2:
        print(f"{path}: too few samples")
        return

    err = [num(r, "err_norm") for r in rows]
    yaw = [abs(num(r, "err_yaw_deg")) for r in rows]

    gt = vo = still_gt = still_vo = 0.0
    n_still = 0
    prev = None
    for r in rows:
        p = (num(r, "gt_x"), num(r, "gt_y"), num(r, "gt_z"))
        q = (num(r, "vo_x"), num(r, "vo_y"), num(r, "vo_z"))
        if prev is not None:
            dg = math.dist(p, prev[0])
            dv = math.dist(q, prev[1])
            gt += dg
            vo += dv
            # "Still" by GROUND TRUTH, not by the VO — asking the VO whether it
            # moved is asking the suspect to testify.
            if dg < 1e-3:
                still_gt += dg
                still_vo += dv
                n_still += 1
        prev = (p, q)

    ratio = vo / gt if gt else float("nan")
    still_ratio = still_vo / still_gt if still_gt > 1e-9 else float("inf")

    print(f"\n=== {os.path.basename(path)}   {num(rows[-1], 't_rel'):.0f} s, "
          f"{len(rows)} amostras")
    print(f"  erro   final {err[-1]:6.2f} m   mediano {sorted(err)[len(err)//2]:6.2f} m"
          f"   pico {max(err):6.2f} m")
    print(f"  yaw    pico  {max(yaw):6.1f} deg")
    print(f"  percurso  GT {gt:6.1f} m   VO {vo:6.1f} m   escala {ratio:5.2f}x")
    print(f"  parado ({100 * n_still / len(rows):.0f}% das amostras): "
          f"VO reportou {still_vo:.2f} m", end="")
    print(f"   ({still_ratio:.2f}x)" if still_gt > 1e-9 else "   (GT imovel)")

    print("\n  linha do tempo — um salto de yaw com path_len parado e um giro perdido")
    step = max(1, len(rows) // 20)
    for r in rows[::step]:
        print(f"   {num(r, 't_rel'):7.1f}s   err {num(r, 'err_norm'):6.2f}"
              f"   yaw {num(r, 'err_yaw_deg'):8.2f}"
              f"   path {num(r, 'path_len'):6.1f}")


def main(argv):
    paths = argv[1:]
    if not paths:
        found = sorted(glob.glob("logs/odom_error_*.csv"),
                       key=os.path.getmtime)
        if not found:
            print("nenhum logs/odom_error_*.csv — voa primeiro")
            return 1
        paths = [found[-1]]
    for p in paths:
        report(p)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
