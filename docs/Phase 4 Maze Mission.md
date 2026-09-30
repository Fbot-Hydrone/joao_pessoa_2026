---
tags: [hydrone, phase4, mission]
---
# Phase 4 Maze Mission

Back to [[Hydrone]]. Stretch goal on top of [[LIO Odometry]] and [[Persistent Map]]: enter the small structure right of spawn and leave through the other end without colliding. Node: `hydrone_mission/phase4_maze_node.py`, run with `scripts/docker_up.sh --dev --phase4 mission:=maze map_name:=<fresh>`.

## Status (2026-09-17)
**Built, not yet flown through.** The node reaches the entrance, but position-control accuracy (±0.5–1 m overshoot on long legs) is larger than the 0.6 m entrance gap. The LIO is not the blocker: it stayed within 5–15 cm of ground truth in every attempt, right up to each impact.

## The structure (measured from the saved map)
Odom frame: takeoff point, x forward, y left.

| Thing | Where |
|---|---|
| Footprint | x −0.8 … 1.35, y −1 … −7.1 (against the enclosure's back wall) |
| Floor / roof | z −0.62 / +0.92 (≈1.55 m inside) |
| Front wall | x = 1.35, **one gap at y −2.25 … −2.85 (~0.6 m)**. The "gap" at y ≈ −6.5 seen on the first merged map was noise from earlier flights |
| Interior walls | y ≈ −2.1, −3.0, −4.05, −6, −7 and x ≈ −0.6, 0.35. Corridors ~0.8–1 m |
| Kopis clearance | base_link rests 0.6 m above the floor (the collision body hangs that far down) and the lidar is +0.5 m, so fly at **z 0.1–0.3** inside |

Tools: `tools/phase4/plot_map.py` (top view), `slice_map.py` (height slices), `plan_offline.py` (runs the node's planner on a saved map and draws the grid and path).

## How the node works
1. **TAKEOFF**: GUIDED, arm, climb to 1 m.
2. **APPROACH_HIGH → LOW → SIDE**: forward along y = 0 (clear of the structure), down to fly_z, then sideways along the front face to `entry_xy` (1.9, −2.55). Never diagonal over the roof: the first attempt clipped the roof corner that way.
3. **MAZE**: at 1 Hz, A* (8-connected, no corner cutting) on a 10 cm grid.
   - The grid is cut from the live voxel map between fly_z − 0.55 and fly_z + 0.6 (floor and roof excluded), voxels with <2 hits dropped, obstacles inflated by 0.2 m.
   - Unknown counts as free, and the grid is fenced to the footprint so the path can't go round the outside.
   - If the exit is walled off in the known map, it flies to the reachable cell nearest the exit to see more.
   - It follows a 0.5 m carrot.
4. **EXIT_LOW → EXIT → LAND**: out to `exit_xy` (0.0, −7.7) past the far end, climb, land.

## Attempts
| # | Change | Result |
|---|---|---|
| 1 | first try, no LIO velocity to the EKF | crashed on the approach: diagonal over the roof corner plus ~1 m overshoot |
| 2 | LIO velocity → EKF, approach around the structure, WP_SPD 0.7 | reached the gap cleanly; **no path**: inflated gap closed and the wrong exit guess |
| 3 | radius 0.2, far-end exit, explore-toward-exit fallback | sideways approach overshot 1.2 m into the wall edge at the gap, slid onto the roof, crashed |

## Position-loop step tests (LIO nav, 1.5 m forward/back)
| Gains | Overshoot | Lateral wander | Peak speed |
|---|---|---|---|
| firmware defaults (PSC_NE_POS_P 1, VEL_P 2, VEL_I 1) | +0.61 / −0.57 m | 0.83 m | 1.97 m/s |
| POS_P 0.8, VEL_I 0.4 (the VEL_P 1.2 set failed: MAVROS param node not up yet) | +0.63 / −0.57 m | 0.59 m | 2.14 m/s |

- `WP_SPD 0.7` is loaded (read back), yet the vehicle reaches ~2 m/s on a MAVROS position target. Check whether this 4.8-dev GUIDED path really takes its limit from `WP_SPD` before tuning further.
- Not continued: tuning the position loop against BiguaSim's actuation lag is its own job.

## What's needed next (in order)
1. **Position-loop accuracy.** Tune the PSC gains against BiguaSim's actuation lag (see [[LIO Odometry]]). A 0.6 m gap needs ±0.15 m.
2. **Slow, short approach into the gap:** stop 1 m out, align y, then creep in.
3. **Verify the real layout from inside** (hover in the entrance room) before trusting the exit side.
4. Only then does the planner's exploration get a fair test.

Open questions from the plan that still stand: degeneracy of LIO inside narrow corridors (not seen yet; it tracked fine at the gap), and whether RGB helps inside ([[RGB Camera Enhancement]]).
