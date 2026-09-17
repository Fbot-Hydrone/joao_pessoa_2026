---
tags: [hydrone, phase4, mission, plan]
---
# Phase 4 Maze Explorer

Back to [[Hydrone]]. Design of the autonomous Phase 4 mission that replaces the hardcoded [[Phase 4 Maze Mission]]: find the structure's entrance on its own, go in, map the whole inside (every dead end), find the exit, get out, land on the arena-middle pad. QR-code reading comes later and plugs into the exploration. Built on [[LIO Odometry]] and [[Persistent Map]].

## The task (rules + Ivo, 2026-09-17)
- The confined structure is 2 × 6 × 1.5 m and runs along one arena wall, next to the takeoff pad (which is elevated ~0.5 m). Together they span the whole arena wall.
- It has **3 openings**:
  - the **entrance window**: in the end wall right beside the takeoff pad (red arrow in the rules)
  - a **door** close to it (floor to roof), which must **not** be used
  - the **exit window**, far away
- Windows are 0.8 × 0.8 m with a sill below them; the door is 0.8 × 1.5 m.
- The exit is always **> 4 m** from the entrance; the two near openings are well under 4 m apart.
- Inside: rooms and corridors ~0.9–1 m wide, dead ends. Five 10 cm QR codes on inner walls (not floor or roof), later.
- After exiting, land on the landing pad in the **middle of the arena**. The Kopis has no camera, so the pad position is inferred from the mapped arena.
- Drone ≤ 330 mm with prop guards. Walls may be touched without penalty, but landing anywhere but the pad ends the attempt. 10 min per attempt.

## Principles
- **Generic first.** The main pipeline knows only general geometry: a confined space has a roof, an opening is a passable gap in its wall, a window has a sill and a door doesn't, and exit > 4 m from entry. The rulebook dimensions live in a separate **fallback** that is used only when the generic pipeline can't decide.
- **Unknown ≠ free.** Plan only through observed free space.
- **Slow and bounded.** Velocity commands with a speed cap inside (~0.25 m/s) instead of jumping position targets, which overshot ~0.6 m on LIO nav.
- **Pure logic is ROS-free** and tested headless against a synthetic maze before BiguaSim.

## Frames and heights (sim, measured)
- `odom`: base_link at takeoff, x forward, y left. Everything plans here.
- Pad top ≈ odom −0.08 (base_link rests ~0.1 m above its feet). Floor ≈ −0.62. Roof ≈ +0.9 (1.5 m above the floor).
- Windows span floor+0.7 … roof, so their centre is at world ≈ 1.1 m ≈ odom 0.5.
- Mid-360 mounted 0.1 m above base_link; FOV −7…52° elevation. It sees little below −7°, so sills are best observed while low (on the pad or climbing).
- In this sim the structure lies along −y, roughly x −0.8…1.35, y −1…−7.1. The entrance window is in the y ≈ −1 end wall, the door in the x = 1.35 face at y ≈ −2.5, the exit window in the x = 1.35 face near the far end. **None of this may be hardcoded.** It is only for checking results.

## Architecture
`hydrone_mission/hydrone_mission/maze/` (pure Python + numpy, no rclpy):

| Module | Job |
|---|---|
| `grid.py` | 2D log-odds occupancy grid at the flight band, ray-cast free space from registered scans + sensor origin; per-cell "roof" and "low band" hit counts from the same 3D points |
| `openings.py` | covered (roofed) region; openings = passable boundaries between covered-free and uncovered-free space; width, centre, inward normal; window vs door by sill evidence |
| `frontiers.py` | frontier cells/clusters inside the structure footprint; reachable-frontier selection by path cost; completion test |
| `planner.py` | inflated grid, A* over known-free cells, path smoothing/shortcutting |
| `arena.py` | arena rectangle from low-band wall hits → landing point (middle) |
| `geometry_fallback.py` | rulebook model: fit the 2 × 6 box, predict the entrance/exit windows and the arena middle when the generic pipeline can't decide |
| `mission.py` | the state machine as a pure class: `step(pose, scan) -> command`. Phases: TAKEOFF/SURVEY → FIND_ENTRANCE → ALIGN → ENTER → EXPLORE → GO_EXIT → PASS_EXIT → GO_LAND → LAND. Emits velocity/hover/land commands |
| `sim/` (tests) | synthetic arena + maze from the rules layout, numpy lidar ray-caster with Mid-360-like FOV, kinematic drone with lag + noise; runs the mission headless |

`hydrone_mission/phase4_maze_node.py` becomes a thin ROS wrapper:
- **in:** `/cloud_registered` + `/hydrone/lio/odom_raw`, MAVROS state and local pose
- **out:** velocity setpoints, arm/mode/takeoff/land, debug topics (grid, openings, frontiers, path)

## Validation
1. Unit tests for each module on synthetic data.
2. The headless mission sim on the rules layout plus variants (mirrored, shifted, different interior walls, noise, lag):
   - enters the window, **never the door**
   - interior coverage ≥ 95%
   - exits through the far window
   - landing point within 0.5 m of the arena middle
   - minimum wall clearance never below the drone radius
3. Offline replay on recorded BiguaSim bags (`maps/bags/`).
4. BiguaSim end-to-end flights with ground-truth clearance and coverage checks.

## Status
Design written 2026-09-17. Implementation in progress (agents), see the log in `notes/log-claude-autonomous-changes/2026-09-17_10-47.md`.
