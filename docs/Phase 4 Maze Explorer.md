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
**2026-09-17: core + headless sim implemented and passing.** The ROS wrapper (`phase4_maze_node.py`) and BiguaSim flights are not done yet. Log: `notes/log-claude-autonomous-changes/2026-09-17_10-47.md`.

### What's there
- `maze/grid.py`: `OccupancyGrid.update(points, sensor_origin)`, a ±20 m grid of 0.1 m cells. Free space is carved only where the ray is inside flight_z ± 0.2 and only up to 6 m. Returns beyond 12 m still carve free space but mark nothing, which is what makes the outside visible through a window from inside. Extra layers:
  - `roof`: hits at flight_z + 0.3 … + 1.0
  - `low`: hits at floor + 0.1 … flight_z − 0.45, counted only on **vertical faces** (a cell whose hits in one scan span ≥ 6 cm). This way the takeoff pad top, which sits right against the entrance wall, doesn't count as a sill.
  - `low_pass`: rays crossing the low band
  - `observed`
  - the floor, estimated as the lowest strong peak of a z histogram
- `maze/openings.py`: `detect_openings(grid, viewer=xy)` returns `(openings, covered, crop)`.
  - Covered region = roof cells, closed and hole-filled.
  - Candidates = covered free cells next to uncovered, non-wall cells. Each one is checked for:
    - width 0.45–1.5 m between its flank walls
    - wall-line snapping (centre moved onto the wall using flanks + sill)
    - no wall inside the gap
    - clear 0.45 m in and out
    - **leads outside**: uncovered open space ≥ 1.2 m beyond. Without this check, a passage into a room whose roof isn't mapped yet looked like an opening.
  - `confirmed=False` means the far side hasn't been seen yet.
  - Classification:
    - `window`: ≥ 3 vertical low hits at the wall line
    - `door`: ≥ 12 low-band passes 0.1–0.2 m past the wall on the side away from the viewer. A ray that clears a sill can't drop into the band that fast with a −7° FOV floor.
  - `window_extent()` gives the sill top and roof from raw points (unit test: 0.08 / 0.88, centre 0.48).
- `maze/frontiers.py`: frontier = free cell 4-adjacent to a never-observed cell, inside covered grown by 0.5 m. Wall cells flip between hit and miss, so "unknown" there would never clear.
  - Clusters < 5 cells are dropped.
  - The view cell is the cheapest reachable cell ≤ 0.6 m from the cluster with line of sight to it and looking toward its unseen side (cos ≥ 0.5). Views that didn't clear a cluster are blacklisted; the cluster is dropped after 3.
- `maze/planner.py`: Dijkstra cost field + descent (no corner cutting), A*, line-of-sight shortcutting, `GridPlanner` in odom metres.
- `maze/arena.py`: min-area rectangle (rotating calipers) for orientation. Each side sits on the outermost line with ≥ 8 hits and is pushed out over the structure footprint. The arena wall behind the structure is never seen, and percentile trimming alone put the middle 0.4–0.9 m off.
- `maze/geometry_fallback.py`: fits the 2 × 6 box to roof cells (long axis = the one the takeoff point is beyond; open face = the side with more free space) and predicts entrance, door, exit and landing. `match()` snaps a prediction to a detected gap on the same wall.
- `maze/mission.py`: `Mission(params).step(t, (xyz, yaw), (points, origin) | None) -> Command(kind, velocity, z, yaw_rate, debug)`. Phases as in the table above, plus `CLIMB`. Details:
  - Inside planning uses the footprint minus everything connected to the takeoff point's open space (that's outside), minus the outer half of every confirmed opening.
  - An exit candidate is > 4 m from the entry. Evidence ranks confirmed > window > unknown > door-looking. The exit is locked once GO_EXIT starts; an unconfirmed one is verified in front of it and dropped if it leads nowhere, and so is a crossing that stalls.
  - A predicted (fallback) opening switches to the real gap on its wall as soon as one is detected.
  - Every fallback use is logged as `FALLBACK: …` and listed in `fallback_used`.
- `test/maze_sim/`: box world from the rules layout (nominal + variant interior, mirror, 90° turns, any odom yaw). It has:
  - a slab-method ray caster with the Mid-360 FOV (360°, −7…52°) and σ 2 cm range noise
  - a kinematic drone (τ lag, accel noise, pose noise + drift, with scans registered using the same pose error)
  - body-vs-box clearance
- `tools/phase4/replay_maze_bag.py`: offline replay of a bag through grid + openings, with PNGs.

### Test results (headless, 12k rays/scan at 10 Hz, dt 0.1 s)
`test/test_maze_core.py`: 30 unit tests. `test/test_maze_mission.py`: 5 scenarios. The whole `test/` dir: 97 passed in 2 min 40 s.

| Scenario | Mission time | Coverage | Min clearance | Landing target error | Fallback |
|---|---|---|---|---|---|
| (a) nominal | 77 s | 1.000 | 0.147 m | 0.00 m | none |
| (b) mirrored + turned, odom 0.7 rad | 75 s | 0.999 | 0.131 m | 0.02 m | none |
| (c) variant interior, odom −1.2 rad | 74 s | 1.000 | 0.133 m | 0.02 m | none |
| (d) noise 4 cm, drift 4 mm/√s, τ 0.5 s | 76 s | 0.999 | 0.101 m | 0.04 m | none |
| (e) sill returns withheld | 92 s | 0.999 | 0.096 m | 0.01 m | entrance |

- All five enter through the entrance window and leave through the exit window. Every crossing of the structure boundary is checked, and the door is never used.
- Coverage = the fraction of ground-truth interior cells (≥ 0.15 m from any wall, at flight height) that the grid marks free.
- Clearance = the gap between the body (0.165 m radius disc, −0.07…+0.10 m) and the nearest box. Touchdown on the pad itself is excluded.
- CPU per scan (desktop core):

  | Step | Time |
  |---|---|
  | Grid update, 20k-ray sweep inside the structure | 10 ms median |
  | Same, outside (3.4k returns, far carving) | 13.5 ms |
  | Openings detection | 6–7 ms |
  | Whole `Mission.step` (includes Dijkstra and frontier ranking once a second) | 10 ms mean, 24 ms p95 |

Seed sweep: the five scenarios again with sim seeds 2, 3 and 4 passed **15/15**.

| Metric | Range over the 15 runs |
|---|---|
| Coverage | 0.988–1.000 |
| Min clearance | 0.117–0.151 m |
| Landing target error | ≤ 0.03 m |
| Mission time | 73–95 s |

Offline check of `replay_maze_bag.py` on a synthetic bag written from the sim: window 0.80 m wide (gap z 0.12…0.87) and door both found, 13.6 ms per scan.

### Known limits
- The fallback box fitted from the pad survey only (about 3 m of roof seen) comes out ~17° off in yaw: the entrance prediction is within 0.3 m, but the far-end exit and landing predictions move by 1–2 m. The mission uses those only after exploring, when the roof is mapped, and snaps predictions to detected gaps.
- **Not flown and not seen on real data yet.** The BiguaSim survey bag hadn't been recorded when this was written; `replay_maze_bag.py` is ready for it. The sim walls are ideal boxes: BiguaSim will have messier returns, a different roof thickness and possibly no-return patches.
- The low band ends 0.45 m under flight_z, so the sill must be taller than that below the window centre (rules: 0.4 m). Door evidence assumes the Mid-360's −7° lower FOV edge.
- flight_z (0.5) is a parameter. The measured window centre is used for passing only if it's within ±0.25 m of it; the grid band itself doesn't move.
- Heavy drift smears walls. With 5 mm/√s drift, interior cells near walls turn occupied and coverage fell to 0.93, and seed-to-seed spread is visible at 4 mm/√s (0.956–0.999).
- A crossing is a straight crawl with a front-blocked check. There is no 3D check of the gap's vertical extent beyond the window-height estimate.
- `CLIMB` (over the structure when there's no way round outside) is written but no scenario exercises it.
- Abort paths (`LAND` after a timeout or failed crossing) land where the drone is, even inside the structure.
- Frontier give-up is time-based (3 s at the goal, 20 s stuck), so some runs spend 20–40 s re-viewing corners that can't be seen from the band.
