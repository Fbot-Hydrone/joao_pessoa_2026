---
tags: [hydrone, phase4, mission]
---
# Phase 4 Known Maze

Back to [[Hydrone]]. This is the Phase 4 mission flown **on the rulebook map, by the maze walls**. LIO stays out of the loop inside. The rulebook draws the whole structure, and flying a place whose walls you know is simpler and less error-prone than exploring it. Built on [[Confined Navigation]]. The generic [[Phase 4 Maze Explorer]] stays as the fallback and as the no-map mode.

## Why not LIO inside
LIO degenerates in a maze of 1 m rooms. The Mid-360's 0.25 m blind zone eats the near walls, and a corridor gives no constraint along itself. The FCU's velocity loop runs on whatever pose feeds its EKF, so a sliding LIO drags the drone even when the mission logic is fine. So inside, the EKF is fed a pose computed from the walls:

- a handful of **virtual range beams** (the Mid-360 levelled by the FCU attitude, the nearest return within ±3° of each of 24 angles, in a thin band at the drone's height)
- matched against the **known map** by a small particle filter

LIO keeps running and only predicts between scans. On the way out, LIO is re-anchored to the map and flies to the arena-middle pad.

## Pipeline
```
Mid-360 ─┬─> FAST-LIO ─> lio_odom_adapter ─> /hydrone/lio/odom ──────┐
         │                                                           v
         └─> beam_localizer_node (beams vs rulebook.yaml) ──> pose_mux_node ─> /hydrone/nav/odom
              ~/beams ~/sectors ~/status        ^ /hydrone/nav/source   │
                  │                             │                      ├─> vision_odom_bridge ─> EKF
                  └──────────> phase4_known_node┘ ─> MAVROS velocity   └─> phase4_known_node
```
- `pose_mux_node` always outputs **LIO through an anchor**, in the map frame from the first message. The map pose never reaches the EKF directly: it pulls the anchor, fast (τ 0.3 s) from the entrance to the exit and slowly outside, matched against LIO at the scan's own timestamp. That keeps the flight smooth and stops LIO from wandering: a LIO sliding at s m/s stays within about s × 0.3 s of the walls.
  - Feeding the raw map pose made the drone swing 0.8 m around a passage in BiguaSim: 10 Hz, ±5 cm jitter, ~100 ms late, plus steps whenever the source switched.
- `phase4_known_node` asks for `map` from the entrance to the exit crossing, and `odom` otherwise. The mux refuses a stale or unhealthy map pose on its own.

## The map: `hydrone_mission/config/phase4/known_maps/rulebook.yaml`
Frame "maze": the arena corner behind the takeoff pad. x runs along the structure, y out into the arena, z up from the floor. Arena [0,8]², takeoff pad 2 × 1.5 m in the corner (yaw 90°: facing the arena), structure [2,8] × [0,2] × 1.5 m, landing pad at (4, 4).

- **Walls** are hard and come from Fig. 6.
- **Passages** between rooms are only drawn in the Fig. 5 render. Those measured in BiguaSim (built to the rules; merged `/cloud_registered` from `maps/bags/maze2..6`) are:
  - entry → L: a **low hole** 0.05–0.77 m high
  - L → b1: a low hole
  - b1 → b2: a window
  - the door
  - the entrance window

  The ones past s ≈ 5 m (b2→t2, t2→r5, r5→r6) are read off the render and marked `soft: true`.
- **Rooms:** entry, l_low, l_top (the two arms of the L), b1, b2, t2, r5, r6. It's one chain: entry → L → b1 → b2 → t2 → r5 → r6 → exit.

Heights differ per passage: holes are passed at 0.41 m, windows at 1.1 m, doors at the cruise height of 0.75 m. The planner and the beams always look at the band the drone is in.

## The mission (`phase4/known_mission.py`, `route.py`)
1. **TAKEOFF** to 1.1 m. It waits up to 6 s on the pad for a healthy map pose, then goes anyway on odometry (logged).
2. **TO_ENTRY / ENTRY_HOLD**: fly outside to 0.9 m in front of the entrance and ask for the map pose. If it never becomes flyable (8 s), enter on odometry (logged).
3. **The tour**: a depth-first walk over the room graph, nearest first, the exit room last. At each look point it turns once around (0.8 rad/s) so a camera sees every wall. Long rooms get two look points.
4. **Passages**:
   - **ALIGN** 0.45 m in front, at the passage's own height.
   - **VERIFY**: the beam straight through, and the beams 15° to either side, must all see ≥ 0.3 m past the wall line. One beam alone can pass along the edge of a gap the body won't fit through.
   - **CRAWL** through at 0.15 m/s.
   - **SEARCH** if VERIFY fails, or if a crawl times out twice: sweep the wall at pass height from −0.6 to +0.6 m in 0.2 m steps, then centre on the middle of the longest open stretch. If nothing is open, the passage is walled up and the route is replanned without it. (The first version took the first offset where one beam saw through, found a passage shifted by 0.4 m at −0.2 m, and crawled into its jamb forever.)
5. **QR**: every code read counts once. With `qr_target` (5) read, the rest of the tour is dropped for the shortest way out.
6. **Exit** window → **GO_LAND** on re-anchored LIO → land on the pad.
7. **No route left** on the known map (the prior is wrong): the generic explorer takes over. It has been fed the scans all along, registered through the flying pose.

All velocities go through the beam guard. Planning inside uses the map rasterised at the drone's band, inflated by 0.2 m, with the forbidden door walled up.

## Running it
- **Headless, no ROS:** `python3 tools/phase4/sim_known.py [--degen 0.15] [--closed b2_t2] [--yaw-error 12] [--viz] [--realtime]`
- **ROS on the simple backend:** `ros2 launch hydrone_bringup phase4_simple.launch.py [closed:="['b2_t2']"] [degen_rate:=0.15]`
- **BiguaSim:** `ros2 launch hydrone_bringup phase4_sim.launch.py mission:=known [viz_host:=<ip>]`
- **Viewer:** [[Maze Viewer]]
- **Tests:** `src/hydrone_mission/test/test_phase4_known.py` (slow)

## Results (kinematic sim, rulebook world, 5 QR codes)
| Run | Result |
|---|---|
| nominal | done in 205 s, all 5 QR, min clearance 0.175 m, landed 0.08 m from the pad centre, no fallback |
| LIO sliding 0.15 m/s inside | done in 205 s, all 5 QR, min clearance 0.182 m, landed 0.09 m off; map pose error while flying on it: mean 0.03 m, max 0.115 m |
| start pose 12° / 15 cm off the prior | done, all 5 QR, landed within 0.1 m |

### BiguaSim (machine 1, `biguasim.yaml`, 2026-09-30)
The **whole mission**, run 8: takeoff, entrance window, all 8 rooms with a turn at each look point, every passage at its own height (three low holes at 0.45 m, windows at 1.02–1.1 m, a door at 0.75 m), exit window, back on LIO, landed and disarmed on the pad. It took **361 s**, and no QR codes were read (BiguaSim has neither a camera nor codes).
- Flying pose vs ground truth inside, with the world→map frames fitted: **mean 0.043 m, p95 0.105 m, max 0.15 m**. z within ±1 cm while indoors.
- Touchdown 0.17 m from the pad centre.
- LIO alone was 0.06 m mean in this run: it didn't degenerate in this sim. The value of flying by the walls shows in the degeneracy runs above.

Getting there took these fixes:
- the EKF feed (see Pipeline)
- a start gated on the FCU home (a takeoff sent while AHRS is still on DCM is accepted, then not flown)
- the mission wanting the map pose from the entrance on
- r5↔r6 is a low hole, not the door the render suggested: the drone couldn't crawl under its lintel at 0.75 m

Localizer: 34 ms/scan mean on this laptop (CPU clamped to powersave). The mission step is ~76 ms, most of it the explorer fallback's background grid: `explore_fallback:=false` drops it on a Nano/RPi5.

## Known limits
- In `biguasim.yaml`, the teleport survey only half worked (scans from different poses disagree by up to ~1 m). Every BiguaSim passage has now been flown through, which confirms its passages. The rulebook's soft passages at the competition are handled online (VERIFY / SEARCH, crawl retry, explorer fallback).
- The explorer-handover scenario (a passage walled up) exposed the explorer flying into a window sill at its 0.75 m band. It now uses 0.40 / 0.90 m for the handover.
- The soft passages are guesses from a render. VERIFY/SEARCH copes with a shifted passage, and a missing one ends in the explorer. A BiguaSim teleport survey gives the exact sim layout.
- The fake QR detector assumes a forward camera with 70° FOV reading up to 1.3 m. The real camera's mount and range will set `spin` and the look points.
- Outside, beyond ~3 m from the structure, the map pose weakens (few walls in view) and flying is back on re-anchored LIO. That's by design.
