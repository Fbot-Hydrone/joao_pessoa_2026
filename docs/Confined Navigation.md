---
tags: [hydrone, package, navigation, phase4]
---
# Confined Navigation

Back to [[Hydrone]]. `src/hydrone_confined`: a general package for flying a drone with a 360° lidar in a confined space. It is not tied to the competition: the Phase 4 mission ([[Phase 4 Known Maze]], [[Phase 4 Maze Explorer]]) is built on it, and so can anything else that flies indoors. The core is pure Python + numpy with no ROS. The nodes are thin wrappers. The simple backend (a kinematic sim) and a viewer ship with it.

## Two ways to know where you are
- **Online map, LIO pose**: the explorer's way. A live occupancy grid (per height band) is built from LIO-registered scans. It finds openings and frontiers, and plans on what it has seen. See [[Phase 4 Maze Explorer]].
- **Known map, beam pose**: the drone is told the walls. A few range beams cut out of the lidar scan are matched against them, so the pose comes from the walls themselves rather than from integrating motion. LIO still runs, but it only predicts between scans. This page is mostly about this part.

## Modules (`hydrone_confined/core`)
| Module | Job |
|---|---|
| `known_map.py` | Walls with thickness and openings with z ranges (window, door, low hole), rooms, pads; loaded from YAML. Everything is asked per **height band**: a window is a gap at 1.1 m and a wall at 0.4 m. Vectorised ray cast, rasterisation. |
| `beams.py` | The "2D robot with N rangefinders", out of a 3D scan. It levels the scan with the FCU roll/pitch, keeps a thin band around the sensor, and takes the k-th nearest return within ±3° of each beam angle. The Mid-360's pattern never fires along an exact angle, so that margin is what makes a beam, and it also bounds the angle error. It also gives the guard's sectors, the wall direction mod 90° (`wall_yaw`), and the ceiling/floor distance. |
| `guard.py` | Last filter before the FCU. It caps the velocity component into any sector closer than 0.7 m to what can still stop in the space left, and pushes out below 0.3 m. Motion along a wall is kept, so the drone slides through doorways. It needs no map and no pose. |
| `beam_localizer.py` | Particle filter, ~250 particles (x, y, yaw). Beam endpoints are scored on a likelihood field (distance to the nearest wall face in the band). Yaw is scored against the rectilinear walls, and particles inside walls are killed. Health only judges beams the map expects a wall for, so a return off something unmapped (event walls, people) says nothing. |
| `map_localization.py` | One scan in, a map pose out. Level → beams/sectors/wall yaw/ceiling → predict (odometry step if believable, else the command) → z → update → healthy or not. Shared by the ROS node and the sim. |
| `pose_mux.py` | The pose that flies: always odometry × an anchor (the map→odom correction). The map pose pulls the anchor: fast when `map` is asked for (τ 0.3 s, capped at 1 m/s and 45°/s, z too), slowly when `odom` is (τ 2 s, 0.25 m/s, planar, only with `track`). A stale (1 s) or unhealthy map pose pulls nothing, and pulling resumes after 0.5 s of healthy poses. Map poses are matched against odometry at their own scan stamp. The output never steps. **map-only**: if odometry is silent for 0.35 s, or lying (a step or velocity > 1.2 m/s), the smoothed map pose flies alone and the odometry velocity isn't passed on. Odometry is re-anchored after 1 s of sane steps. |
| `navigator.py` | Hover, pure pursuit, replanning `go_to`, with pluggable obstacles (`GridObstacles` = live-grid repulsion, `BeamObstacles` = the guard) and pluggable planners. |
| `grid.py`, `openings.py`, `frontiers.py`, `planner.py`, `morph.py`, `arena.py` | The online-mapping half (moved from `hydrone_mission/maze`, unchanged). See [[Phase 4 Maze Explorer]]. |
| `viz_udp.py` | JSON-over-UDP debug stream for the viewer. Fire and forget. |

### Things that turned out to matter
- **Ceiling height.** Low in a 1 m room the Mid-360's steepest rays (52°) end on the walls, never on the ceiling. A median of the "up" returns then read z 0.15–0.45 m too high. Now the top cluster (within 6 cm of the 98th percentile) is used, and it is rejected when it sits entirely on the FOV rim, because a real ceiling shows up over many elevations. It must also agree with odometry within 0.15 m. Otherwise z is carried by odometry.
- **Health.** "Fraction of beams agreeing" was 0.3 on the pad with a perfect pose: most beams hit things outside the map. Only beams the map predicts a wall for count now.

## Nodes (`hydrone_confined/nodes`)
| Node | In | Out |
|---|---|---|
| `beam_localizer_node` | lidar (`PointCloud2`, or `livox_ros_driver2/CustomMsg` read raw: `cloud_type:=livox`), `/mavros/imu/data`, LIO odometry | `~/map_pose` (Odometry, `map`), `~/status` (OK = flyable), `~/beams`, `~/sectors` (LaserScan), `~/particles` |
| `pose_mux_node` | `/hydrone/lio/odom`, the localizer, `/hydrone/nav/source` | `/hydrone/nav/odom` (feed `vision_odom_bridge`), `/hydrone/nav/mode`, TF map→odom |
| `kinematic_sim_node` | MAVROS velocity setpoints + services | the same topics as BiguaSim + SITL + MAVROS + FAST-LIO: cloud, IMU, state, odometry, `/hydrone/qr`, `/hydrone/sim/truth` |
| `viz_udp_node` | whatever of the above exists | the viewer's UDP stream |

The Livox `CustomMsg` is parsed straight out of the CDR bytes (`rosutil.livox_custom_xyz`): 20k Python point objects per scan would cost tens of ms before any work is done. It is tested against the real serializer (`test/test_rosutil.py`).

## Simple backend
- **Headless** (`hydrone_confined/sim`): kinematic drone (velocity lag, acceleration noise, attitude from acceleration), a Mid-360-like ray caster over boxes, a drifting LIO-like odometry with an optional **degeneracy** model (sliding along the maze at `degen_rate` while inside), and QR codes seen through a camera frustum.
  - The FCU executes a velocity in the frame of **its own estimate**: v_world = R(true yaw) R(−estimated yaw) v_cmd, so a wrong pose shows up as wrong flight.
  - Worlds come from a KnownMap YAML (`world.from_known_map`) and can be perturbed (passages moved or walled up, start pose off) so the world disagrees with the map the drone was given.
- **ROS**: `kinematic_sim_node`, used by `phase4_simple.launch.py` (see [[Phase 4 Known Maze]]).
- **Viewer**: `tools/maze_viewer`, see [[Maze Viewer]].

## Tests
`src/hydrone_confined/test`, run in the container:
```bash
cd /ws/src/hydrone_confined && python3 -m pytest -q -p no:cacheprovider test/
```
- `test_known_map_beams.py`: bands, ray cast, beams, levelling, ceiling, wall yaw, guard, localizer convergence/tracking, mux switching
- `test_core_grid.py`: the online-mapping modules
- `test_rosutil.py`: raw Livox parser vs the serializer
