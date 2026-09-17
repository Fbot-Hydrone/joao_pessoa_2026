---
tags: [hydrone, phase4, mapping]
---
# Persistent Map

Back to [[Hydrone]]. The map Phase 4 builds from [[LIO Odometry]] poses, and how it's saved and reloaded. Node: `hydrone_lio/lio_map_node.py`, started by `phase4_sim.launch.py`.

## Layers
All in `odom` (base_link's pose at takeoff):

| Layer | What | Topic | File |
|---|---|---|---|
| Voxels | 5 cm grid, hit count per cell. What a planner reads | `/hydrone/map/voxels` (xyz + intensity = hits) | `voxels.pcd` (x y z hits) |
| Dense cloud | plain points, **deduplicated**: a point is kept only if its 2 cm cell is still empty | `/hydrone/map/cloud` | `cloud_dedup.pcd` |
| Keyframes | raw registered scan + pose every 0.5 m or 15° | — | `keyframes/NNNN.pcd`, `poses.csv` |

- **Why dedup instead of plain accumulation.** A hovering Mid-360 re-measures the same wall 10× a second, so the accumulated cloud becomes a swarm of near-duplicates that grows forever (Ivo's concern). The first point in each 2 cm cell wins, so revisiting adds nothing. 2 cm is the sensor's range noise; finer cells would keep noise copies.
- **Why keyframes too.** The voxel and dedup layers are lossy. Keyframes keep the untouched data (FAST-LIO `dense_publish_en: true`) with the pose they were registered at, so a better odometry later can rebuild the map offline.

## Save and load
- `ros2 service call /hydrone/map/save std_srvs/srv/Trigger`, and automatically on shutdown.
- Files go to `<map_dir>/<map_name>/`. `map_dir` defaults to `/ws/maps`, which docker-compose mounts from `./maps` (gitignored).
- `scripts/docker_up.sh --phase4 load_map:=true map_name:=phase4` starts from the saved voxels + dense cloud.
- **No relocalization.** A loaded map assumes the drone starts where that map's odom began (same spawn). Place recognition against a saved map belongs to [[Degraded Sensing]].
- PCDs are binary float32, readable by PCL / CloudCompare / Open3D.

## Frames
FAST-LIO registers into `camera_init` (the lidar at start). `odom = mount ∘ camera_init` is a fixed transform, applied once per point.

## Limits
- Python dict/set storage. Fine for the arena (a few hundred thousand voxels), not for a campus.
- No removal of dynamic objects or of stale cells: a voxel hit once stays forever.
- The whole map is re-published every `publish_period_s` (2 s), and only when someone subscribes.
