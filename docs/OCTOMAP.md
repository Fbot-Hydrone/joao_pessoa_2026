# The 3-D occupancy map

A probabilistic octree of the arena, built from the ZED's point cloud. It
answers the one question the rest of the stack could not: **what is at this
point in space** — occupied, free, or never looked at.

That third answer is the reason it exists. `feature_map_node` accumulates
points, so it has two answers: there is a point here, or there is not. "There
is not" covers both *I looked and it is empty* and *I have never looked*. To a
planner one is a corridor and the other is a bet.

```
/zed/.../cloud_registered ──> cloud_filter_node ──> /hydrone/map/cloud_filtered
                                                            │
                                                      octomap_server
                                                            │
                                        /octomap/octomap_binary   (the tree)
                                        /octomap/projected_map    (2-D)
```

## Running it

Nothing to pass. It is on by default:

```
BS_SIM_DIR=... ./scripts/docker_up.sh --phase1
```

The node logs what it is doing on startup:

```
filtering /zed/zed_node/point_cloud/cloud_registered -> /hydrone/map/cloud_filtered
(stride 4, 0.4-12.0 m, 2 Hz, hole borders rejected)
```

Launch arguments, all with working defaults: `octomap` (true), `octomap_res`
(0.15), `octomap_hz` (2.0), `octomap_free_space` (false).

## Looking at it

**The rviz plugin must be installed on the HOST**, not only in the image —
rviz2 runs on your machine:

```
sudo apt install ros-humble-octomap-rviz-plugins   # then restart rviz2
```

Then `Add -> By display type -> octomap_rviz_plugins/OccupancyGrid`, topic
`/octomap/octomap_binary`.

**Do not display the MarkerArrays.** They are why the cubes used to blink.
MEASURED on `/octomap/occupied_cells_vis_array`: every publication carries 17
markers, **16 of them DELETE and 1 ADD** — octomap_server emits one marker per
tree depth and deletes the empty ones on every insert, so rviz tears the whole
display down and rebuilds it each cycle. They are also 111 KB per update
against 3 KB for the whole tree, and they grow for the length of the flight.
The OccupancyGrid display decodes the latched tree locally and has nothing to
redraw between updates.

Colours in that display are **height**, not occupancy (`Voxel Coloring:
Z-Axis`). Switch it to `Cell Probability` to see how sure the map is — a wall
seen from several angles saturates, a voxel seen once stays pale. Everything
drawn is an OCCUPIED voxel; free and unknown are invisible, not absent.

`Max. Octree Depth` (16) is the lightest knob there is: drop it to 14 and you
see the octree's coarse cubes for a fraction of the draw cost.

## Reading it from code

ROS's converters for `octomap_msgs/Octomap` are **C++ only**, so a Python node
subscribing to `/octomap/octomap_binary` gets bytes it cannot question.
`hydrone_map.octree` closes that gap (it needs `octomap-python`, which the
Dockerfile installs):

```python
from hydrone_map.octree import tree_from_msg, query, is_free, path_is_clear

tree = tree_from_msg(msg)              # from /octomap/octomap_binary
query(tree, (1.2, -0.4, 0.9))          # 'occupied' | 'free' | 'unknown'
is_free(tree, p)                       # unknown is NOT free
path_is_clear(tree, a, b)              # every voxel along the segment
```

`is_free` refuses unknown deliberately. Flying through what was never mapped is
how a drone finds a wall.

Decode when you need to plan, not per frame: `tree_from_msg` writes a temp file
because `octomap-python` only reads from disk. At a few KB that is cheap, but
it is not free.

For 2-D planning, `/octomap/projected_map` is a plain `nav_msgs/OccupancyGrid`
— an `int8` array, no decoding needed at all.

## The numbers behind the settings

All measured on a synthetic 6x6 m floor plus one wall, unless noted.

| setting | value | why |
|---|---|---|
| `octomap_res` | 0.15 | 0.10 -> 111 KB per marker update, 0.15 -> 52 KB, 0.20 -> 31 KB. 0.15 halves 0.10's traffic and still leaves ~5 cells across a Phase 4 window (0.8 m); 0.20 leaves 4, too coarse for a 330 mm drone |
| `process_hz` | 2.0 | the camera runs at 10 Hz and the drone moves centimetres between frames. It is the CPU dial (ray casting is per point) and the bandwidth dial |
| `latch` | true | with false, octomap_server publishes ONLY ON CHANGE — a viewer that connects later gets nothing until the map next changes, and draws an empty failed display |
| `publish_free_space` | false | 172 KB per update, and it grows all flight. Turn on to inspect (`octomap_free_space:=true`), not to fly |
| `occupancy_min_z` | 0.25 | without it the floor is projected as obstacle and `/projected_map` comes back 1681 occupied / 4 free — useless. Clipped: 1260 free, 41 occupied, 547 unknown. 0.25 also clears a landing pad, which is somewhere to land, not something to avoid |
| `sensor_model.hit/miss` | 0.7 / 0.4 | slow to believe, slow to forget. Phase 4 flies through gaps this map defines |
| `reject_hole_borders` | true (here) | off in feature_map_node. The sky/wall border is where a ray does the most damage |

## Why the cloud is filtered first

Never point octomap_server at the raw camera. A pixel on a silhouette returns a
foreground/background blend: MEASURED on the arena, a wall at **4.86 m**
produced edge samples out to **18.07 m**. In a voxel grid that is one wrong
cell. In an occupancy grid the ray to that phantom carves everything it crosses
as FREE — including the real wall it belongs to — and the planner reads a
doorway where there is masonry.

The rejection lives in `hydrone_map.cloud_filter`, shared with
`feature_map_node` so both maps reject the same points, and pinned by
`test_cloud_filter.py`.

## Why the cloud is NOT transformed

`cloud_filter_node` passes the header through untouched. octomap_server finds
the ray origin by looking the cloud's `frame_id` up in TF; hand it a cloud
already folded into `odom` and every ray is traced from the world origin
instead of from the camera, filling the map with free space swept from a point
the drone has never been. `map_odom_node` is what makes that TF lookup reach
the world.

## What is NOT done

In the order that matters to navigation.

### 1. `odom` drift — the real problem, and the one to MEASURE first

The map accumulates in `odom`, which is the VO's frame, and the VO drifts. Over
a ten-minute attempt a wall seen at minute 1 and again at minute 8 is inserted
twice: you get a double wall, or one smeared by however far the estimate
walked. A planner reads that as a corridor where there is masonry, or the
reverse. It does not show up on a freshly started sim; it shows up on a long
attempt.

**MEASURED 2026-08-27** (odom_error_20260827_010810.csv, a 51.5 m flight):
final position error **5.28 m**, peak **11.98 m**, yaw error up to 180°. In an
8x8 m arena that is not an estimate of anything.

But the cause was not bad odometry. Splitting the samples by whether the
vehicle was actually moving:

    TOTAL       ground truth 51.5 m, VO reported 527.3 m   -> 10.2x inflation
    94% still   ground truth  0.0 m, VO reported 441.8 m   -> 1.61 mm/frame
     1% moving  ground truth 50.6 m, VO reported  58.2 m   -> 1.15x

Essentially all of the drift was noise integrated while parked. Fixed by a
zero-velocity update in `visual_odometry_node` (`min_step_m`); see
[`VO-DRIFT.md`](VO-DRIFT.md). **The numbers above are from BEFORE that fix and
need re-measuring** — the CSV now lands in `./logs/`, which is bind-mounted.

If the re-measured drift is centimetres over ten minutes, this item is closed
and the map is ready. If it is still metres, the choice is real: map in `map`
(does not drift, but jumps when the EKF corrects, which tears the map), clear
the map periodically, or SLAM.

### 2. `filter_speckles` is off

An isolated noise voxel stays in the map. Irrelevant to look at, a phantom
obstacle to a planner — it makes the drone dodge nothing. One line.

### 3. There is no obstacle inflation

The map says voxel (2.1, 0.3, 1.0) is occupied. It does not know the drone is
330 mm across and does not fit through gaps that are technically open. Every
serious planner inflates obstacles by the robot's radius before planning. This
does not exist in the stack and is required before the first autonomous flight
in the confined space — it is the difference between "a path exists" and "I fit
through it".

### 4. `path_is_clear` is not a planner

It answers "is this straight line clear?". What generates the lines — A*, RRT
over the octree — belongs in `hydrone_nav` and does not exist yet. That is the
piece that turns this map into navigation.

### 5. The map does not survive an attempt

Phase 1 allows 3 attempts in 30 minutes and each starts from an empty map.
`octomap_saver_node` writes a `.bt` and the server can load one. Worth doing for
the arena's structure; not for the pads, which are moved between attempts.

## Related

- [`PACKAGES.md`](PACKAGES.md) — where the map lives and why
- [`ZED-FEATURE-MAP.md`](ZED-FEATURE-MAP.md) — the voxel map it sits beside, and
  which is a pure observer nothing reads
