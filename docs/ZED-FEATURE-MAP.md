# ZED feature map — how the point cloud and the coverage grid are built

What `feature_map_node` does with the ZED's RGB-D stream, why the map lands in
the `map` frame, and what to set in RViz to actually see it.

> Sibling documents: [`ZED-VISUAL-ODOMETRY.md`](ZED-VISUAL-ODOMETRY.md) covers the
> node that estimates *where the drone is*. This one covers the node that records
> *what the drone has seen*. They read the same camera topics and are otherwise
> unrelated — see [§6](#6-why-this-is-not-in-the-vo-node).

Source: `src/hydrone_nav/hydrone_nav/feature_map_node.py`.

---

## 1. What it produces

| Topic | Type | Rate | Frame | Contents |
|---|---|---|---|---|
| `/hydrone/map/features` | `sensor_msgs/PointCloud2` | `publish_hz` = 1 Hz | `map` | one point per occupied voxel centre |
| `/hydrone/map/coverage` | `nav_msgs/OccupancyGrid` | `publish_hz` = 1 Hz | `map` | how thoroughly each 0.5 m patch of floor has been observed |

Both are **whole-map snapshots**, republished in full on every tick — not
incremental updates. Both QoS profiles are RELIABLE + **VOLATILE**, depth 1.
That volatility matters in RViz; see [§7](#7-seeing-it-in-rviz).

Neither is an obstacle map. The cloud is a sparse landmark cloud (ORB corners
only, not dense depth) and the grid stores *observation counts*, not occupancy.
Nothing in the stack plans around either of them today.

---

## 2. Inputs

```
/zed/zed_node/rgb/image_rect_color    ─┐  bgr8
/zed/zed_node/depth/depth_registered  ─┤  32FC1, metres, NaN = invalid
/zed/zed_node/rgb/camera_info         ─┤  K = fx, fy, cx, cy
/mavros/local_position/pose           ─┘  where the drone was
        +
TF base_link -> zed_left_camera_optical_frame   (constant mount, read once)
```

All four are subscribed with a **BEST_EFFORT, KEEP_LAST(1)** sensor profile, and
each callback does nothing but stash the latest message. Only the RGB callback
does work — RGB is the clock, and it pairs itself with whatever depth and pose
happened to arrive most recently. There is no time synchroniser and no attempt
to interpolate the pose to the image stamp.

That is a deliberate simplification with a real cost: at speed, the pose used to
place a frame's features can be one camera period stale, which smears the cloud
along the direction of travel. It is tolerable because the mapper runs at 4 Hz
against a vehicle that cruises slowly, and because nothing flies on this output.

**In simulation** all of `/zed/zed_node/*` comes from `zed_mimic_node`, which
republishes BiguaSim's cameras under the real ZED wrapper's exact topic names and
frame ids. On the real drone `zed_wrapper` publishes them. The mapper cannot tell
the difference and needs no configuration change between the two.

### Decoding

Images are converted by `hydrone_vision/image_convert.py` — `np.frombuffer` plus
a reshape, no `cv_bridge`. That is not a micro-optimisation: cv_bridge's compiled
extension is linked against the numpy ROS was built with, this container installs
numpy 2.x over it, and the mismatch **segfaults on the first conversion** rather
than failing at import. `depth_image_to_numpy` also normalises the two depth
conventions — 16UC1 millimetres (with 0 = no return → NaN) and 32FC1 metres — so
the rest of the node only ever sees float32 metres with NaN holes.

---

## 3. The per-frame pipeline

Every RGB frame that survives the rate gate runs these five steps.

### Step 0 — throttle

`process_hz` (default **4.0**) caps how many frames are processed per second;
frames arriving early return immediately. The ZED stream is shared with
`visual_odometry_node`, which is in the flight-critical loop, and mapping does not
need the full camera rate to build a usable map.

### Step 1 — detect features

```python
self.orb = cv2.ORB_create(nfeatures=max_features)   # default 400
keypoints = self.orb.detect(gray, None)
```

**`detect` only — no `compute`, no descriptors, no matching.** This is the single
biggest difference from the VO node, and it follows from what a map needs. The VO
must answer "is this the *same* corner as last frame?", which requires descriptors
and a matcher. The map only needs "there is *a* durable visual landmark at this 3D
position", and ORB's FAST-based detector answers that on its own. Skipping the
descriptor stage removes the most expensive part of ORB.

The consequence is that the map has **no notion of feature identity**. It cannot
tell one corner seen ten times from ten different corners — which is exactly why
accumulation is done by voxel hashing rather than by tracking landmarks
([§4](#4-accumulation)).

`max_features` is 400 here versus 1000 in the VO node. The VO needs a large pool
so enough survive ratio-test matching; the map keeps every keypoint it finds.

### Step 2 — read depth at each keypoint

Keypoint pixel coordinates are rounded to integers and used to index the cached
depth image. Two masks are applied, both vectorised:

- **bounds** — `0 <= u < w`, `0 <= v < h`. RGB and depth are the same size in
  this stack, but the node does not assume it.
- **validity** — `np.isfinite(d)` and `min_depth <= d <= max_depth`
  (**0.4 – 20.0 m**). This drops NaN holes, the sim's far plane, and the ZED's
  noise floor at close range in one test.

If nothing survives either mask the frame is abandoned.

### Step 3 — back-project to the camera optical frame

Standard pinhole inverse projection, one vectorised expression over all surviving
keypoints:

```
x = (u - cx) / fx · d
y = (v - cy) / fy · d
z = d
```

The result is in the **optical convention**: Z forward along the lens axis, X
right, Y down. Note the asymmetry with the VO node — that one back-projects the
*previous* frame's matches to feed PnP, whereas this one back-projects the
*current* frame's keypoints because it is placing them, not tracking them.

### Step 4 — transform to the world

Two hops, composed:

```python
R_world_opt = R_world_base @ R_base_opt
p_cam       = p_base + R_world_base @ t_base_opt
world       = local @ R_world_opt.T + p_cam
```

- **`R_base_opt` / `t_base_opt`** — the camera mount, `base_link →
  zed_left_camera_optical_frame`. Looked up from TF **once** and cached forever
  (`_ensure_mount_tf`); it is a bolted-on camera, so re-reading it every frame
  would buy nothing. Until that lookup succeeds the node logs
  `waiting for TF base_link -> zed_left_camera_optical_frame` every 5 s and
  processes nothing — **this is the usual reason the map stays empty**.
- **`R_world_base` / `p_base`** — the drone's pose, straight from
  `/mavros/local_position/pose`, converted by a local `quat_to_matrix`.

Note `local @ R.T` rather than `R @ local`: the points are stored row-wise
(N×3), so the transpose applies the rotation to each row without a transpose of
the data itself.

#### Is the placement actually global?

Yes — verified numerically against the expression above, not just read. Camera
mounted 0.1 m forward and 0.05 m down, lens pointing along the nose, drone at
(10, 5, 2) yawed +90° so the nose points along world +y, and one feature 3 m
straight ahead of the lens:

```
camera in world : [10.    5.1   1.95]
point in world  : [10.    8.1   1.95]     expected [10. 8.1 1.95]  ok
shift when p_base -> 0: [10. 5. 2.]  ==  p_base           ok
```

The second line is the one that matters: the output translates **one-for-one**
with the drone's position, so the cloud is genuinely placed in the world and not
in the camera's frame.

Which means that if the published map ever *looks* camera-relative — the whole
cloud sitting on the drone, or fixed relative to it — the transform is not the
cause. **`p_base` is.** `self.pose` is whatever `/mavros/local_position/pose`
last delivered, and there are two ways that goes wrong, neither of them visible
from inside this node:

- **The EKF has no position estimate** and MAVROS publishes ~zeros. The map then
  degenerates to a back-projected depth snapshot centred on the origin, rotated
  by whatever attitude is available. This is the expected symptom while
  GPS-denied localization is unhealthy — which, per
  [`LANDING-SITES.md`](LANDING-SITES.md) §11, is the stack's current blocking
  problem.
- **The drone never moved.** A vehicle that sits on the pad or hovers in one spot
  builds its map from a single viewpoint, which at a glance is indistinguishable
  from a camera-relative one. Not a bug — no trajectory, no map.

Distinguish them by watching the pose the mapper actually consumes:

```bash
ros2 topic echo --field pose.position /mavros/local_position/pose
```

If those numbers are pinned near zero while `/zed/zed_node/pose_GT` moves, it is
the first case; if both are static, it is the second.

### Step 5 — accumulate

The world points are folded into the two hashes described next.

---

## 4. Accumulation

Two dictionaries, both keyed by integer cell index, both storing a **hit count**:

```python
keys     = np.floor(points / voxel).astype(np.int64)          # (i, j, k)  3-D
cov_keys = np.floor(points[:, :2] / cov_res).astype(np.int64) # (ci, cj)   2-D
```

| | `voxels` | `coverage` |
|---|---|---|
| key | `(i, j, k)` | `(ci, cj)` — x/y only, z discarded |
| cell size | `voxel_size` = **0.15 m** | `coverage_res` = **0.5 m** |
| value | times a feature landed in this voxel | times a feature landed in this column |
| cap | `max_voxels` = **200 000** | `max_coverage_cells` = **100 000** |

**Why a dict and not a dense array.** The arena is mostly empty and its extent is
not known when the node starts. A dense grid would have to guess a bounding box
and would be almost entirely zeros; the hash grows only where something was
actually seen, and needs no origin chosen in advance.

**Why hit counts.** With no feature identity ([§3](#step-1--detect-features)),
repeated observation is the only evidence of durability available. A voxel hit
once may be a depth artefact; a voxel hit fifty times is a real corner. The cloud
does not currently expose the count as a channel — it is used only by the
coverage grid — but it is the natural place to add a confidence filter.

**The caps and how they degrade.** Once `max_voxels` is reached, existing cells
keep counting up but **new cells are silently dropped**, with one warning logged:

```
feature map hit max_voxels (200000); new cells are being dropped.
Raise the cap or the voxel size.
```

Growing forever on an unbounded plane is how a long flight ends in swap, and a
mapper is not allowed to take down the flight it is observing. The failure mode
is deliberately "the map stops extending" rather than "the process dies". At
0.15 m voxels, 200 k cells is a lot of arena — if you hit it, raise `voxel_size`
before `max_voxels`, since memory scales with cell count either way.

---

## 5. Publishing

A timer at `publish_hz` (1 Hz) does both. It **returns immediately if `voxels` is
empty**, so a node that never got past the TF wait publishes literally nothing —
no empty clouds, no heartbeat.

### The point cloud

One point at the centre of every occupied voxel: `xyz = (keys + 0.5) * voxel`.
The message is assembled by hand — three `FLOAT32` fields, `point_step` 12,
`height` 1, `is_dense` True, payload from `xyz.astype(np.float32).tobytes()`.

So the cloud is **quantised, not raw**: two features 5 cm apart become one point.
Cloud size is bounded by the number of occupied voxels, not by flight duration —
hovering in place adds hit counts, not points.

There are **no intensity or rgb fields**, only x/y/z. That matters for the RViz
colour transformer ([§7](#7-seeing-it-in-rviz)).

### The coverage grid

The occupied cells' index bounds become the grid extent, so the published
`OccupancyGrid` is a **tight bounding box that moves and grows** as the flight
proceeds — `info.origin` is recomputed every tick from `i_min`/`j_min`. Cells
never observed stay `-1` (unknown).

Observed cells are log-compressed into 0–100:

```python
value = clip(log1p(count) / log1p(50), 0, 1) * 100
```

Linear scaling would be useless here: one heavily textured wall accumulates
thousands of hits and would flatten every lightly-observed patch to zero. The
log makes "seen 1 vs 10 times" as visible as "seen 100 vs 1000", which is the
comparison a search actually cares about. A cell hit ~50 times saturates at 100.

Finally, `msg.data = array.array('b', grid.tobytes())` — a Python list of ints
here would be converted element-by-element by rclpy, the same trap the image
encoders avoid.

---

## 6. Why this is not in the VO node

Both nodes run ORB over the same RGB stream and back-project through the same
depth, so merging them would save one ORB pass. It is deliberately not done.

`visual_odometry_node` is **flight-critical**: the FCU navigates on its output
with GPS disabled, and ArduPilot flags external nav unhealthy if the stream
stutters. `feature_map_node` is a **pure consumer** — it subscribes, it publishes
no pose, it broadcasts no TF, it is referenced by no other node. The worst a bug
in it can do is waste CPU. That separation is worth more than one ORB pass, and
it is why the map node is behind its own `feature_map:=false` switch.

The same reasoning is why `quat_to_matrix` is duplicated here rather than
imported from `hydrone_vision/pad_detector_node` — a mapping node should not pull
in a vision package's ROS entry point, and everything it imports at module scope,
for twelve lines of algebra.

---

## 7. Seeing it in RViz

### The frame situation

The map is published in **`map`**, because it is built from
`/mavros/local_position/pose` and MAVROS stamps that with `frame_id: "map"`
(`apm_config.yaml`, `local_position.frame_id`). `pad_map_node` does the same, so
both maps share a frame by construction.

TF, however, comes up as **two disconnected trees**:

```
odom -> base_link -> {zed_camera_link -> ..., down_cam_link -> ...}   VO / zed_wrapper
map  -> map_ned                                                       MAVROS static
```

Nothing estimates a `map → odom` edge, in sim or on the drone, so
`tf2_echo map base_link` fails with *"Tf has two or more unconnected trees"* and
RViz can show the map or the vehicle but never both.

`landing_sites.launch.py` now closes the gap with a static identity
`map → odom` (`map_odom_tf:=false` to disable). **Identity is an approximation,
not the answer**: `odom` is the VO's origin (wherever the drone booted) and `map`
is the FCU's EKF origin, so they coincide at takeoff and diverge afterwards by
exactly the accumulated VO drift. That offset *is* the localization error — the
node declares it zero so the frames connect. When something eventually estimates
it, that estimator publishes `map → odom` and this node goes away.

This is **not** done by setting MAVROS `local_position.tf.send: true`. That would
broadcast `map → base_link` while the VO already broadcasts `odom → base_link`.
Two parents for one frame is not a tree, and TF rejects it.

### RViz settings that actually matter

**Fixed Frame** → `map`.

**PointCloud2** on `/hydrone/map/features`:

| Setting | Value | Why |
|---|---|---|
| Style | `Boxes` | shows the voxel structure |
| Size (m) | `0.15` | match `voxel_size`; the default 0.01 m renders a few hundred points as almost nothing, which looks exactly like an empty topic |
| Color Transformer | `AxisColor` (Z) or `FlatColor` | the cloud has only x/y/z — there is no intensity field to colour by |

**Map** on `/hydrone/map/coverage`:

| Setting | Value | Why |
|---|---|---|
| Durability Policy | **`Volatile`** | RViz2's Map display defaults to *Transient Local*; the publisher is Volatile, so the QoS is incompatible and the display connects to nothing — with no obvious error |
| Color Scheme | `costmap` or `raw` | the value is an observation count, not occupancy; `map` colouring reads it as walls |

### When the display stays empty

In order of likelihood:

1. **RViz is running as a different user than the ROS nodes.** DDS discovery is
   UDP multicast and works across users, so the topic appears in RViz's dropdown
   and `ros2 topic info` shows the publisher — but same-host delivery falls back
   to shared memory, and the reader's `/dev/shm` segments are owned by whoever
   created them. A publisher that cannot write into the subscriber's segment
   delivers nothing, silently. Check with `ros2 topic hz` **as each user**: if one
   sees data and the other does not, this is it. In the container the stack runs
   as `hydrone`, so run RViz as `hydrone` too.
2. **The mount TF never arrived** — `_ensure_mount_tf` is still failing, so
   nothing was ever accumulated and nothing is ever published. Look for
   `waiting for TF base_link -> zed_left_camera_optical_frame` in the log.
3. **No pose** — `/mavros/local_position/pose` is silent (MAVROS down, or the EKF
   has no position), so every frame returns early.
4. **Fixed Frame is not `map`**, or the `map → odom` link is missing and you are
   viewing from `odom`.
5. **Point size left at the default**, so the cloud is drawn but invisible.

And if it renders but looks wrong rather than absent — the cloud appearing to
sit on the drone instead of spread across the arena — that is the `p_base`
question, not a frame or transform question. See
[Is the placement actually global?](#is-the-placement-actually-global).

Once it is up, the health check is `ros2 topic hz /hydrone/map/features` — it
should read 1.0 Hz — and `ros2 topic echo --once --field width
/hydrone/map/features`, which is the live voxel count.

---

## 8. Parameters

| Param | Default | Meaning |
|---|---|---|
| `max_features` | 400 | ORB keypoint budget per frame |
| `voxel_size` | 0.15 | cloud quantisation, metres |
| `coverage_res` | 0.5 | coverage cell size, metres |
| `min_depth` / `max_depth` | 0.4 / 20.0 | valid depth band, metres |
| `max_voxels` | 200000 | hard cap; new cells dropped past it |
| `max_coverage_cells` | 100000 | same, for the coverage grid |
| `publish_hz` | 1.0 | snapshot rate for both outputs |
| `process_hz` | 4.0 | frames per second actually processed |
| `in_rgb` / `in_depth` / `in_info` | ZED wrapper names | input topics |
| `pose_topic` | `/mavros/local_position/pose` | where the drone's pose comes from |
| `optical_frame` / `base_frame` | `zed_left_camera_optical_frame` / `base_link` | mount TF lookup |
| `cloud_topic` / `coverage_topic` | `/hydrone/map/features` / `/hydrone/map/coverage` | outputs |

The world frame is **not** a parameter — `self.world_frame = "map"` is hardcoded.
`pad_map_node` exposes the same thing as a `world_frame` parameter; the
inconsistency is real but harmless while both are `map`.

---

## 9. What it is not

- **Not SLAM.** There is no loop closure, no pose graph, no bundle adjustment,
  and no feedback into localization. The map inherits every metre of VO drift
  from the pose it is given, so a long flight produces a *smeared* map rather
  than a wrong one — the same wall observed before and after the drift lands in
  two places.
- **Not an obstacle map.** Sparse ORB corners are not a safe basis for avoidance:
  a blank white wall produces no features at all and would read as free space.
  `landing_sites.launch.py` has no forward obstacle avoidance; it flies over the
  arena's 1.5 m structure at `cruise_alt` 2.5 m.
- **Not consumed by the search.** The coverage grid answers "where have I looked?",
  which is what a search needs before claiming there is nothing left to find — but
  `pad_mission_node` flies a fixed bounded spiral and never reads it. Wiring it in
  (skip a well-seen leg, re-fly a poorly-seen one) is the natural next step and is
  deliberately deferred until the fixed pattern has been flown end to end. See
  [`LANDING-SITES.md`](LANDING-SITES.md) §11.
- **Not flown.** As of 2026-08-19 the mapper has been observed publishing in sim
  (767 voxels, a 52×36 coverage grid at 0.5 m) but the closed-loop mission has
  never got airborne, so no map has been built over a real trajectory.
