# ZED visual odometry — real VO estimate vs. ground truth

How the drone now gets its GPS-denied position estimate from the ZED, and how the
two odometry streams (`/zed/zed_node/odom` and `/zed/zed_node/odom_GT`) relate.

> Context: the CBR 2026 arena bans GPS, so ArduPilot flies on **external-nav**
> (VISION_POSITION_ESTIMATE) instead. See [`DEVELOP-PIPELINES.md`](DEVELOP-PIPELINES.md)
> for the GPS-denied flight plumbing this builds on.

---

## The two odometry streams

| Topic | Producer | What it is | Used for |
|---|---|---|---|
| `/zed/zed_node/odom` | **`visual_odometry_node`** | a **real VO estimate** computed from the ZED RGB-D images | **flight** — feeds `vision_odom_bridge` → MAVROS → EKF |
| `/zed/zed_node/odom_GT` | **`zed_mimic`** | BiguaSim **ground truth** (perfect pose from the sim) | **debug/reference only** — never fed to the flight controller |

Both carry `nav_msgs/Odometry` in the same `odom -> base_link` frame (NWU-style),
so they are **directly comparable**. The gap between them *is* the VO error/drift.

```
zed_mimic ─┬─> /zed/zed_node/rgb,depth,camera_info ─┐
           └─> /zed/zed_node/odom_GT  (ground truth) │  (debug only)
                                                     ↓  (VO input)
visual_odometry_node ─> /zed/zed_node/odom ─> vision_odom_bridge ─> MAVROS ─> EKF3
```

### Why keep `odom_GT`?
`odom_GT` is the **oracle**. Because the sim knows the drone's exact pose, we can
overlay it against the VO estimate and measure exactly how far the VO drifts:

- **In RViz2:** add two `Odometry` displays (`/zed/zed_node/odom` and
  `/zed/zed_node/odom_GT`), `Fixed Frame = odom`. They start coincident; the growing
  gap while flying is the VO drift.
- **Numerically:** subtract the two positions (zero `odom_GT` at its first sample,
  since the VO starts its origin at the first frame). Drift ÷ distance-traveled is the
  headline VO-quality number.

On the **real drone** this stream simply doesn't exist (there is no ground truth) —
`odom_GT` is a simulation-only debugging aid. The flight path (`/zed/zed_node/odom`)
is identical in sim and reality, so nothing downstream changes.

---

## How the real VO works (`visual_odometry_node`)

It reproduces the **core** of the ZED 2i's positional tracking. Per Stereolabs, the
ZED SDK's tracking is a stereo visual-inertial SLAM whose motion core is
**Stereo Visual Odometry — it tracks visual features in 3D across frames**. This node
does exactly that core: detect features → track across frames → recover their 3D
positions from depth → solve the frame-to-frame camera motion → accumulate.

### Inputs
- **RGB** (`/zed/zed_node/rgb/image_rect_color`) — where features are found/matched.
- **Depth** (`/zed/zed_node/depth/depth_registered`, meters, NaN = invalid) — gives each
  2D feature a 3D position. This is the sim's stereo output, doing the ZED's job.
- **CameraInfo** — intrinsics `K` (`fx, fy, cx, cy`) for 2D↔3D projection.

### Per-frame pipeline
On every RGB frame:

1. **Detect features.** ORB extracts up to `max_features` (default 1000) keypoints +
   binary descriptors from the grayscale image.

2. **Match to the previous frame.** Brute-force Hamming matching + **Lowe's ratio
   test** (`match_ratio` = 0.75) keep only confident matches — i.e. the *same physical
   corner* seen in frame *t‑1* and frame *t*. This is "tracking features across frames".

3. **Back-project the previous matches to 3D.** Using the previous frame's depth + `K`,
   each matched pixel `(u, v)` becomes a 3D point in the camera optical frame:
   ```
   x = (u - cx)/fx · d      y = (v - cy)/fy · d      z = d
   ```
   Points with invalid depth (NaN, or outside `[min_depth, max_depth]` = 0.3–20 m) are
   masked out. Depth here is what makes this **stereo** VO (scale is recovered, unlike
   monocular). Vectorized in numpy — no per-pixel Python loop.

4. **Solve the motion (`solvePnPRansac`).** For each match we now have a **3D point**
   (frame *t‑1*) and the **2D pixel** where it reappears (frame *t*). PnP finds the
   camera rotation+translation explaining those 3D→2D correspondences; **RANSAC**
   rejects outliers (moving objects, bad matches). Needs ≥ `min_inliers` (12) inliers.
   Result: the relative transform `T_cur_prev`.

5. **Accumulate into a global pose.** The camera's motion is the inverse of the point
   transform, chained onto the running pose:
   ```python
   pose_opt = pose_opt @ inv(T_cur_prev)
   ```
   The odom **origin is wherever the drone started** (first frame = identity).

6. **Convert frames & publish.** The pose is in the optical convention (Z fwd, X right,
   Y down); the pipeline expects base_link NWU (X fwd, Y left, Z up). A fixed rotation
   `C = R_BASE_FROM_OPTICAL` maps it: `T_odom_base = C · T_optical · C⁻¹`. Published as
   `nav_msgs/Odometry` on `/zed/zed_node/odom` (+ TF), forwarded by `vision_odom_bridge`
   to MAVROS exactly like the ground-truth path used to be.

### Robustness: "holding pose"
If a frame has too few features, too few valid-depth matches, or PnP fails / yields
< `min_inliers`, the node logs `holding pose` and **re-publishes the last pose** rather
than jumping. This keeps a continuous odom stream — ArduPilot's external nav flags
"not healthy" if the feed drops.

### Tunable parameters
Exposed as ROS params (no code edit needed):

| Param | Default | Meaning |
|---|---|---|
| `max_features` | 1000 | ORB keypoint budget (more = steadier, more CPU) |
| `match_ratio` | 0.75 | Lowe ratio (lower = stricter matches) |
| `min_depth` / `max_depth` | 0.3 / 20.0 | valid depth band for back-projection (meters) |
| `min_inliers` | 12 | minimum RANSAC inliers to trust a motion estimate |

---

## Faithful to the ZED — and where it's simpler

**Faithful:** feature detection → cross-frame tracking → 3D-from-stereo → PnP motion →
accumulation. That *is* the ZED's visual-odometry core, not a ground-truth passthrough.

**Simplified on purpose** (documented in the node):
- **No IMU fusion.** The real ZED is visual-*inertial*; this is visual-only, so fast
  rotation / motion blur hurt more.
- **No landmark map / loop closure.** The ZED keeps a sparse map to correct drift; this
  is pure frame-to-frame, so **it drifts** over a flight — which is exactly what
  `odom_GT` lets you measure.
- **Frame-to-frame** (not frame-to-keyframe) — simplest, drifts a little faster.

These three are the natural accuracy upgrades. Complementary **sensors** (e.g. a
downward rangefinder for altitude) are handled separately in the pipelines
experiments, not here — this node is only the ZED VO.

---

## Verifying it (once the sim is healthy)
1. `ros2 topic hz /zed/zed_node/odom` — VO is producing.
2. Compare against `/zed/zed_node/odom_GT` in RViz2 / PlotJuggler — watch the drift grow.
3. Watch the node log for `holding pose` warnings — frequent ones mean weak tracking;
   tune `min_depth`/`max_depth`/`max_features` first.
