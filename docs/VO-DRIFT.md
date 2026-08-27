# VO drift, and the zero-velocity update

The visual odometry was inflating distance by **10x**, and none of it came from
flying.

## What was measured

`odom_error_node` logs VO against BiguaSim ground truth, per synchronised pair,
to a CSV. From `odom_error_20260827_010810.csv` — 159,814 samples over a flight
of 51.5 m:

```
final position error   5.28 m
peak position error   11.98 m
final yaw error       -69.6°     (peak |180°|)
median drift          13.4% of path
```

In an 8x8 m arena, an estimate that is 5 to 12 m out is not an estimate.

## Where it came from

Splitting the samples by whether the vehicle was actually moving:

```
TOTAL        ground truth  51.5 m    VO reported 527.3 m    10.2x
94% STILL    ground truth   0.0 m    VO reported 441.8 m    1.61 mm per frame
 1% MOVING   ground truth  50.6 m    VO reported  58.2 m    1.15x
```

Read that middle row again: across 94% of samples the vehicle had moved less
than **0.01 mm** by ground truth, and the VO claimed **441.8 metres**.

While genuinely moving, the same VO is respectable — 1.15x scale error is
ordinary for feature-based odometry with no loop closure. **The estimator was
not bad at odometry. It was bad at standing still.**

## Why every existing gate let it through

`visual_odometry_node` already rejected bad frames: too few matches, too few
RANSAC inliers, a poor inlier ratio, and a step too LARGE to be physical
(`max_step_m`). A still camera fails none of those. Feature positions carry
sub-pixel noise, so PnP returns a few millimetres of motion with inliers to
spare — a confident, well-conditioned, entirely fictional answer. It passed
every test and was integrated forever.

Nothing rejected a step too SMALL to be real.

## The fix

A zero-velocity update: below a threshold, treat the frame as motionless and
hold the pose instead of integrating.

```python
if step_t < self.min_step_m and step_r < self.min_step_rad:
    return                      # holding pose
```

| parameter | default | why |
|---|---|---|
| `min_step_m` | 0.005 | 3x above the measured 1.61 mm noise floor, ~10x below one frame of real flight (0.5 m/s at 10 Hz is 50 mm) |
| `min_step_deg` | 0.25 | NOT measured — the CSV logs accumulated yaw error, not per-frame rotation noise. A starting value, worth measuring |

**BOTH must be small.** Gating on translation alone would delete Phase 1's
search, which turns on the spot: zero translation, real rotation.

The real ZED does the same thing — the SDK detects a static state and holds its
pose. That is why the hardware looked better than the simulation.

## Verifying it

The CSV now lands in `./logs/` on the host (docker-compose bind-mounts it to
`/ws/logs`, and `odom_error_dir` defaults there). **Before this it was written
to `/home/hydrone/` inside the container and destroyed on every `down`** — the
file analysed above was recovered from a stopped container by luck.

After a flight:

```
ls logs/odom_error_*.csv
```

Columns are documented in `odom_error_node.py` (`CSV_COLUMNS`). The two that
matter: `err_norm` (position error in metres) and `drift_pct` (that error as a
percentage of distance actually travelled). The node also logs
`VO: still (...); holding pose [N frames so far]` every 30 s, which is the
count of frames the gate suppressed.

**The numbers in this document are from BEFORE the fix.** Re-measure and
compare — that comparison is what says whether the octomap can be trusted over
a ten-minute attempt, and whether SLAM is worth building at all. See
[`OCTOMAP.md`](OCTOMAP.md).

## The other half: it kept losing the scene

With the ZUPT in, the logs stopped being about noise and started being about
tracking. Of 27 VO warnings in the next flight, **20 were lost tracking** —
`only 0 usable matches`, `only 1`, `only 5` — against 1 from the ZUPT.

The reason was already measured in this repo on 2026-08-20, in
`feature_map_node`'s docstring: **46 ORB keypoints in an entire frame, all
inside a 30-pixel band** on the horizon line. The arena is matte white wall on
blown-out white floor under a smooth sky.

The arena is not textureless. It is **low contrast**, which is a different
problem with a standard answer, and the node was configured for neither:

* `cv2.ORB_create(nfeatures=1000)` uses OpenCV's default FAST threshold of
  **20**, tuned for ordinary scenes.
* Nothing equalised the image first, so the faint gradients that do exist —
  panel seams, scuffs, the shading in a corner — were flattened between a
  bright floor and a bright sky.

MEASURED on a synthetic frame built to match that description (white wall 214,
floor 246, sky 232, seams 2-6 grey levels deep, sensor noise σ=1.2):

```
ORB default (fastThreshold=20), no CLAHE          0 keypoints
fastThreshold=7, no CLAHE                         0 keypoints
fastThreshold=20, with CLAHE                      1 keypoint
fastThreshold=7 + CLAHE   <- new defaults       489 keypoints, spread 412 px
```

**Neither change works alone.** Lowering the corner threshold finds nothing
when there is no gradient to find; equalising alone lifts the gradient but not
past a threshold of 20. Together they recover the scene, and — as important —
the keypoints spread across 412 of 480 rows instead of the 30-pixel band. A
tracker cannot survive a turn on features that all sit on one line.

`clahe_clip: 0` disables the equalisation for a camera on a textured scene,
where it is cost without benefit.

**NOT yet validated on a real sim frame.** The measurement above is synthetic,
built from the description of the 2026-08-20 measurement rather than from that
frame, which is no longer on disk. What the flight log should show is the
`only N usable matches` warnings becoming rare.

## What this does NOT fix

The 1.15x scale error while moving is untouched, and it is real: over 50 m of
path that is ~7 m of accumulated error with no loop closure to pull it back.
Whether that matters depends on how far the drone flies between landings — in
Phase 1 the legs are short, which is exactly why `hydrone_nav.route` picks the
nearest pad rather than the shortest tour.
