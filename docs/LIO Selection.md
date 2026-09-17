---
tags: [hydrone, phase4, decision]
---
# LIO Selection

Back to [[Hydrone]]. Which existing lidar-inertial odometry core Phase 4 builds on, and why. Decided 2026-09-17 in the autonomous session (Ivo asked for an existing core with our hooks on top, not registration written from scratch). How it's wired: [[LIO Odometry]].

## Decision
**FAST-LIO2, ROS 2 branch** (`hku-mars/FAST_LIO`, branch `ROS2`, commit `a4743b0`, 2025-01-15), vendored at `src/fast_lio` unchanged apart from dropping its 128 MB `doc/` folder.

## What we needed
| Need | Why |
|---|---|
| Livox Mid-360 as-is (`livox_ros_driver2` CustomMsg, built-in IMU) | the real driver's output, so the sim contract is the real one |
| ROS 2 Humble | the whole stack |
| Runs on a CPU the size of a Jetson Nano (4× Cortex-A57) or a Raspberry Pi 5 | Ivo's stated compute target |
| A clear place for the team's prior and gates | [[Motor Physics Prior]], [[Degraded Sensing]] |
| Survives narrow, repetitive geometry | [[Phase 4 Maze Mission]] |
| Small enough to read and fork | we will modify it later |

## Candidates
| Core | Mid-360 / ROS 2 | CPU cost | Hooks for our layers | Why not (for now) |
|---|---|---|---|---|
| **FAST-LIO2** | official ROS2 branch ships `mid360.yaml`, reads CustomMsg directly | low; upstream lists ARM boards (Khadas VIM3, TX2, RPi 4B) | iEKF: IMU propagation step is where a motor/attitude prior can enter; output is one Odometry to gate | **picked** |
| Point-LIO | ROS 2 only through community ports (`dfloreaa/point_lio_ros2`) | higher: updates per point | same filter family | better at violent motion we don't fly; no official ROS 2 |
| Faster-LIO | ROS 1 upstream | lower than FAST-LIO (iVox) | same as FAST-LIO | no maintained ROS 2; switch later if the Nano can't keep up |
| DLIO | ROS 2 on a `feature/ros2` branch; PointCloud2 input | low | geometric observer, less natural place for a covariance-weighted prior | would need a CustomMsg→PointCloud2 step; branch, not main |
| iG-LIO | ROS 2 port is recent third-party work (2026 paper on fixing toolchain failures in the port) | low-moderate | GICP + ESKF | port maturity |
| LIO-SAM | Mid-360 only through adaptations, wants a 9-axis IMU and ring field | high (factor graph + features) | factor graph is flexible | Mid-360 IMU is 6-axis; too heavy for the Nano |
| KISS-ICP | official ROS 2, MIT | very low | its constant-velocity prediction is exactly where our physics prior would slot in | lidar-only: drifts under drone-style motion; we'd rebuild the IMU fusion it lacks |
| GLIM | ROS 2, modular | heavy (GPU preferred, global optimization) | very flexible | way over a Nano |
| MOLA-LO | ROS 2 native, pipeline in YAML | moderate | very configurable | heavier stack, less field use on small drones |
| FAST-LIVO2 | lidar + camera | high | future RGB fusion | later, see [[RGB Camera Enhancement]] |

Ran against our own requirements, not a benchmark here. Accuracy claims in the literature mostly come from ground vehicles and handheld rigs, not quadrotors (the LiPO 2024 comparison, for example, is ground-only).

## Why FAST-LIO2 wins for us
- It's the only candidate whose **official** repo runs the Mid-360 on ROS 2 Humble with no glue. The simulator can then publish exactly what `livox_ros_driver2` publishes on the drone, and switching to the real sensor changes nothing downstream.
- Direct point-to-map registration with no feature extraction. The arena is texture-poor and geometrically simple ([[ZED Feature Map]] measured 46 ORB keypoints), so methods that need edges and planes starve.
- An ikd-tree map and an iterated EKF are cheap enough for ARM. Upstream runs on RPi 4-class boards; `point_filter_num` and the voxel filter sizes are the knobs for the Nano.
- **Hooks for the team pipeline** (the reason Ivo wanted an existing core):
  - *Now*: outside the core. `motion_prior_node` compares FAST-LIO displacement with motor physics, and `lio_odom_adapter` withholds poses from the EKF when they disagree. No fork needed.
  - *Later*: inside the core. The prior can become a pseudo-measurement in the iEKF update, and per-sensor health can scale `acc_cov` / `gyr_cov` / point noise online.
- ~3k lines in a few files. The team can read it.

## Known risks, stated plainly
- **GPL-2.0.** Fine for competition use. Anything distributed that links it inherits GPL.
- **Degenerate geometry** (long corridors, the maze interior, open field). FAST-LIO has no explicit degeneracy detection, and this is precisely what [[Degraded Sensing]] must add.
- **Init needs a still IMU.** It averages the first IMU samples for gravity, so the drone must sit still at startup (it does, on the ground).
- **The ROS2 branch is community-maintained** inside the official repo. The last commit was 2025-01; pinned by commit.
- **Jetson Nano headroom is unmeasured.** Start there if real-time breaks: Faster-LIO's iVox or a smaller `point_filter_num`.
