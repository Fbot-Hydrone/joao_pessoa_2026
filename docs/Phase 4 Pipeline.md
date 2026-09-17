---
tags: [hydrone, phase4, plan]
---
# Phase 4 Pipeline

Back to [[Hydrone]]. Overview and roadmap for the Phase 4 autonomy stack — the
Kopis X8 flying on its own sensing, no GPS, no sim ground truth in the loop.

## The airframe

Kopis X8 with a Livox Mid-360 lidar (360°×59° FOV, non-repetitive scan
pattern, built-in IMU, 10 Hz), a Pixhawk flight controller (ArduPilot —
attitude + IMU), ESCs with RPM telemetry, and in the future an RGB camera. No
GPS. Compute target is the original Jetson Nano, or at worst a Raspberry Pi
5 — everything in this pipeline must be CPU-light.

This is a different problem from [[Phase 1 Mission]]: Phase 1 trusts a
proprietary ZED VIO and sim ground truth is a legitimate debugging aid. Phase
4's whole job is proving it can navigate on its own sensing, so the LIO must
be validated in closed loop, and ground truth is only ever used to *measure*
drift, never to fly.

## Layered roadmap

The basic layer gets the drone flying on lidar-inertial odometry with a
physics-based sanity check. The robustness layer — degraded sensing,
fallbacks — is a separate track that hardens the same base, not a
prerequisite for it.

| step | what | status |
|---|---|---|
| 0 | Mid-360 replica in sim (BiguaSim RaycastLidar probe, or resampled depth cameras) publishing `/livox/lidar` + `/livox/imu` | Now — see [[Livox Mid-360 Sim]] |
| 1 | Pick an existing LIO core (FAST-LIO2 expected) with a documented rationale | Now — see [[LIO Selection]] |
| 2 | Minimal base: LIO in closed loop via MAVROS `vision_pose`, `motion_prior_node` (RPM + attitude → expected displacement, gate), drift measured against ground truth | Next — see [[LIO Odometry]], [[Motor Physics Prior]] |
| 3 | Persistent map: voxel map + deduplicated plain cloud + keyframes, saved/reloadable | Next — see [[Persistent Map]] |
| 4 | Maze mission (stretch) | Later — see [[Phase 4 Maze Mission]] |

Alongside, not gating the above: [[Degraded Sensing]] (detecting and modelling
sensor lies) and [[Sensor Fallbacks]] (the ladder down to RPM + physics dead
reckoning). [[RGB Camera Enhancement]] is future work layered on top once the
camera exists.

## Data flow (target)

| sensor | node | topic | consumer |
|---|---|---|---|
| Livox Mid-360 (or sim replica) | `livox_ros_driver2` / [[Livox Mid-360 Sim]] | `/livox/lidar` (CustomMsg), `/livox/imu` | LIO core |
| Pixhawk IMU + attitude | MAVROS | `/mavros/imu/data`, `/mavros/local_position/pose` | `motion_prior_node`, LIO |
| ESC RPM telemetry | MAVROS (`ESC_TELEMETRY`) | `/mavros/esc_telemetry` (or equivalent) | `motion_prior_node` |
| `motion_prior_node` | — | expected displacement direction + consistency flag | LIO gating (drops lidar motion misaligned with the prior) |
| LIO core | FAST-LIO2 (expected) | odometry + local map | `vision_odom_bridge` → MAVROS `vision_pose` → EKF3 |
| LIO core | — | keyframes / voxel map | [[Persistent Map]] |
| RGB camera (future) | — | features / loop closure | [[RGB Camera Enhancement]] |

## Why a motion prior at all

Ivo's framing: estimate the direction the drone should displace from each
motor's RPM and the Pixhawk attitude, and discard lidar-derived movements that
are unaligned with that estimated displacement vector, within an error margin.
This is the cheapest possible sanity check on the LIO — it needs no lidar
frame-to-frame matching, only RPM + attitude + rigid-body physics — and it is
the last line of defense in [[Sensor Fallbacks]] when everything else is dead.
Detail in [[Motor Physics Prior]].

## Why voxelize before differencing

Mapping the point cloud into a voxel grid first means scan-to-scan differences
of the Mid-360's non-repetitive pattern matter less, points cluster, and less
compute is spent per frame — important on a Jetson Nano / RPi 5 budget. Diffing
close-in-time voxel snapshots surfaces moving features (what changed, where did
it go); combined with rough drone physics that gives odometry. This is the
"most basic layer" in Ivo's words, underneath the LIO-core-driven approach in
[[LIO Odometry]].

## Status

**Now:** sim sensor replica ([[Livox Mid-360 Sim]]), LIO core selection
([[LIO Selection]]).
**Next:** closed-loop LIO + motion prior + drift measurement ([[LIO Odometry]],
[[Motor Physics Prior]]), persistent map ([[Persistent Map]]).
**Later:** degraded-sensing pipeline ([[Degraded Sensing]],
[[Sensor Fallbacks]]), RGB camera fusion ([[RGB Camera Enhancement]]), maze mission
([[Phase 4 Maze Mission]]).

Sim entry point: `scripts/docker_up.sh --phase4` (see [[Scripts]]) brings up
the Kopis X8 + Mid-360 on ground truth so the airframe holds still while this
pipeline is built — `--ground-truth` and `--no-odom-print` do not apply to it.
Local sim only for Phase 4: the remote-world mode cannot rotate sensors, which
the Mid-360's spin (real or replica) needs.
