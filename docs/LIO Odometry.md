---
tags: [hydrone, phase4, odometry]
---
# LIO Odometry

Back to [[Hydrone]]. The minimal Phase 4 odometry: FAST-LIO on the Mid-360, flying ArduPilot's EKF with no ground truth. Why FAST-LIO: [[LIO Selection]]. Sensor: [[Livox Mid-360 Sim]]. Map built on top: [[Persistent Map]].

## Run it
```
scripts/docker_up.sh --dev --phase4 -d        # stack; EKF on the LIO
docker cp scripts/phase4_lio_check.py joao_pessoa_2026-hydrone-1:/tmp/
docker exec -u hydrone joao_pessoa_2026-hydrone-1 bash -lc \
  'source /ws/install/setup.bash; python3 /tmp/phase4_lio_check.py'
```
The check script arms, takes off to 1 m, hovers, flies a 2 m square, lands, and prints LIO-vs-ground-truth drift. The per-pose drift CSV is in `maps/logs/`.

Launch args: `ext_nav:=ground_truth` (debug only: tune the airframe without the estimator), `gate:=false`, `map_name:=`, `load_map:=true`, `measure_drift:=false`.

## Viewing it in RViz
`rviz2 -d src/hydrone_lio/rviz/phase4.rviz` (on the host, same `ROS_DOMAIN_ID`) or, inside the container, `rviz2 -d /ws/install/hydrone_lio/share/hydrone_lio/rviz/phase4.rviz`.
- **Fixed Frame must be `odom`.** RViz defaults to `map`, which nothing publishes, so every display waits and drops ("queue is full").
- View `/livox/lidar_pc` (10 Hz scan), `/hydrone/lio/odom_raw`, `/hydrone/map/voxels`, `/cloud_registered`.
- **Not** the raw engine topics under `/biguasim/uav0_id0/...`: the per-tick `Mid360` cloud comes at ~37 msg/s and fills RViz's queue before the LIO TF for that instant arrives.
- Stamps are simulation time and run minutes behind the wall clock (the sim is ~0.2× real time). That's expected. Everything published for viewing uses the same clock, so RViz is fine with it.
- `odom → camera_init` is published statically by `lio_odom_adapter`, so FAST-LIO's own frames join the tree.

## Pipeline
| Node | In | Out |
|---|---|---|
| `fastlio_mapping` (vendored) | `/livox/lidar`, `/livox/imu` | `/Odometry` (camera_init→body), `/cloud_registered` |
| `lio_odom_adapter` | `/Odometry`, `/hydrone/lio/consistency` | `/hydrone/lio/odom_raw`, `/hydrone/lio/odom` (gated), TF `odom→base_link` |
| `motion_prior_node` | ESC telemetry, `/mavros/imu/data`, `/hydrone/lio/odom_raw` | `/hydrone/lio/consistency` (DiagnosticStatus per 0.5 s) |
| `vision_odom_bridge` (`lio_nav`) | `/hydrone/lio/odom` | `/mavros/vision_pose/pose` |
| `odom_error_node` (`lio_error`) | `odom_raw` + BiguaSim GT | CSV, measurement only |

- Config: `hydrone_lio/config/fast_lio_mid360_sim.yaml`: `point_filter_num 3`, voxel filters 0.4 m (Jetson Nano budget), blind 0.35 m, extrinsic identity.
- ArduPilot (`kopis_sitl.parm`): yaw from external nav (`EK3_SRC1_YAW 6`), compass off. The LIO frame's zero is the takeoff heading, which a compass would fight.

## Motion prior + gate (the team's first hook)
See [[Motor Physics Prior]] for the plan. What exists:
- `thrust = k_eta·Σω²` along body z, tilted by the FCU attitude with yaw removed. Minus gravity, double-integrated over 0.5 s from the LIO's own start velocity, gives `d_pred`.
- A window is OK when `|d_lio − d_pred| ≤ 0.10 m + 0.5·|d_pred|`.
- After 3 bad windows in a row the adapter stops forwarding LIO poses to the EKF until an OK window arrives. ArduPilot coasts on its IMU meanwhile.
- In sim, ESC RPM comes from the bridge on `/hydrone/sim/esc_telemetry`, because ArduPilot's JSON SITL has no RPM input. On the drone it's `/mavros/esc_telemetry/telemetry`.
- Known blind spot: a slow, smooth drift passes, because `v0` comes from the LIO. That needs the revisit layer ([[Degraded Sensing]]).
- Proved useful on flight 1: the LIO lost track while the airframe was being flung around, the gate tripped, and the EKF failsafe landed the vehicle.

## Results (sim, 2026-09-17)
| Run | Nav | Outcome | LIO vs GT |
|---|---|---|---|
| static, before fixes | — | on the ground | 1.0 m / 8° within 2 s |
| static, after fixes | — | on the ground | 4.5 cm / 0.5° yaw in 17 s |
| flight 1 (inherited tune) | LIO | ±4 m swings, 8 m fling, gate trip, failsafe land | lost track in the fling |
| tune 3 (Kopis tune) | GT (LIO shadow) | 2 m square, landed | **max 9.6 cm**, final 7.7 cm, 0.8° yaw; 1/50 windows flagged |
| **LIO 2** | **LIO** | 2 m square, landed | **max 7.5 cm**, final 5.5 cm, 0.8° yaw; 6/81 flagged, gate never tripped |

The LIO-nav square came out 3.3 × 4.3 m on ground truth: the position loop overshoots the legs. The LIO wasn't the cause (it agreed with GT within 7.5 cm).

## What broke on the way (and the fixes)
1. **Free-fall init.** BiguaSim drops the spawned drone 0.18 m, and FAST-LIO takes gravity from its first ~10 IMU samples. The mimic now waits 3 s of sim time.
2. **Lever-arm noise.** Moving the IMU to the lidar with a 5 ms gyro difference turned 0.07 g of sim jitter into 0.5 g. The difference is now taken over 20 ms.
3. **Wrong thrust curve.** BiguaSim's Kopis thrust is exactly quadratic in PWM, so `MOT_THST_EXPO 1.0`, `MOT_THST_HOVER 0.3`. That removed the big position swings.
4. **Hidden 6.8 Hz limit cycle.** Found by FFT of a recorded bag, and invisible in position (±8 cm). Gyro std was 2.3 rad/s, ~100 rad/s² angular acceleration, and FAST-LIO fell through the floor within 2 s of the climb ending. The firmware-default rate gains are far too hot for a T/W 3.4 quad: `ATC_RAT_RLL/PIT P 0.07 I 0.07 D 0.0015` brought gyro std to 0.004.
5. **Sim clock.** Sensors are stamped with simulation time, or FAST-LIO would integrate wall-clock dt while the sim runs at ~0.2× real time.

## Not done / next
- The Jetson Nano CPU budget hasn't been measured.
- Position-loop overshoot on LIO nav (PSC gains or `WP_ACC`).
- Degeneracy detection (FAST-LIO has none). The maze interior is the first real test: [[Phase 4 Maze Mission]].
