---
tags: [hydrone, phase4, sensors, sim]
---
# Livox Mid-360 Sim

Back to [[Hydrone]]. How the Phase 4 simulator produces a Mid-360 that FAST-LIO can't tell from the real one. Part of [[Phase 4 Pipeline]], consumed by [[LIO Odometry]].

## Contract (same as the real driver)
| Topic | Type | Frame | Rate |
|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` (xfer_format 1), `offset_time` in ns | `livox_frame` | 10 Hz, ≤20k pts |
| `/livox/imu` | `sensor_msgs/Imu`, accel **in g** | `livox_frame` | 200 Hz (one per sim tick) |
| `/livox/lidar_pc` | `PointCloud2` xyz (sim extra, only built when someone subscribes) | `livox_frame` | 10 Hz |
| TF `base_link → livox_frame` | static, from the scenario file's `location` | | |

On the drone, `livox_ros_driver2` publishes the same topics and `livox_mimic_node` isn't launched. `src/livox_ros_driver2` here is **messages only** (no Livox-SDK2).

## How
```
BiguaSim RaycastLidar "Mid360"  --(per tick, body frame)-->  ardubridge  --PointCloud2-->  livox_mimic_node  --> /livox/lidar
BiguaSim IMUSensor              ------------------------->  ardubridge  --Imu--------->  livox_mimic_node  --> /livox/imu
```
- **Engine raycasts, not depth images.** BiguaSim's packaged binary has a CARLA-style `RaycastLidar` (marked WIP upstream). It's configured in `config-KopisX8.yaml`: 40 channels, 200k pts/s, 10 Hz, 70 m.
- **Non-repetitive pattern.** The engine's rings are fixed, so `ardubridge_node._lidar_step` tilts the sensor by two incommensurate ±3° roll/pitch sines every tick. The engine FOV is set to −4..49° so that the wobble lands on the real −7..52°. Points come back in the **agent body frame**, so the tilt needs no undoing.
- **What the mimic adds** (`hydrone_bringup/livox_mimic_node.py`):
  - 100 ms windows cut **by stamp** (sim time), so a slow sim gives the same scans, just later
  - base_link → livox_frame transform
  - 2 cm σ range noise along the beam (datasheet), 2% random dropouts, 0.1 m blind zone, 70 m max
  - down-sampling to ≤20k points
  - `line` = one of 4 elevation bands (the Mid-360 reports 4 lines)
- **IMU.** The body IMU's reading is moved to the lidar origin (lever arm: α×r + ω×(ω×r)), rotated into livox_frame and divided by g. The real unit's IMU is ~4 cm from its optical centre, so FAST-LIO's extrinsic is identity in sim.
- **Clock.** `stamp_clock:=sim` stamps every bridge topic `anchor + sim_time`. FAST-LIO integrates the IMU over those stamps. With wall stamps a 0.3× sim would look like a drone moving 3× slower than its accelerations say.

## Measured (probe `tools/phase4/probe_raycast_lidar.py`, 2026-09-17, CPU on `powersave`)
| Config | Sim speed |
|---|---|
| depth camera 256² only | 40.9 tick/s |
| RaycastLidar only | 59–74 tick/s |
| both | 38 tick/s |

- **Geometry:** lidar points vs the depth camera looking the same way give a median error of 8 mm (the camera's pixel quantization). UE's y axis is flipped (left-handed); the encoder flips it.
- **Repetition, over 0.1 s windows:**

  | | Distinct elevations | NN distance vs 1 s earlier | Exact repeats |
  |---|---|---|---|
  | no wobble | 40 | 3.5 mm median | 15% within 1 mm |
  | ±3° wobble | 646 | 1.7 cm median | 0% |

## Bugs found on the way
- `biguasim/sensors.py` `RaycastLidar.sensor_data` returned **0 points** every tick: it kept `previous_frame` as a view of the live shared buffer, so its "unchanged tail" trim always matched. Fixed on branch `phase4-raycast-lidar-fix` of bs-drone-competition.
- `sensor_data_encode.py`:
  - `IMUSensor` was listed with a `Bias` topic that had no encoder, which crashed the bridge
  - `DynamicsSensor/IMU` put angular *acceleration* in `angular_velocity` (this also fed phase 1's zed_mimic IMU)

## What it is NOT
- Real Mid-360 scan geometry (a rosette through a rotating prism). The wobbled rings match its coverage and its non-repetition, not its exact pattern or density distribution.
- No reflectivity (constant 100), no multi-return, no sun/glass/dust effects: [[Degraded Sensing]] has to be tested some other way.
- **Local sim only.** A `--world` remote bridge can't rotate sensors, and `phase4_sim.launch.py` refuses to start with `WORLD_ADDRESS` set.
- The old approach (one depth camera spun 60°/tick, three 60° wedges per revolution) is gone.
