# Pipeline 1 — Sensor configuration (Phases 1 & 2)

ArduPilot on Pixhawk + Jetson, GPS-denied. This is the authoritative sensor spec:
**physical mounting** (real drone build guidance), the **EKF3 source setup**, and
**how each piece is represented in the BiguaSim simulation**.

> Related: [`ZED-VISUAL-ODOMETRY.md`](ZED-VISUAL-ODOMETRY.md) (the VIO source) and
> [`DEVELOP-PIPELINES.md`](DEVELOP-PIPELINES.md) (GPS-denied flight plumbing).

---

## Sensor suite

| Sensor | Placement | Function | Simulated? |
|---|---|---|---|
| **ZED 2i** | front-facing, horizontal | VIO — primary XY (+Z, yaw on real drone) | ✅ VIO node on ZED RGB-D |
| **Down camera** | nadir, straight down | base-marker (PrecLand P1) + kit (P2) detection | ⬜ deferred (perception step) |
| **Rangefinder** | nadir, down | Z reference for landing/flare (not primary Z) | ✅ RangeFinderSensor + bridge |
| **IMU** | in Pixhawk, at CG | attitude, rates, inertial dead-reckoning | ✅ SITL / JSON FDM |
| **Barometer** | in Pixhawk | slow Z fallback | ✅ SITL |
| **Gripper** | central, down | pick up the kit (P2) | ⬜ hardware only |

### Physical mounting (real drone — build guidance)
- **ZED 2i** — aligned with the forward axis, lens horizontal (zero pitch in hover).
  Props and landing gear/gripper out of FOV. Baseline/orientation must be accurate —
  misalignment becomes scale/heading error in the EKF. Rigid mount, vibration-isolated
  from motors. Runs on the Jetson (ZED SDK / CUDA), feeds pose to EKF3 via MAVROS/DDS.
- **Down camera** — as close as possible to CoM / landing axis, straight down. Its
  offset vs. drone center and gripper must be calibrated (hand-eye) or precision
  landing/grasping miss. Clean FOV, no legs in frame.
- **Rangefinder (VL53L1X)** — near center, clear beam. Nothing (gear, gripper, kit)
  in the beam path or it reads the drone. If the gripper hangs below, offset the
  rangefinder to clear it. **Specs that drive the mount/config:** I²C ToF, **range
  ~4 m**, **FoV ~27°**, nadir. Because the range is only ~4 m it is **landing-only**
  (`EK3_RNG_USE_HGT=-1`), not a global height source: the base platforms sit at
  0–1.5 m, so on approach the beam sees a **step change** in ground distance as it
  crosses the platform edge — great for flare/precision-landing timing, but it would
  inject a discontinuity into EKF height if fused globally. The narrow 27° FoV keeps
  that step crisp; keep the beam clear of the gripper so the step isn't the drone.
- **IMU** — as close to CG as possible, vibration-decoupled per Pixhawk guidance.
  Vibration here contaminates everything.
- **Barometer** — internal; shield from prop wash / wind (foam over the sensor).
- **Gripper** — on the central axis, aligned with the down camera; designed with the
  landing gear so the drone descends around the kit and grasps it centered (rules
  require the full landing gear inside the base). Must not block the down camera FOV
  or the rangefinder beam.
- **General:** balance CG with the gripper mounted; group all down-facing devices
  (camera, rangefinder, gripper) near center without obstructing each other; keep the
  ZED forward with props and body out of its FOV.

---

## EKF3 source setup (ArduPilot)

Target (real drone) vs. what is configured now in `config/params/holybro_sitl.parm`:

| Axis | Real-drone target | **Configured now (sim)** | Param |
|---|---|---|---|
| XY pos/vel | external nav (VIO) | **VIO** | `EK3_SRC1_POSXY/VELXY = 6` |
| Z (height) | external nav (VIO) | **baro** (sim VO-Z unreliable) | `EK3_SRC1_POSZ = 1` |
| Yaw | external nav (VIO) | **compass** (sim VO-yaw unreliable) | `EK3_SRC1_YAW = 1` |
| Rangefinder | Z ref for landing only | landing only, **not** EKF height | `RNGFND1_*`, `EK3_RNG_USE_HGT = -1` |
| Baro | complementary/fallback Z | fallback Z | (default) |
| GPS / optical flow | none | none | `GPS1_TYPE = 0` |

- **No GPS** (arena rules), **no optical flow**, **magnetometer unreliable indoors** —
  on the real drone yaw comes from VIO; in sim we keep compass yaw until VO yaw is
  validated in NED.
- **Rangefinder stays out of the global Z fusion** — `RNGFND1_TYPE=10` (MAVLink),
  `RNGFND1_ORIENT=25` (down), and `EK3_RNG_USE_HGT=-1`. ArduPilot still uses it for
  the land detector / flare and precision landing.

> **Planned next step (Q1 option 2):** switch Z and yaw onto VIO
> (`EK3_SRC1_POSZ=6`, `EK3_SRC1_YAW=6`) to match the real-drone target, once the VO
> Z/yaw are good enough (needs VO improvement and/or the real ZED SDK). Kept
> sim-pragmatic for now so the drone stays controllable.

---

## Simulation wiring (this pipeline)

```
ZED VIO:   zed_mimic RGB/depth ─> visual_odometry_node ─> /zed/zed_node/odom
                                        ─> vision_odom_bridge ─> VISION_POSITION_ESTIMATE ─> EKF3
Rangefinder: BiguaSim RangeFinderSensor (LaserScan, nadir)
                 ─> rangefinder_bridge ─> /mavros/rangefinder (Range)
                 ─> MAVROS distance_sensor plugin ─> DISTANCE_SENSOR ─> ArduPilot RNGFND1
                 └─> /mavros/distance_sensor/rangefinder (Range) — the same
                     reading on the topic MAVROS publishes on the real drone, so
                     the autonomy layer reads ONE name in both worlds.
ZED cloud: zed_mimic depth ─> /zed/zed_node/point_cloud/cloud_registered
                 ─> feature_map_node (accumulation only; the ZED SDK publishes
                    this natively on the drone)
```

**Files touched:**
- `config.yaml` — added a nadir `RangeFinderSensor` (`LaserCount:1`, pitch −90).
- `config/params/holybro_sitl.parm` — `RNGFND1_*` + `EK3_RNG_USE_HGT=-1`.
- `hydrone_bringup/rangefinder_bridge.py` (new) — LaserScan → Range adapter (SIM-ONLY).
- `config/mavros_distance_sensor.yaml` (new) — distance_sensor plugin, subscriber mode
  (SIM-ONLY; on real the plugin publishes from the native I²C read).
- `launch/sources_sim.launch.py` — the SIM sources layer (runs `rangefinder_bridge`,
  loads the distance_sensor config). `zed_mimic` runs with `publish_tf=False` so the
  single owner of `odom→base_link` is `visual_odometry_node`.

The sim/real launch split (sources_sim ↔ sources_real, selected by `use_sim` in
`hydrone_bringup.launch.py`) is described in the boundary map below; the autonomy
stack (`hydrone.launch.py`) is unchanged and agnostic.

### Verify (in the container, after `docker_up.sh`)
1. Sim sensor publishing: `ros2 topic echo /biguasim/uav0_id0/RangeFinderSensor` — a
   `LaserScan` with a plausible **downward** range (≈ takeoff altitude). **If the range
   looks horizontal**, Holodeck's RangeFinderSensor scanned sideways — adjust the
   `rotation` (or use a socket) in `config.yaml`; it is natively a horizontal scanner.
2. Adapter: `ros2 topic echo /mavros/rangefinder` — a `Range` (the topic the
   MAVROS distance_sensor subscriber listens on in this build). The mimic copy on
   `/mavros/distance_sensor/rangefinder` must carry the same value — that is the
   one `pad_map_node` reads, in sim and on the drone.
3. ArduPilot sees it: in QGC/MAVProxy check `RANGEFINDER` distance updates, or
   `ros2 topic echo /mavros/rangefinder/rangefinder`.
4. Confirm it is **not** driving height: EKF altitude should still track baro, and
   `EK3_RNG_USE_HGT` reads −1.

### Known verification points
- **RangeFinderSensor orientation** — natively horizontal; the nadir beam via pitch
  −90 + `LaserCount:1` needs the check in step 1 above.
- **MAVROS distance_sensor param format** — `mavros_distance_sensor.yaml` uses the
  `/**` + `ros__parameters: distance_sensor:` form; confirm the plugin loads it (topic
  from step 2 exists). The `distance_sensor` plugin lives in mavros-extras (installed).

---

## Sim ↔ real boundary

Sim and real share the whole autonomy stack; only the **sources layer** swaps
(`use_sim` in `hydrone_bringup.launch.py`). Both sources layers converge on the same
two contract buses:

```
        SIM-ONLY sources                    fronteira            AGNOSTIC (shared)
BiguaSim → ardubridge → /biguasim/* ─(zed_mimic)──►  /zed/zed_node/*  ─► autonomy
                                     ─(visual_odometry_node)─┘             (vision/nav/
ArduPilot SITL ───────────────────────(mavros@SITL)─►  /mavros/*      ─►  controller/
BiguaSim RangeFinderSensor ─(rangefinder_bridge)─► /mavros/distance_sensor/*  mission)
```

- **SIM-ONLY nodes** (replaced or absent on real): `ardubridge_node`, ArduPilot
  `SITL`, `zed_mimic_node`, `visual_odometry_node`, `rangefinder_bridge`, and the
  `mavros_distance_sensor.yaml` (subscriber mode).
- **On real:** `zed_wrapper` publishes `/zed/zed_node/*` **including `/zed/zed_node/odom`
  natively** (its own VIO — so no `visual_odometry_node`); the VL53L1X is read natively
  by ArduPilot over I²C and MAVROS **publishes** `/mavros/distance_sensor/*` (so no
  `rangefinder_bridge`).
- **AGNOSTIC (both sides):** `vision_odom_bridge` (`/zed/zed_node/odom` →
  `/mavros/vision_pose/pose`) and the autonomy stack.

### Contract-leak checklist (validate sim→real BEFORE trusting `use_sim:=false`)
Matching topic **names** is necessary but not sufficient — the data behind the name
must match too. Two known places the contract can leak silently:

1. **`RNGFND1_TYPE` — the ONLY autopilot param that differs sim vs real.** The rest of
   `holybro_sitl.parm` is shared. Sim: `RNGFND1_TYPE=10` (MAVLink; distance injected by
   `rangefinder_bridge` via MAVROS). Real: `RNGFND1_TYPE=16` (VL53L1X, read natively over
   I²C). Same output topic `/mavros/distance_sensor/*`, same orientation
   (`RNGFND1_ORIENT=25`), same `EK3_RNG_USE_HGT=-1` — only the backend type changes.

2. **Frame convention of `/zed/zed_node/odom` (ENU / REP-103).** ⚠️ Real risk.
   `vision_odom_bridge` today assumes the odom is **NWU** and rotates **+90° about Z**
   to ENU (calibrated for `visual_odometry_node`'s sim output). The real `zed_wrapper`
   publishes odom already in **ENU (REP-103)** — feeding that through the same +90°
   rotation would rotate position 90° and make the drone circle/spin, *silently*, just
   by flipping the flag. **Fix when integrating real:** make the contract ENU on both
   sides — either have `visual_odometry_node` output ENU (REP-103) and drop the +90°
   in `vision_odom_bridge` (making it a pure Odometry→PoseStamped forwarder, agnostic),
   or branch the bridge's conversion by source. (Note: the bridge's docstring currently
   says "ENU input" while the code treats it as NWU — reconcile when fixing.)

> Rule: a `/zed/zed_node/odom` that leaves in a different frame convention between sim
> and real breaks "just flip the flag" as silently as a divergent topic name would.

---

## Out of scope here (later pipelines)
- **Down-facing camera** for PrecLand (P1) / kit detection (P2) — perception pipeline;
  needs distinct sensor type/naming in the type-keyed bridge.
- **Gripper** — hardware/actuation.
- **VIO-primary Z + yaw** — the planned EKF source switch above.
