# `develop-pipelines` — GPS-denied flight on ZED visual odometry

What this branch adds **on top of `develop`**, why, and how to use it.

**Goal:** the CBR 2026 arena bans GPS, so the drone must localize from the ZED's
**visual odometry (VIO)** instead. This branch makes ArduPilot SITL fly with **GPS
disabled**, taking position from VIO via ArduPilot's **external-nav** path — takeoff,
position hold, all of it — inside the BiguaSim sim.

> In the sim, `/zed/zed_node/odom` is currently BiguaSim **ground-truth** dynamics
> (via `zed_mimic`), not a real VO algorithm. That's intentional for step 1: it
> proves the plumbing. The real drone feeds the **same topic** from the real ZED VO,
> so the autonomy code is unchanged. Next steps: inject noise/drift, then a real VO.

---

## How it works (the two channels)

Don't confuse these — they are separate:

| Channel | Direction | Role |
|---|---|---|
| **JSON FDM** (`lat/lon/alt`, vel, quat, imu) | sim → SITL | the **physics world** (ground truth). Not a sensor; ArduPilot needs it to run. Generates the simulated IMU/baro/compass (and GPS). |
| **`VISION_POSITION_ESTIMATE`** | VIO → MAVROS → SITL | the **sensor** ArduPilot *navigates* by. |

With **GPS disabled**, ArduPilot does **not** fuse the JSON-derived GPS. Its EKF
navigates on **VIO (horizontal) + baro (altitude) + IMU + compass** — all legitimate
non-GPS sensors. So the JSON `lat/lon` is **not cheating**: proof is that with GPS off
and no VIO, ArduPilot has **no** position estimate at all until VIO is fed.

```
BiguaSim DynamicsSensor ──> /biguasim/.../DynamicsSensor/Odom
   └─(zed_mimic)──> /zed/zed_node/odom ──(vision_odom_bridge)──> /mavros/vision_pose/pose
        └─(MAVROS vision_pose plugin: ENU→NED)──> VISION_POSITION_ESTIMATE ──> EKF3 external nav
```

---

## Files changed vs `develop`

### 1. `Dockerfile`
- Add `ros-humble-mavros-extras` (alongside `mavros` + `mavros-msgs`). The
  **`vision_pose` plugin lives in mavros-extras** and won't load without it.

### 2. `src/hydrone_bringup/hydrone_bringup/vision_odom_bridge.py` (new node)
The heart of the branch. It:
- Subscribes `/zed/zed_node/odom`, publishes `/mavros/vision_pose/pose`
  (**RELIABLE QoS** — the plugin subscribes reliable; a best-effort publisher is
  silently dropped, and ArduPilot receives nothing).
- **Converts the pose NWU→ENU** (+90° about Z): BiguaSim's odom is NWU, but MAVROS
  assumes ENU. Without this the position is rotated 90° and position-hold pushes
  sideways → **the drone circles and spins**. (The GPS/JSON path never had this bug
  because `bridge.py` already does NWU→NED.)
- Sends **`SET_GPS_GLOBAL_ORIGIN`** at 1 Hz for 30 s. Without a global origin,
  ArduPilot never sets **home**, and GUIDED takeoff's altitude-frame conversion
  silently fails (`NAV_TAKEOFF: FAILED`).

### 3. `src/hydrone_bringup/setup.py`
- Register the `vision_odom_bridge` executable.

### 4. `src/hydrone_bringup/launch/hydrone_sim.launch.py`
- **Base sim now runs MAVROS + `vision_odom_bridge`** (loading mavros with
  `apm_pluginlists.yaml`/`apm_config.yaml` so `vision_pose` loads). The external-nav
  params make the vehicle un-flyable without a vision feed, so this must live in the
  base bring-up — not just the mission launch — or `docker_up.sh` gives
  "Need Position Estimate" / "VisOdom: not healthy".
- `synthetic_clock` stays **True**: wall-clock ran ArduPilot's control loop on stale
  physics → uncontrollable/flipping hover.

### 5. `src/hydrone_bringup/launch/hydrone_mission_sim.launch.py`
- Simplified: it now just includes the base sim (which brings up MAVROS + bridge)
  plus the autonomy stack — no duplicate MAVROS.

### 6. `src/hydrone_bringup/config/params/holybro_sitl.parm`
GPS-denied external-nav + flight tuning:
```
GPS1_TYPE       0     # no GPS (arena rules)
VISO_TYPE       1     # MAVLink visual-odometry backend
EK3_SRC1_POSXY  6     # horizontal position from ExternalNav (VIO)
EK3_SRC1_VELXY  6     # horizontal velocity from ExternalNav
EK3_SRC1_POSZ   1     # altitude from baro (VO Z is unreliable)
EK3_SRC1_VELZ   0
EK3_SRC1_YAW    1     # yaw from compass (vision quat is ENU; compass is reliable)
FLTMODE_CH      0     # RC mode switch off so companion GUIDED sticks
SCHED_LOOP_RATE 100   # 100 Hz attitude loop (50 Hz lags -> oscillation)
```

### 7. `src/biguasim-ros2/biguasim_main/config/config.yaml`
- `ticks_per_sec: 200` (feeds the 100 Hz attitude loop; gyro = ticks ≥ 1.8·SCHED for arming).
- Cameras **on** at `640×480`, `Hz: 20`. See the tradeoff below.

---

## The performance tradeoff (cameras vs speed)

Rendering the cameras caps the sim loop; with **synthetic clock** the attitude loop
runs in *sim-time*, so flight stays stable regardless — the cost is **wall-clock speed**:

| Mode | Loop | Real-time | Attitude | Use for |
|---|---|---|---|---|
| **Cameras ON** (current) | ~75 Hz | ~0.375× (slow-mo) | stable (SCHED 100) | vision work |
| **Cameras OFF** | ~260 Hz | ~1.0× | stable (SCHED 100) | fast flight, no vision |

Switch by commenting the two camera sensors in/out in `config.yaml`.

**Camera `Hz` rule:** `Hz` must **divide `ticks_per_sec`** (200 → valid: 200,100,50,40,25,20,10,8,5,4,2,1). Higher `Hz` = smoother feed, slower sim. To speed up per-frame, lower `CaptureWidth/CaptureHeight` (that cuts render cost; `Hz` only changes how often you pay it).

---

## How to run

```bash
BS_SIM_DIR=<path-to>/bs-drone-competition ./scripts/docker_up.sh
```
Then, **wait ~20–30 s** (EKF external nav + origin/home must establish — takeoff
before that gives `NAV_TAKEOFF: FAILED`), connect QGC (UDP 14550), set **GUIDED**,
arm, and **Takeoff**. The drone lifts and holds position on VIO with GPS off.

Full mission: `ros2 launch hydrone_bringup hydrone_mission_sim.launch.py phase:=1`.

---

## Gotchas learned the hard way
- **Wait before takeoff** — origin/home + EKF need ~30 s.
- **`vision_pose` QoS must be RELIABLE**, or ArduPilot gets zero vision.
- **Odom is NWU** — must convert to ENU for MAVROS or it spins.
- **No origin → no home → takeoff silently fails.**
- **Wall-clock breaks control** — keep `synthetic_clock: True`.
- pymavlink `set_mode`/arm on SITL's TCP 5762 is unreliable (commands acked, not
  applied); **QGC is the reliable tester**. Reading telemetry via pymavlink works.

## Next steps
1. Inject VIO noise/drift → confirm ArduPilot's estimate drifts from truth (proves VIO flight).
2. Swap in a real VO algorithm on the ZED RGB/Depth images (same `/zed/zed_node/odom`).
3. Phase 1–2 sensor experiments: downward `RangeFinderSensor`, camera-position sweep.
