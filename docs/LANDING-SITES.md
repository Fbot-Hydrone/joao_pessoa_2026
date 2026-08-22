# Autonomous landing sites — find a pad, land on it, take off, keep going

Everything about the landing-pad behaviour: the detector, the maps, the mission,
how to run it, how to tell whether it is working, and what is deliberately not
done yet.

> **There is a second mission over the same machinery.**
> [`PHASE1-MISSION.md`](PHASE1-MISSION.md) documents `phase1_mission_node`,
> which searches by turning on the spot instead of flying forward, uses the
> forward camera as an identifier and the belly camera as a validator, and
> declares the base it takes off from instead of detecting it. It shares this
> document's detector, projection and pad map unchanged. The two are
> alternatives — each has its own launch file and only one may drive the
> vehicle at a time.

**One-line summary:** the drone takes off, flies a bounded search pattern, finds
the blue-with-yellow-ring-and-cross pad with a classic (non-learned) OpenCV
pipeline on a forward and a downward camera, lands on it, records that it landed
there, takes off again, and continues until nothing unvisited is left.

---

## 1. Run it

```bash
BS_SIM_DIR=<path-to>/bs-drone-competition ./scripts/docker_up.sh --landing-sites
```

or, inside an already-running workspace:

```bash
ros2 launch hydrone_bringup landing_sites_sim.launch.py
```

Autonomy only (sources already up):

```bash
ros2 launch hydrone_bringup landing_sites.launch.py
```

> **This replaces `hydrone.launch.py`, it does not add to it.** Both publish
> position setpoints; running them together puts two nodes on
> `/mavros/setpoint_position/local` fighting over the vehicle.

**Give it ~30 s before expecting movement.** With GPS disabled the EKF needs the
vision pose and a global origin before it accepts a takeoff (see
`DEVELOP-PIPELINES.md`). `pad_mission_node` waits for exactly that and logs what
it is waiting for; it does not need babysitting.

> **`landing_sites.launch.py` currently launches no mission node.** Its
> `pad_mission` entry is commented out at line 279, so the launch brings up the
> detectors, the maps and the TF link and nothing flies. Uncomment it to fly the
> forward run.

Useful launch arguments:

Defaults live in `landing_sites.launch.py`. `landing_sites_sim.launch.py`
deliberately does not restate them — a wrapper that re-declares an argument
overrides the inner file's default with its own, and editing the documented file
then does nothing (see [`PHASE1-MISSION.md`](PHASE1-MISSION.md) §1). Overriding
on the command line works through either file.

| argument | default | what it does |
|---|---|---|
| `cruise_alt` | `2.5` | flight altitude, m. **Must clear the arena's 1.5 m structure** — there is no forward obstacle avoidance. |
| `forward_step` | `1.0` | how far ahead the setpoint is placed, m. Each step is a position error the FCU answers with acceleration — keep it small. |
| `forward_limit_m` | `0.0` | land and finish after this much ground; 0 = forward until aborted |
| `rearm_distance_m` | `3.0` | ignore the down camera for this much ground after each takeoff, or the drone lands on the pad it just left, forever |
| `min_confidence` | `0.60` | down-camera confidence that counts as "a pad is below" |
| `auto_start` | `true` | `false` holds until `/hydrone/mission/start` is called |
| `debug_images` | `true` | annotated detector views |
| `feature_map` | `true` | the world/coverage mapper over the ZED's cloud |
| `map_odom_tf` | `true` | publish the measured `map → odom` that joins TF's two trees |
| `range_topic` | `/mavros/distance_sensor/rangefinder` | the same in sim and on the drone — see §7 |
| `odom_source` | `vo` | what the EKF navigates on. `ground_truth` swaps in BiguaSim dynamics as a debugging aid — read §10 first. |

Watching it:

```bash
ros2 topic echo /hydrone/mission/status          # state machine, 1 Hz
ros2 topic echo /hydrone/pads/map                # the pad map
ros2 run rqt_image_view rqt_image_view /hydrone/pads/down/debug_image
ros2 run rqt_image_view rqt_image_view /hydrone/pads/forward/debug_image
```

In RViz (fixed frame `map`): `/hydrone/pads/markers` (grey = candidate, cyan =
confirmed, **green = landed on**, orange = the takeoff base — see §5),
`/hydrone/map/cloud`, `/hydrone/map/coverage`.
The camera's own per-frame cloud is `/zed/zed_node/point_cloud/cloud_registered`.

Stop it early: `ros2 service call /hydrone/mission/abort std_srvs/srv/Trigger`
— it lands where it is.

---

## 2. The shape of the thing

```
  BiguaSim / real hardware
        │
        │  /zed/zed_node/rgb + depth        /down_cam/image_raw
        ▼                                          ▼
   pad_detector_node (forward)            pad_detector_node (down)
        │        pixel -> world, via depth or the ground plane
        └──────────────┬───────────────────────────┘
                       ▼   /hydrone/pads/detections
                  pad_map_node          ← rangefinder, for pad heights
                       │   /hydrone/pads/map
                       ▼
                 pad_mission_node  ──► /mavros/setpoint_position/local
                                   ──► set_mode / arming / takeoff

   (in parallel, observing only)
   /zed/zed_node/point_cloud/cloud_registered
             ──► feature_map_node ──► /hydrone/map/cloud + /coverage
```

| node | package | role |
|---|---|---|
| `pad_detector_node` | `hydrone_vision` | one per camera: detect pads, place them in the world |
| `pad_map_node` | `hydrone_nav` | fuse detections into a persistent map; track `visited` |
| `feature_map_node` | `hydrone_nav` | accumulates the ZED's point cloud into a world map + observation-coverage grid — see [`ZED-FEATURE-MAP.md`](ZED-FEATURE-MAP.md) |
| `pad_mission_node` | `hydrone_mission` | the flight: search, land, take off, repeat |
| `down_cam_mimic_node` | `hydrone_bringup` | **sim only**: BiguaSim's belly camera → `/down_cam/*` + TF |

Messages (`hydrone_msgs`): `PadDetection` (one per-frame observation), `Pad` /
`PadMap` (the fused map), `MarkPadVisited` (the "I landed here" service).

---

## 3. The detector

`hydrone_vision/pad_detector.py` — pure OpenCV/numpy, no ROS, no model files,
deterministic cost per frame. The ROS wrapper is a separate file so the
algorithm can be tested against images directly.

The pad is a saturated blue field carrying a yellow ring with a yellow cross
through its centre. **Finding blue is not enough** — a tarp, a shirt, a painted
line, a puddle reflecting sky all pass that. The structure is the identity of the
pad, so most of the work goes into proving a blue blob really carries a
concentric ring-and-cross:

| # | check | rejects |
|---|---|---|
| 1 | HSV blue/yellow masks with a high **saturation floor** | washed-out look-alikes |
| 2 | area, **solidity**, aspect ratio | slivers, painted lines, bridged blobs |
| 3 | yellow fraction inside the blue footprint | plain blue objects; yellow ones with a blue rim |
| 4 | **concentricity** of the yellow and blue centroids | blue with an off-centre yellow smear |
| 5 | **ring coverage**: rays out of the centre hit yellow in *every* direction | one-sided smears |
| 6 | **cross arms**: halfway out along each ray, yellow appears in exactly four angular lobes | a solid yellow disc (yellow at every angle), a bare ring (yellow at none) |

Checks 5 and 6 come from one polar sweep of 72 rays. Each ray is measured against
**the ring radius found along that ray**, never a global circle — which is what
makes it hold when perspective squashes the pad into an ellipse, and what makes
it independent of the drone's heading.

### Confidence, and the 0.75 line

Checks 1–4 are hard gates. Checks 5–6 also gate, **but only once the pad is big
enough on screen** (`r95 ≥ structure_radius_px`, 22 px) for a missing ring or a
missing arm to actually be visible; below that, morphology has already fused ring
and arms into one blob and the measurement means nothing.

That produces a useful property, and the mission depends on it:

> The cross check carries weight 0.25 and is forced to zero when the structure
> cannot be resolved. The remaining weights sum to 0.75, so **confidence > 0.75
> implies the ring and the cross were actually verified.**

Which is what lets a mission treat a distant sighting as a *lead* worth flying
to and never as something to land on sight. `pad_mission_node` no longer uses
it (it only ever reads the belly camera, from directly above); it is
`phase1_mission_node` that depends on the property, in the split between the
forward camera identifying and the belly camera validating — see
[`PHASE1-MISSION.md`](PHASE1-MISSION.md) §5.
`test_pad_detector.py::test_high_confidence_implies_the_structure_was_verified`
pins this.

### Measured behaviour

From `src/hydrone_vision/test/test_pad_detector.py` (41 tests, synthetic renders):

- **Detected** from a pad filling the frame down to ~**18 px across** — at
  640×480 / 90° that is a 1 m pad about 25 m away.
- **Rotation invariant** (0–138° checked), and correct under oblique
  (trapezoidal) views.
- Survives Gaussian noise to σ=20, exposure ×0.55–×1.25, and motion blur.
- **Rejects**: bare ground, a plain blue rectangle, blue with an off-centre
  yellow blob, blue with a *solid* yellow disc, blue with a ring but no cross,
  yellow alone, a thin blue line, and a desaturated print of the real pad.

### One documented miss

If the image border cuts **through the ring**, the ring stops enclosing
anything, the blue inside and outside merge, and no candidate carries a
concentric ring — the frame is missed. A pad merely clipped at the edge, ring
intact, is still found.

This is safe (a missed frame, never a false landing site) and it clears itself as
the pad moves inward, which is what happens while flying toward it. Pinned by
`test_pad_clipped_through_the_ring_is_a_known_miss` so it stays a known
behaviour rather than a surprise.

### Retuning for the real arena

Every threshold is a ROS parameter on `pad_detector_node`. The ones that will
actually need touching under real light are the HSV bands:

```
blue_hsv_low/high    default [95,110,50] .. [135,255,255]
yellow_hsv_low/high  default [18,110,90] .. [38,255,255]
```

Point the drone at the real pad, watch `/hydrone/pads/<camera>/debug_image`, and
widen the hue or lower the saturation floor until the pad is solidly masked. The
saturation floor is what keeps pale look-alikes out — lower it as little as you
can get away with.

**The sim already needed exactly this.** Measured on BiguaSim's rendering
(2026-08-18): UE's tonemapping/bloom pushes the whole pad toward white, and the
saturation floor of 110 admitted **zero** pixels of either colour, so no pad was
ever detected from any altitude. Measured inside the pad's bounding box on a
lossless `/down_cam` frame at a 3 m hover:

```
blue field         S 37-75, mean 56
yellow ring+cross  S 38-59, mean 48
```

An earlier grab over the spawn pad put the yellow at S ≈ 82–109, so saturation
varies a lot with altitude and local lighting inside the map. **Both** floors
have to come down, not just yellow: `detect()` runs `findContours` on the BLUE
mask and iterates over blue contours, so an empty blue mask means zero
candidates and the yellow, ring, cross and concentricity checks never run at
all.

`landing_sites.launch.py` and `phase1.launch.py` therefore both pass

```
blue_hsv_low:   [95, 30, 50]
yellow_hsv_low: [18, 30, 90]
```

S ≥ 30 covers every pixel of both grabs with margin and leaves the
discrimination to the structural checks, which is where it belongs — on a
confirmed detection ring coverage is 1.0, arms 4, concentricity offset 0.005.
The library default (and its tests) are unchanged; re-do this tuning against the
real arena's light.

---

## 4. Pixel → world

Two routes, preferred in this order:

- **Depth** — back-project with the intrinsics through the registered depth
  image. Metric, assumption-free. The forward ZED normally uses this.
- **Ground plane** — intersect the pixel's ray with a horizontal plane at
  `ground_z`. The arena floor is flat and the belly camera points nearly
  straight at it, so this is accurate to centimetres and needs no depth sensor.

A ray at or above the horizon is refused outright rather than extrapolated into
a huge number that would poison the map.

**The world frame is the frame of `/mavros/local_position/pose`** (the FCU's
local ENU, normally `map`), not the VO `odom` frame — even though the two are
near-identical here. The mission's setpoints go to
`/mavros/setpoint_position/local`, which is interpreted in exactly that frame, so
composing the camera pose from the same source the controller uses removes a
whole class of "the map and the controller disagree" error. TF is consulted only
for the constant `base_link → <optical frame>` mount.

`src/hydrone_vision/test/test_pad_projection.py` works this chain against poses
whose answer can be done by hand: centre pixel straight below, right-of-image →
right-of-drone, up-of-image → ahead-of-drone, yaw following the airframe, depth
back-projection, the ground-plane fallback, and every refusal case.

---

## 5. The pad map

`pad_map_node` turns the detection stream into a handful of persistent entries.

- **Association**: nearest neighbour inside `merge_radius` (1.2 m) — larger than
  the projection error, smaller than the spacing between the arena's bases.
- **Fusion**: weighted running mean, weight `confidence / max(range, 1)`. A
  close look from the belly camera outvotes a speck 20 m ahead, because
  projection error grows with range on both routes.
- **Confirmation**: `min_observations` (3) sightings before the mission will
  fly to it. One sighting is noise.
- **Pruning**: unconfirmed entries not re-seen within `provisional_ttl_s` (20 s)
  are dropped. Confirmed and visited entries are never pruned.
- **Rejections are logged.** A detection that does not pass `position_valid`,
  `min_confidence`, `max_range_m` (30 m) or the `min/max_pad_height` band is
  warned about, naming the gate and the offending value, throttled per
  (camera, gate) at `reject_log_period_s` (5 s; 0 logs every one). Without it
  the detector draws a confident box while the map stays empty and nothing says
  why — see [`PHASE1-MISSION.md`](PHASE1-MISSION.md) §12 for the gate table and
  what usually closes at range.
- **`visited`**: set through the `MarkPadVisited` service after touchdown. This
  is the flag that turns "land" into "land, then keep going" — without it the
  drone re-detects the pad it is standing on and lands on it forever.
- **`require_armed`** (default true): nothing is mapped at all until
  `/mavros/state` first reports armed. On the ground both cameras are looking at
  the base the drone is standing on, from the grazing angles that detect it
  best, through a pose the EKF has not settled. The gate latches — the vehicle
  disarms on every pad it lands on and detections must keep flowing then.
- **`is_takeoff_base`**: set through the `RegisterTakeoffBase` service at arm
  time, with the position the drone is standing at. That entry is never pruned,
  never offered as a landing candidate, has its height recorded as measured (the
  drone is standing on it), is exempt from the rangefinder refinement below, and
  claims detections within `takeoff_base_radius` (1.5 m, wider than
  `merge_radius`) so a glancing sighting from the air cannot spawn a phantom pad
  beside it. Association compares `distance / claim_radius`, so the wider claim
  cannot steal a detection that sits closer to a genuine pad. It draws **orange**
  in RViz.

  Both of these exist for `phase1_mission_node`; the full argument is in
  [`PHASE1-MISSION.md`](PHASE1-MISSION.md) §4. `pad_mission_node` is unaffected
  by either — it never reads `is_takeoff_base`, and it arms before it needs the
  map.

### How the elevated base gets its height

Detections project onto the assumed floor, so every pad starts at z≈0. The arena's
second base is ~0.5 m up, and landing on it needs the real number.

While the drone hovers within `overhead_radius` (0.5 m) of a mapped pad, the
downward rangefinder is measuring **that pad's top surface**, so
`height = drone_z − range` and the entry is corrected in place. `height_measured`
flips true and the mission descends to `height + land_trigger_agl` instead of
guessing.

Once a pad is `visited` its height is frozen: standing on it measured the surface
exactly, and a later glancing pass over the floor beside it must not undo that.
(`test_touchdown_height_is_not_overwritten_by_later_flyovers`.)

---

## 6. The mission

`pad_mission_node` is deliberately the smallest thing that flies a full cycle:

```
WAIT_FCU -> ARMING -> TAKEOFF -> FORWARD -> LAND -> DWELL -+-> DONE
              ^                                            |
              +--------------------------------------------+
```

Take off, walk the setpoint along world **+X** in `forward_step` (1 m) pieces,
land the moment the belly camera reports a pad, mark it visited, take off again,
carry on. There is no search pattern, no forward-camera lead, no align or
descend phase and no return-home leg — that was the point. Get one full
takeoff/detect/land/takeoff cycle working in the sim, then add pieces back.
Anything more is another thing that can break while you are trying to find out
why the drone will not fly.

> An earlier draft of this document described a much larger mission —
> `SEARCH`/`INSPECT`/`ALIGN`/`DESCEND`/`RETURN_HOME`, an expanding square
> spiral, `align_alt`, `search_radius`, `commit_confidence`, `max_pads`. None of
> that is in the node. It was cut back to the skeleton above before the first
> flight attempt and the document was not updated with it. Corrected
> 2026-08-21. The **two-stage** idea it described — a distant camera identifies,
> a close one validates — was worth keeping and now lives in
> `phase1_mission_node`; see [`PHASE1-MISSION.md`](PHASE1-MISSION.md).

Driven by a 10 Hz tick. **Every service call is asynchronous with its own
deadline** — a blocking call inside a timer callback would stall the setpoint
stream and hand the vehicle to the FCU's failsafe.

**Commands are re-sent on a timer, not on the previous call's result.** MAVROS
acking a mode change is not the same as ArduPilot accepting it (EKF not ready,
pre-arm check pending), so `/mavros/state` is the only thing treated as truth.
`ARMING` checks GUIDED *before* armed, which matters on the relaunch after a
landing: ArduCopter auto-disarms only after `DISARM_DELAY` (10 s) and the dwell
is shorter, so the vehicle is usually still armed — and still in `LAND`. Taking
"armed" as done would send a takeoff while in LAND, which ArduPilot refuses,
forever.

### Why the setpoint is stepped

The setpoint is placed one `forward_step` ahead and only advanced once the
vehicle has actually arrived, so the position error the FCU sees never exceeds
one step. That error is what the position controller turns into acceleration,
and an aggressive demand under BiguaSim's ~0.3–0.5 s actuation lag is what flips
the vehicle. `phase1_mission_node` takes the other route — one setpoint per leg,
with the speed capped at the FCU — for the reasons in
[`PHASE1-MISSION.md`](PHASE1-MISSION.md) §8.

### Why the drone does not land on the pad it just left

After a takeoff, the pad it took off from is still directly below and still
being detected. `rearm_distance_m` (3 m) ignores the belly camera until that
much ground has been covered on the current leg; without it the mission lands on
the same pad forever. The `visited` flag in the map is the durable half of the
same idea.

### Landing and taking off again

At a detection the setpoint stream **stops** and LAND is handed to the FCU,
whose rangefinder flare does the touchdown — a position setpoint arriving
mid-landing at best is ignored and at worst fights it. Touchdown is detected by
either the vehicle disarming or its altitude settling low, held for
`land_settle_s`. Then: mark visited (recording the resting altitude as the pad's
height), dwell, and go back to `ARMING` — which re-arms and re-takes-off,
because ArduCopter will not climb from a bare position setpoint in GUIDED.

### Yaw

Setpoints carry `orientation.w = 1.0`, i.e. yaw 0, which commands the nose along
world +X — the direction of travel, so the two agree. Note this is an absolute
heading, not "hold what you have": a vehicle that booted facing elsewhere will
turn to face map-east on the first setpoint. `phase1_mission_node` seeds its yaw
from the heading the vehicle climbed with instead.

---

## 7. Sim vs real

The autonomy layer consumes only the agnostic contract buses and is identical in
both. What changes is underneath it:

| | sim | real |
|---|---|---|
| ZED (images, depth, **point cloud**, odom) | `zed_mimic_node` + `visual_odometry_node` | `zed_wrapper` (ZED SDK) |
| belly camera | `down_cam_mimic_node` | a USB/CSI camera driver + a static TF |
| rangefinder | `rangefinder_bridge` feeds MAVROS on `/mavros/rangefinder` **and mimics** MAVROS's `/mavros/distance_sensor/rangefinder` | MAVROS **publishes** `/mavros/distance_sensor/rangefinder` |

**Nothing above the sources is told which world it is in.** `landing_sites_sim`
passes the autonomy layer no topic overrides at all — every topic it reads is the
one the real hardware publishes, and the sim's job is to publish those topics.

That used to be untrue in one place: the rangefinder. In sim the Range has to
reach MAVROS on `/mavros/rangefinder` (that is where this MAVROS build's
`distance_sensor` plugin *subscribes*), while on the drone MAVROS *publishes* the
natively-read VL53L1X on `/mavros/distance_sensor/rangefinder` — so the autonomy
had to be launched with `range_topic:=...` on real hardware, and a forgotten flag
meant pad heights were never measured, `height_measured` stayed false, and the
drone would try to land on the elevated base as if it were on the floor.
`rangefinder_bridge` now publishes both: the first topic is plumbing INTO the
FCU, the second mimics MAVROS's real output. The autonomy reads the second, in
both worlds, with no argument.

On the drone, two `zed_wrapper` settings matter to this stack: the point cloud
must be ON (`feature_map_node` has no other input), and
`pos_tracking.publish_map_tf` must be **false** — the wrapper broadcasts
`map → odom` by default and `map_odom_node` owns that edge here. See
`sources_real.launch.py`.

`sources_real.launch.py` carries the belly-camera driver and its static TF as a
commented, ready-to-fill stub. Its rotation is checked against the simulated
mount by `test_real_hardware_static_tf_matches_the_simulated_mount`, so the two
cannot silently diverge — the worst possible split being one that works in sim
and lands in the wrong place on the drone.

**Calibrate the real belly camera.** The projection is only as good as
`fx/fy/cx/cy`; do not ship the nominal FOV.

---

## 8. The simulated belly camera

Added to `src/biguasim-ros2/biguasim_main/config/config.yaml`:

```yaml
- sensor_type: RGBCamera
  sensor_name: DownCamera        # what lets a 2nd RGBCamera coexist with the ZED
  ros_publish: true
  Hz: 10
  location: [0.0, 0, -0.1]
  rotation: [0, 90, 0]           # [roll, pitch, yaw] deg — +90 pitch = lens DOWN (verified in UE)
  configuration: {CaptureWidth: 320, CaptureHeight: 240, FOV: 90}
```

`sensor_name` is the mechanism: BiguaSim keys sensors by name (defaulting to the
type) and the ROS bridge publishes under that name, so this one lands on
`/biguasim/<agent>/DownCamera` instead of colliding with the ZED's `RGBCamera`.

`sources_sim.launch.py` reads `location` **and `rotation`** straight out of this
block and hands both to `down_cam_mimic_node`, which builds the ROS mount TF from
them. Same numbers aim the simulated camera and build the transform, so the two
cannot drift apart — and `_camera_offset_xyz` is pinned to `sensor_name:
RGBCamera` so adding this camera cannot hand the ZED the wrong mount.

### The rotation sign — checked, and it was the other one

BiguaSim's own docs describe pitch inconsistently ("rotation around the fixed
**right** (y) axis", in a frame where y is **left**). **Checked by eye against a
running UE5 render 2026-08-18: the literal reading wins.** Pitch is rotation
about the RIGHT axis with the right-hand rule, so `rotation: [0, 90, 0]` aims
the lens at the ground — the original "+pitch = nose up" guess was backwards.
That reading also matches ROS RPY 1:1, so `down_cam_mimic_node` carries the
value into the mount TF with **no sign flip** (the original flip pointed the
TF at the sky while the image showed the ground, and every down-camera
detection died at projection time as "ray at or above the horizon").

### Cost — and why it is not optional

A third camera render, and the render budget turned out to be a FLIGHT-SAFETY
parameter, not a frame-rate preference. With the ZED pair at 640×480@20 plus
this camera at 640×480@10, the sim loop fell far enough behind that the FDM
picked up ~0.3–0.5 s of actuation lag — visible in the dataflash `RATE` stream
as the actual rate trailing the demanded rate — and the attitude controller
oscillated to a flip within seconds of the first real maneuver
("Crash: Disarming: AngErr=170>30", reproduced on ground-truth odometry, so it
was not a localization problem). 2×640×480@20 was the proven-stable budget;
three renders now fit inside it: the ZED pair runs at **10 Hz** and this camera
at **320×240@10** (a 1 m pad at 2.5 m altitude still spans ~64 px there, far
above the detector's ~18 px floor). If the sim still runs hot on a weaker
machine: drop `Hz` to 5, or comment the sensor out entirely — the launch only
starts `down_cam_mimic_node` when the sensor is actually declared. Note that
`Hz` must divide `ticks_per_sec` (200).

---

## 9. A bug this work turned up in the container

**ROS Humble's `cv_bridge` segfaults in this project's image.**

`Dockerfile` installed `numpy` unpinned, which resolves to 2.x. `cv_bridge` ships
a compiled boost extension linked against numpy 1.x. The mismatch does **not**
fail at install time — `import cv_bridge` prints

```
AttributeError: _ARRAY_API not found
```

and carries on. The first `imgmsg_to_cv2()` call then **segfaults the process**.

That takes down `visual_odometry_node`, which with GPS disabled is what feeds the
EKF its position — so the vehicle loses its position estimate the moment the
first image arrives. `vision_node` is affected the same way.

Two changes:

1. `Dockerfile` now pins `numpy<2`. **Rebuild the image** for this to take
   effect. Verify with:
   ```bash
   docker run --rm <image> bash -c '. /opt/ros/humble/setup.sh && \
       python3 -c "from cv_bridge import CvBridge; CvBridge(); print(\"ok\")"'
   ```
2. The landing-pad nodes do not use `cv_bridge` at all —
   `hydrone_vision/image_convert.py` does the conversion in numpy.

   **That file also carries a performance trap worth knowing about.** It used to
   assign `msg.data = <bytes>`, and rclpy converts a `bytes` object into a
   `uint8[]` field element by element in Python — 361 ms for one 672x376x3 frame
   on the Jetson, against 0.2 ms for `array.array("B", ...)`. Since this
   function publishes every simulated image too, the simulator was paying it on
   every frame. See [`JETSON-REAL-STACK.md`](JETSON-REAL-STACK.md) §7. An Image
   message is a header, a byte buffer and an encoding string; reshaping that
   needs no C extension, and the one it was using could take a node down
   mid-flight. (It also handles row padding and `16UC1` millimetre depth, which
   the real belly camera may well produce.)

---

## 10. Localization: the mission does not fly yet, and why

**Measured 2026-08-17, first closed-loop run.** With GPS disabled, whatever is on
`/zed/zed_node/odom` is relayed as `VISION_POSITION_ESTIMATE` and *is* the EKF's
position. By default that is `visual_odometry_node`, the branch's ORB/PnP VO.

What the first flight attempt actually did:

```
   0 – 350 s   on the ground, never moved   VO error 0.04 -> 0.70 m,  attitude 0.2 -> 20 deg
    ~358 s     "Arming motors"
    ~363 s     error 1.8 -> 4.8 -> 9.4 -> 26.6 m,  attitude -> 175 deg
               AP: Crash: Disarming: AngErr=165>30, Accel=1.0<3.0
```

The decisive number is in the first line. Ground truth read **0.000 for all 121
samples** — the drone had not moved a centimetre — and the VO had still walked
0.39 m and 8.6° away within 78 s. That is not flight drift; a stationary camera
should integrate to nothing. The cause is the scene: `visual_odometry_node`
logged **302 starved frames** with 4–11 feature matches against its 12 minimum,
because on the CompetionMap superflat the forward camera is staring at bare,
featureless ground. PnP has nothing to solve, so the pose random-walks.

Then the vehicle armed, GUIDED began correcting a position error that did not
exist, and flew 26 m before flipping. The 166° attitude "error" logged at the
end is the airframe lying upside down — a *consequence*, not the cause.

**This is an open defect, and the default deliberately does not hide it.**
`odom_source` defaults to `vo`: the drone will fly on the ZED SDK's VIO, so the
simulator flies on an estimator too. A simulator that flies on truth has stopped
testing the thing that has to work.

### Partially fixed 2026-08-18: the catastrophic jumps are gated out

The 2026-08-18 run showed the failure was worse than a random walk: ONE
degenerate PnP solution moved the pose **11 m / 95° in a single frame** while
the drone sat still, and with GPS off that jump became the EKF position on the
spot. `visual_odometry_node` now carries the standard gates it lacked: a
minimum inlier **ratio** (`min_inlier_ratio`, 0.5 — 12 inliers out of 80
matches is RANSAC telling a story, not a solution) and a physical plausibility
bound on one frame-to-frame step (`max_step_m` 0.5 / `max_step_deg` 20 — at
20 Hz nothing real moves further than that in one frame). Rejected steps hold
pose, exactly like a starved frame.

What the gates do NOT fix: on this featureless map the forward camera still
starves (4–11 matches against a 12 minimum), so the VO holds pose while the
drone actually moves — safe, but blind. Flying the mission on `vo` needs
either a feature-rich map or the IMU-fusion step below. Note also that on the
ground the forward ZED is looking at the horizon and sky, which is the worst
possible input; the view improves once airborne, so there is a chicken-and-egg
worth being aware of.

Worth remembering: **this node is SIM-ONLY**. The real drone runs `zed_wrapper`
and the ZED SDK's own stereo-inertial VIO, a far better estimator. This is a
stand-in for it, because the SDK cannot run against BiguaSim frames.

### `ground_truth` is a debugging tool, not a pass

```bash
ros2 launch hydrone_bringup landing_sites_sim.launch.py odom_source:=ground_truth
```

BiguaSim dynamics fly the vehicle. Use it to tell an autonomy bug apart from a
localization one: if a behaviour fails under `vo` and passes under
`ground_truth`, the fault is in the estimate, not the behaviour. **A run on
ground truth never demonstrates that something works** — the real drone has no
ground truth.

Only the exact string `ground_truth` selects it. Every other value, typos
included, falls through to `vo`, so nobody can accidentally acquire perfect
localization and a false sense of progress. `test_odom_source.py` pins that,
along with the single-owner and single-TF-broadcaster invariants.

In both modes the estimator that is *not* flying keeps running and keeps being
measured against the other by `odom_error_node`.

Useful while looking at this:

```bash
ros2 topic echo /zed/zed_node/pose_GT --field pose.orientation   # truth, flat
ros2 topic echo /zed/zed_node/odom_VO --field pose.pose          # what VO thinks
```

`pose_GT` exists because the ground-truth quaternion was previously only
reachable inside an `Odometry` message, which is awkward to echo, plot or point
an RViz Axes display at — and "is the estimate wrong, or did the airframe
actually flip?" is the question that run turned on.

---

## 11. Tests

148 tests across `hydrone_vision`, `hydrone_nav` and `hydrone_mission`, plus 18
in `hydrone_bringup`. None need UE5:

```bash
docker run --rm -v $PWD:/repo -w /repo <image> bash -c \
  '. /ws/install/setup.sh && python3 -m pytest \
   src/hydrone_vision/test src/hydrone_nav/test src/hydrone_mission/test -q'
```

| file | what it covers |
|---|---|
| `hydrone_vision/test/test_pad_detector.py` (41) | the detector against synthetic renders: size sweep, rotation, oblique views, noise/exposure/blur, and every negative in §3 |
| `hydrone_vision/test/test_pad_projection.py` (17) | the frame algebra, by hand-checkable poses; and the real-vs-sim mount TF |
| `hydrone_nav/test/test_pad_pipeline.py` (6) | the real nodes wired together — topic names, QoS compatibility, TF lookup, fused map position, `visited`, elevated-pad height. Its `FakeSim` publishes `/mavros/state` armed, because with `require_armed` a stack that never arms maps nothing |
| `hydrone_nav/test/test_takeoff_base.py` (24) | the pre-arm gate and its latch; registering the takeoff base, claiming an existing entry, the wider claim radius; never pruned; height not rewritten by a fly-over; the flag and the marker colour; every rejection gate naming itself in the log |
| `hydrone_bringup/test/test_odom_source.py` (14) | which estimator flies the vehicle: single owner of the flight topic, single TF broadcaster, fail-safe on a bad value |
| `hydrone_mission/test/test_pad_mission.py` (19) | the forward run: the setpoint never far ahead of the vehicle, stale and unconfident detections refused, the just-left pad not re-landed on |
| `hydrone_mission/test/test_phase1_mission.py` (41) | the Phase 1 mission — see [`PHASE1-MISSION.md`](PHASE1-MISSION.md) §11 |
| `hydrone_bringup/test/test_launch_arguments.py` (4) | neither `*_sim.launch.py` wrapper re-declares or forwards an argument its autonomy layer declares. A wrapper that does silently overrides the inner file's defaults, so editing them has no effect — found 2026-08-22 |

The flight states (arming, takeoff, landing) are **not** unit-tested — they are
conversations with ArduPilot, and mocking one proves nothing about the real
vehicle. They are exercised by flying the sim.

---

## 12. Not done yet

Ordered by how much they matter.

1. **Localization blocks everything — see §10.** The visual odometry drifts
   while the drone is stationary, so the vehicle cannot hold position long
   enough to fly the mission. Until that is fixed, `landing_sites_sim.launch.py`
   will arm and then fly away. This is the next thing to work on, and it is not
   in the landing-site code.
2. **The landing behaviour itself still has not been observed.** The first
   closed-loop run (2026-08-17) never got airborne. Detection, mapping and the
   state machines are tested; find → land → take off → continue has not yet been
   watched end to end, by either mission. Expect tuning of tolerances, timeouts
   and the HSV bands, not structural change.
3. **No forward obstacle avoidance.** `cruise_alt` (2.5 m) simply flies over the
   arena's 1.5 m structure. The ZED depth is right there and a "stop if the
   centre of the frame is closer than X" guard is small; it was left out to keep
   the first flight's failure modes few, since a badly-tuned guard stops the
   drone on the pad itself.
4. **The coverage grid is not fed back into either search.** `feature_map_node`
   answers "where have I looked?", which is what a search needs before it can
   claim there is nothing left — but both missions decide on their own and only
   publish the grid for the operator. `phase1_mission_node` is the one that
   would benefit: it could stop turning once the circle is covered rather than
   counting to eight.
5. **Yaw is not controlled *by this mission*.** `pad_mission_node`'s setpoints
   hold an absolute yaw of 0. `phase1_mission_node` does command yaw, and turning
   in place is its entire search — see [`PHASE1-MISSION.md`](PHASE1-MISSION.md).
6. **One pad geometry.** The check thresholds assume the ring sits well inside
   the field and the cross spans the ring. A pad with very different proportions
   would need `structure_radius_px` and the yellow-fraction band retuned — the
   checks themselves are proportional and would still apply.
