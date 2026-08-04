# Config as single source of truth — sim wiring (2026-08-04)

Audit of hardcoded values in the ROS 2 packages that duplicate — or silently
override — data that already lives in
[`src/biguasim-ros2/biguasim_main/config/config.yaml`](../src/biguasim-ros2/biguasim_main/config/config.yaml),
plus the changes that removed the duplication.

Scope: `hydrone_*` and `biguasim-ros2`. Vendored trees (`ardupilot/`,
`micro_ros_agent/`) were not touched.

> Related: [`SENSOR-CONFIG.md`](SENSOR-CONFIG.md) (sensor spec),
> [`RANGEFINDER-SIM.md`](RANGEFINDER-SIM.md) (rangefinder chain).

---

## The rule

**`config.yaml` is the only place a sim value is written.**
Nodes never read it — that would couple the sim-agnostic `hydrone_bringup`
nodes to biguasim's file. Instead `sources_sim.launch.py` reads the config and
passes everything down as ROS parameters. Node defaults still exist so a node
can be run bare with `ros2 run`, but they are fallbacks, not the source of
truth, and they now mirror the config instead of contradicting it.

```
config.yaml ──read──> sources_sim.launch.py ──ROS params──> zed_mimic_node
                                             └─────────────> rangefinder_bridge
            └────────read────────> ardubridge_node (it owns the file already)
```

---

## What was found

| # | Where | Was hardcoded | config.yaml says | Live? |
|---|---|---|---|---|
| 1 | `ardubridge_node.py:13-14` | `PACKAGE_NAME="Competition"`, `WORLD="CompetionMap"` | `package_name`, `world` | **yes** — config keys read by nobody |
| 2 | `zed_mimic_node.py:92` | `camera_offset_xyz = [0.12, 0.0, 0.05]` | camera `location: [0.14, 0, -0.08]` | **yes** — wrong static TF |
| 3 | `rangefinder_bridge.py:40` | `max_range = 40.0` | `LaserMaxDistance: 40` | **yes** (values agreed by luck) |
| 4 | `config.yaml:6` | `main_agent: auv0` | agent is `uav0` | **yes** — see below |
| 5 | `zed_mimic_node.py:74-78` | `/biguasim/auv0_id0/…` | agent `uav0` | no — launch overrides |
| 6 | `rangefinder_bridge.py:37` | `/biguasim/uav0_id0/…` | agent `uav0` | no — launch overrides |

**Topic names were already config-driven.** `sources_sim.launch.py` has read
`agents[0].agent_name` since the sim/real split and passes `in_rgb`, `in_depth`,
`in_odom`, `in_imu`, `in_scan` as parameters, so items 5–6 were never used at
runtime — they were stale strings (`auv0`, from an older agent name) that only
misled anyone reading the node. The values that *did* bite are 1–4, because
nothing overrode them.

---

## Changes

### 1. `package_name` / `world` now come from the config

`ardubridge_node.py` built the biguasim scenario from two module constants while
`config.yaml` carried the same two keys, unread. Editing the config did nothing.

```python
# before
PACKAGE_NAME = "Competition"
WORLD = "CompetionMap"
ardu_scenario = ArduBiguaSimRunner.build_scenario(
    self.profile, package_name=PACKAGE_NAME, world=WORLD, ...)

# after
DEFAULT_PACKAGE_NAME = "Competition"   # fallback only
DEFAULT_WORLD = "CompetionMap"
ardu_scenario = ArduBiguaSimRunner.build_scenario(
    self.profile,
    package_name=scenario_cfg.get('package_name', DEFAULT_PACKAGE_NAME),
    world=scenario_cfg.get('world', DEFAULT_WORLD), ...)
```

Switching worlds is now a config edit. Behavior is unchanged today — the
constants matched the config.

### 2. Camera mount offset from the sensor block — **this was a real bug**

`zed_mimic_node` publishes the static `base_link -> zed_camera_link` TF from
`camera_offset_xyz`, defaulted to `[0.12, 0.0, 0.05]` and never overridden. The
camera in `config.yaml` sits at `[0.14, 0, -0.08]`. The TF was therefore off by
**2 cm forward and 13 cm vertically, with the wrong sign on Z** — the sim camera
is *below* the body center, the TF claimed it was above.

No frame conversion is needed: biguasim's body frame is **GLU** (x forward,
y left, z up), identical to ROS `base_link` (FLU), so `location` carries over 1:1.

`sources_sim.launch.py` now reads the `RGBCamera` block (falling back to
`DepthCamera`) and passes `camera_offset_xyz`. Values are cast to `float` because
YAML writes the middle element as a bare `0`, and a mixed `int`/`float` list is
not a valid `double_array` parameter override.

### 3. Rangefinder range ceiling from `LaserMaxDistance`

`rangefinder_bridge` drops any return outside `[min_range, max_range]`. Its
`max_range` was a private `40.0` that happened to equal the sim sensor's
`LaserMaxDistance: 40`. Raising the sim sensor's range would have left the
bridge quietly clipping every return past 40 m. It is now passed from the config.

### 4. `main_agent: auv0` → `uav0`

Not just cosmetic. biguasim matches this key against `<agent_name>-id0` to flag
the main agent (`environments.py:448-450`); pointing at a nonexistent `auv0`
meant **no agent was ever marked as main**. Only the plain `biguasim_node` path
is affected — the ardubridge path calls `build_scenario`, which overwrites
`main_agent` with the real agent name.

### 5. Stale fallbacks aligned + documented

`auv0_id0` → `uav0_id0` in `zed_mimic_node`'s defaults and docstring;
`camera_offset_xyz`'s default set to the config value. Every fallback now carries
a comment saying the launch file is the source of truth and that the default only
applies to a standalone `ros2 run`.

---

## Files touched

| File | Change |
|---|---|
| `hydrone_bringup/launch/sources_sim.launch.py` | `_biguasim_topic_prefix(path)` split into `_biguasim_agent()` + `_biguasim_topic_prefix(agent)`; added `_sensor()`, `_camera_offset_xyz()`, `_rangefinder_max_range()`; passes `camera_offset_xyz` and `max_range` |
| `hydrone_bringup/hydrone_bringup/zed_mimic_node.py` | fallbacks `auv0_id0`→`uav0_id0`, `camera_offset_xyz` default `[0.14, 0.0, -0.08]`, docstring |
| `hydrone_bringup/hydrone_bringup/rangefinder_bridge.py` | comments marking `in_scan`/`max_range` as config-derived |
| `biguasim-ros2/biguasim_main/biguasim_main/ardubridge_node.py` | `package_name`/`world` read from config, constants demoted to defaults |
| `biguasim-ros2/biguasim_main/config/config.yaml` | `main_agent: auv0` → `uav0` + comment |

---

## Verification

The launch helpers were exercised directly against the real config (ROS stubbed
out, since ROS lives in the container):

```
prefix      /biguasim/uav0_id0
cam offset  [0.14, 0.0, -0.08]   ['float', 'float', 'float']
max range   40.0                 float
```

Not yet run in the container — the sim itself has not been launched with these
changes. What to check on the next run:

- `ros2 param get /zed_mimic camera_offset_xyz` → `[0.14, 0.0, -0.08]`
- `ros2 run tf2_ros tf2_echo base_link zed_camera_link` → same translation
- `ros2 param get /rangefinder_bridge max_range` → `40.0`
- ardubridge log line still reports `HolybroX500 | N sensores` (world unchanged)

Rename the agent in `config.yaml` (e.g. `uav0` → `drone`) and everything should
follow: bridge publishers, both bridges' subscriptions, `main_agent`. Only the
standalone-run fallbacks would go stale, and those are labelled as such.

---

## Known remaining duplication (out of scope, deliberate)

`GPS_ORIGIN = (33.810313, -118.393867)` in `ardubridge_node.py:12` is repeated as
`origin_lat`/`origin_lon` in `vision_odom_bridge.py:37-38`. It was left alone:
`config.yaml` has no key for it, and `vision_odom_bridge` is an **agnostic** node
that also runs on the real drone, so it must not depend on biguasim's config.
Unifying it means adding a `gps_origin` key and passing it as a launch parameter
on the sim path only.
