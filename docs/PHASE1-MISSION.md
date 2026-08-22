# Phase 1 — take off, turn until you see a base, land on it, come home

The Phase 1 flight for the 5×5 m arena. Everything about it: what it does, why
it does it that way, what changed elsewhere in the stack to make it possible,
how to run it, and what has and has not been observed.

**One-line summary:** the drone arms, declares the base it is standing on as
off-limits, climbs to 1 m, turns on the spot in 45° steps until a landing base
appears in the map, flies over it, confirms it on the belly camera, lands, marks
it visited, and repeats until `target_bases` are done — then returns to the base
it started on and lands there.

**`target_bases` defaults to 1, not the competition's 2.** Nothing here has been
flown; the first question is whether one complete
find → confirm → land → take off → return cycle closes at all. A second base
only adds a leg on an estimate that has already been through a landing and a
takeoff, and it doubles the time before the run tells you anything. Raise it
with `target_bases:=2` once one cycle has been watched end to end.

This is an **alternative** to [`LANDING-SITES.md`](LANDING-SITES.md)'s
`pad_mission_node`, not a replacement for the machinery under it. The detector,
the projection and the pad map are the same code, tuned the same way. What
differs is the flight.

---

## 1. Run it

```bash
scripts/docker_up.sh --phase1                         # sim, from a cold host
ros2 launch hydrone_bringup phase1_sim.launch.py      # sim, inside the workspace
ros2 launch hydrone_bringup phase1.launch.py          # autonomy only
```

`docker_up.sh` forwards any `name:=value` argument straight through to the
launch, so the table below is tunable without editing a file:

```bash
scripts/docker_up.sh --phase1 target_bases:=2 takeoff_alt:=1.5
scripts/docker_up.sh --phase1 --dev confirm_timeout_s:=60.0
```

### Where the defaults live, and how to be sure they took

**`phase1.launch.py` is the only file that declares a default.**
`phase1_sim.launch.py` deliberately does not restate them: an argument declared
in both places means the wrapper's value silently overwrites the inner file's,
which is exactly the bug that ate the 2026-08-22 session — `takeoff_alt` edited
to 4 m, vehicle climbed to 1 m, nothing warned. Launch configurations are
inherited by an included description, so saying nothing is both sufficient and
correct: the inner default applies when no argument is given, and a command-line
override still reaches it. `test_launch_arguments.py` fails if the duplication
comes back.

**Check the banner, every run.** `phase1_mission_node` prints its effective
configuration on the first line it logs:

```
phase1_mission ready — takeoff to 4.0 m, search by 45 deg turns (max 20),
land on 9 base(s), then home. Auto-starting.
```

If those numbers are not the ones you set, the launch file being read is not the
one you edited — the two causes look identical from the cockpit. Either the
shadowing above (fixed), or a **stale installed copy**: `phase1.launch.py` and
`phase1_sim.launch.py` did not exist when the container image was built, and new
launch files are not covered by the dev bind mount's symlinks. Run
`scripts/dev_rebuild.sh --restart`, or rebuild the image, and read the banner
again.

> **This replaces `hydrone.launch.py` and `landing_sites.launch.py`, it does not
> add to them.** All three carry a mission node, and two nodes publishing to
> `/mavros/setpoint_position/local` fight over the vehicle.

**Give it ~30 s before expecting movement.** With GPS disabled the EKF needs the
vision pose and a global origin before it accepts a takeoff (see
[`DEVELOP-PIPELINES.md`](DEVELOP-PIPELINES.md)). `phase1_mission_node` waits for
exactly that and logs what it is waiting for.

| argument | default | what it does |
|---|---|---|
| `takeoff_alt` | `1.0` | altitude for **everything** — takeoff, turning, travelling, confirming. See §6. |
| `target_bases` | `1` | landing sites to visit before going home. The takeoff base is not one of them. **1 until a full cycle has been flown; the competition number is 2.** |
| `rotation_step_deg` | `45.0` | size of each search turn, clockwise |
| `max_rotations` | `8` | turns before the fallback fires. 8 × 45° is one full circle. |
| `settle_s` | `2.0` | time held stationary after each turn before the map is believed |
| `confirm_detections` | `3` | belly-camera looks needed before committing to a landing |
| `confirm_confidence` | `0.60` | confidence that counts as a look |
| `confirm_timeout_s` | `25.0` | hover budget over a candidate before blacklisting it |
| `require_armed` | `true` | `pad_map` maps nothing until the vehicle first arms |
| `ground_z` | `0.0` | height of the arena **floor** above the takeoff plane — see §7 |
| `auto_start` | `true` | `false` holds until `/hydrone/mission/start` is called |
| `debug_images`, `feature_map`, `map_odom_tf` | | as in `landing_sites.launch.py` |
| `odom_source` | `vo` | sim only. `ground_truth` is a debugging aid, never a pass — LANDING-SITES §10. |

Watching it:

```bash
ros2 topic echo /hydrone/mission/status          # state, turns, target, blacklist
ros2 topic echo /hydrone/pads/map                # the pad map
ros2 run rqt_image_view rqt_image_view /hydrone/pads/down/debug_image
ros2 run rqt_image_view rqt_image_view /hydrone/pads/forward/debug_image
```

In RViz (fixed frame `map`): `/hydrone/pads/markers`, where the colours are now
grey = candidate, cyan = confirmed, green = landed on, **orange = the takeoff
base**.

Stop it early: `ros2 service call /hydrone/mission/abort std_srvs/srv/Trigger` —
it lands where it is.

---

## 2. The flight

```
WAIT_FCU -> ARMING -> REGISTER -> TAKEOFF -> SELECT -+-> TRAVEL -> CONFIRM
              ^                                      |              |
              |                                      +-> ROTATE     |
              |                                      |    ^    |    |
              |                                      |    +- SETTLE |
              |                                      |              v
              +---------------- DWELL <----------------------------LAND
                                  |
                                  +-> DONE
```

| state | what it does | how it leaves |
|---|---|---|
| `WAIT_FCU` | wait for the MAVROS link, a local position and the command services | records `home`, goes to `ARMING` |
| `ARMING` | GUIDED, then arm. Re-sent on a timer, because a MAVROS ack is not ArduPilot's agreement | `/mavros/state` shows GUIDED **and** armed |
| `REGISTER` | calls `RegisterTakeoffBase` with the position we are standing at. **Once per run** | ack, or a 5 s grace if the service never appears |
| `TAKEOFF` | `CommandTOL` to `takeoff_alt`. Not a setpoint — ArduCopter will not climb from one in GUIDED | altitude reached; the first setpoint holds the position and heading it climbed with |
| `SELECT` | quota → home; else a confirmed candidate → `TRAVEL`; else `SETTLE` | immediately |
| `SETTLE` | hold still for `settle_s`, **then** read the map | candidate → `SELECT`; nothing and turns left → `ROTATE`; nothing and out of turns → the fallback |
| `ROTATE` | one 45° clockwise step, position unchanged | heading reached (or `rotate_timeout_s`), turn counted, back to `SETTLE` |
| `TRAVEL` | one setpoint, one leg, heading held | inside `arrive_tol_m` → `CONFIRM` (or `LAND`, going home); timeout → blacklist |
| `CONFIRM` | hover and count belly-camera looks | `confirm_detections` looks → `LAND`; timeout → blacklist, back to `SETTLE` |
| `LAND` | setpoint stream **stops**, LAND mode, wait for disarm or a settled low altitude | touchdown held for `land_settle_s` → mark visited → `DWELL` |
| `DWELL` | sit for `dwell_s` | more to do → `ARMING`; done → `DONE` |

Driven by a 10 Hz tick. **Every service call is asynchronous with its own
deadline** — a blocking call inside a timer callback would stall the setpoint
stream and hand the vehicle to the FCU's failsafe.

---

## 3. Why the search is a turn, not a pattern

`pad_mission_node` flies forward and lands on what it finds. That works in an
open field. In a 5×5 m arena it is the wrong shape of search, for one reason:

**every metre flown is visual-odometry drift.** The arena is texture-poor — ORB
finds ~46 keypoints in a whole frame, and LANDING-SITES §10 documents the VO
walking 0.39 m and 8.6° in 78 s while the drone sat still on the ground. A
search that translates spends its position estimate to buy coverage.

A 5×5 m square does not need that. From one spot at 1 m the forward ZED sees a
90°-wide slice of it, and eight headings cover the circle. So the search buys
coverage with **rotation**, which costs no translation at all, and the only
horizontal leg in the whole mission is the one to a base the drone has already
decided to land on.

### The pause is not politeness

Detection runs continuously — the detectors know nothing about this node — but
the mission only **acts** on the map after `settle_s` of holding still.

A detection is projected through `/mavros/local_position/pose`. Taken while yaw
is slewing at 25 °/s, that pose is stale by however long the image took to
arrive, and the pad lands in the map metres from the real thing. The drone then
flies to a place where there is nothing, fails to confirm, blacklists a real
base, and searches on. The settle window is what stands between the search and
that failure, and it is cheap: two seconds of hovering against a leg that would
have been flown for nothing.

Two seconds is also the ceiling deliberately. Standing still is not free either
— the VO drifts on the ground, let alone in a hover — so the window is sized to
let the estimate stop moving and no longer.

---

## 4. Why the takeoff base is declared instead of detected

The drone always starts standing **on** a base, and that base is not one of the
sites it must land on. It is a rectangle with a circular hole rather than the
disc-with-ring of a landing site, but it carries the same blue, the same yellow
and the same cross — and it **is** sometimes detected, most often from an
oblique angle, which is exactly the geometry the cameras have while sitting on
it.

Left alone, that produces a map candidate. Ruling it out means flying over it
and hovering for a confirmation that will probably fail. That is a travel leg
and up to 25 s of hover spent proving something the vehicle knew before the
propellers turned — and it is paid on every run, in the currency the mission has
least of.

So two changes in `pad_map_node`:

**Nothing is mapped before the first arm** (`require_armed`, default true). Not
"ignore the start base" — ignore everything. On the ground there is no useful
geometry to map from anyway: the belly camera is centimetres off a surface, the
forward camera is looking at the horizon and the sky, and both project through a
pose the EKF has not settled. The gate **latches**: the drone disarms on every
pad it lands on, and detections must keep flowing then.

**The base is registered, not detected.** `RegisterTakeoffBase` is called in the
instant between arming and climbing, which is the only moment the drone's
position *is* the base's position, to the centimetre, with no camera involved.
The entry it creates:

- carries `is_takeoff_base`, and the mission never offers it as a candidate;
- is **never pruned** — it has one "observation" and is not going to earn more;
- records `height_measured = true` at the altitude the drone is standing at,
  because standing on something measures it better than any fly-over;
- is not second-guessed by the rangefinder height refinement, for the same
  reason a visited pad is not;
- claims detections within `takeoff_base_radius` (1.5 m, wider than the 1.2 m
  ordinary `merge_radius`), so a glancing sighting from the air — the highest
  projection error there is — merges into it instead of spawning a phantom pad
  beside it;
- draws **orange** in RViz, checked before every other state, because grey, cyan
  and green all mean "a landing site in some condition" and this one means
  "never fly here".

Association compares `distance / claim_radius`, not raw distance, so the wider
claim cannot steal a detection that sits closer to a genuine pad.

Registration is **idempotent**: calling it again moves the flag and clears it
from the previous holder, so a restarted mission cannot end up with two takeoff
bases.

If the service never comes up, the mission logs it and flies anyway. It still
knows `home`, and `_is_candidate` refuses anything within 1 m of it, so the worst
case is a takeoff base that RViz draws as an ordinary pad.

---

## 5. Both cameras, and the two-stage decision

`pad_mission_node` threw away every forward-camera detection: it had no phase to
fly to a distant lead with, so acting on one only complicated the path. This
mission has exactly that phase, so both cameras are in the loop, with different
jobs:

- **The forward ZED identifies.** It sees across the whole arena. At those
  ranges the ring and the cross are a handful of pixels, so the detector's
  structure checks cannot resolve them and confidence is capped by design at
  0.75 (LANDING-SITES §3). Enough to fly to, never enough to land on.
- **The belly camera validates.** From directly above at 1 m the same structure
  is hundreds of pixels across. This is the look that decides.

The mission does not subscribe to forward detections directly. It reads
`/hydrone/pads/map`, which fuses both cameras and requires **three** sightings
before an entry is confirmed. One frame is noise; three fused frames with a
weighted position is a lead. That is the first gate.

The second gate is `CONFIRM`: `confirm_detections` (3) separate belly-camera
frames above `confirm_confidence` (0.60), inside `confirm_timeout_s`. Each frame
counts once — the detection is cleared after being counted, because the camera
and the tick both run at 10 Hz and a stale detection left in place would satisfy
the entire quota in 0.3 s. (`test_one_frame_cannot_satisfy_the_whole_quota`.)

A candidate that fails either gate is **blacklisted** and the search resumes
from wherever the drone is now, with the turn counter reset — the drone is
somewhere new facing a new direction, so the turns it made over the old spot say
nothing about what is visible from here. The blacklist is mission-local on
purpose: the map records what was *seen*, and a pad that failed confirmation was
genuinely seen. Deciding it is not worth a second visit is the mission's
judgement, so the mission keeps it.

---

## 6. Altitude: 1 m, for everything

`landing_sites` cruises at 2.5 m to clear the arena's 1.5 m structure. Phase 1
flies at 1 m, and that is a deliberate trade with two consequences worth knowing
before the first flight.

**It is chosen for cheapness, not for margin.** This is test code on hardware
that has never flown the mission; a fall from 1 m costs a propeller, a fall from
2.5 m costs the airframe.

**It therefore does not clear a 1.5 m structure.** There is no forward obstacle
avoidance in this stack at all. This launch assumes the Phase 1 arena is clear
at 1 m, which is the arena being flown. Raising `takeoff_alt` for an arena that
is not clear is the fix; nothing in the code checks.

**The belly camera's footprint at 1 m is ~2 m square** (320×240, 90° FOV). A 1 m
base stays in frame while the position error is under about half a metre, so the
confirmation hover depends on the VIO holding position to roughly that. If
`CONFIRM` starts timing out on bases that are genuinely there, that is the first
thing to suspect, and the first thing to try is a higher `takeoff_alt` — a
wider footprint costs a climb and buys tolerance. Detection itself is not the
constraint: at 1 m a 1 m base spans ~160 px, against the detector's ~18 px floor.

---

## 7. `ground_z`, and the assumption underneath the belly camera

The belly camera has no depth. It projects a pixel by intersecting its ray with
a horizontal plane at `ground_z` — and `ground_z` is measured **from the takeoff
plane**, which is the top of the base the drone started on, not the arena floor.

Everything in the arena is at ground level for the current test, so the takeoff
base is flush with the floor, the two planes coincide, and `ground_z = 0.0` is
correct. **When that stops being true** — a raised start base, or elevated
landing sites — `ground_z` must become minus the start base's height, or every
belly-camera projection is biased by that offset. The argument is exposed on
both launch files for exactly that day.

Elevated **landing** sites additionally need the rangefinder height refinement
already in `pad_map_node` (LANDING-SITES §5), which this mission does not
currently use for its descent: `LAND` is handed to the FCU whole, and the FCU
flares on its own rangefinder. That is fine at ground level and will need
revisiting for Phase 2.

---

## 8. Speed: FCU limits, not setpoint stepping

`pad_mission_node` walks the setpoint forward in 1 m steps so the position error
the FCU sees never exceeds one step. This mission does not: **one setpoint per
leg**, and the speed is the flight controller's business.

Chopping a 2 m leg into pieces does not make the vehicle gentler — the position
controller already accelerates against its own limits — it adds arrival tests,
each with a tolerance, evaluated against the least trustworthy signal in the
stack. Capping the controller's limits achieves the same thing at the source.

Three parameters, added to `config/params/holybro_sitl.parm`:

| parameter | firmware default | set to | why |
|---|---|---|---|
| `WP_SPD` | 10.0 m/s | **0.5 m/s** | the whole arena crossed in under a second is neither useful nor survivable indoors |
| `WP_ACC` | 2.5 m/s² | **0.5 m/s²** | acceleration is what the airframe feels when a setpoint appears, and BiguaSim's ~0.3–0.5 s actuation lag turns a sharp demand into an oscillation |
| `ATC_RATE_WPY_MAX` | 60 °/s | **25 °/s** | 60 °/s puts a 45° turn at 0.75 s, fast enough to smear the VO through the whole step. 25 °/s makes it ~1.8 s, and the settle pause then has something already nearly stationary to wait on |

GUIDED position targets take their limits from exactly these:
`ModeGuided::pos_control_run` calls
`NE_set_max_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss())`
(`ArduCopter/mode_guided.cpp:255`).

**These are not the block that was bisected out.** `holybro_sitl.parm:42` records
a "fly slow" tuning block removed on 2026-08-19 because it softened the
*attitude loop itself* — angle P 4.5→1.5, `ATC_INPUT_TC` 0.15→0.4, rate P/I/D
−41 %, `PSC_ANGLE_MAX` 30→10° — until the loop was too slow for the position
controller to track and had no authority left to recover. Nothing above touches
a gain or a lean limit. Capping demanded speed makes the position controller
**ask for less**; softening the attitude loop made it **worse at delivering**
what it was asked for. Only the first is safe, and only the first is here.

**Names.** This tree is ArduCopter 4.8.0-dev, where `WPNAV_*` was renamed `WP_*`
and converted from cm to SI, and `ATC_SLEW_YAW` (cdeg/s) became
`ATC_RATE_WPY_MAX` (deg/s). On the real drone's 4.6.3 the equivalents are
`WPNAV_SPEED 50`, `WPNAV_ACCEL 50` and `ATC_SLEW_YAW 2500`.

**Not touched, on purpose:** `WP_SPD_UP`, `WP_SPD_DN`, `PILOT_SPEED_UP`. The
climb is the one part of the flight that already behaves, and GUIDED takeoff
takes its rate from those.

### Yaw is commanded now

The old mission published every setpoint with `orientation.w = 1.0`, i.e. yaw 0
— which quietly commands the vehicle to face map-east regardless of where it
booted. This mission carries a real yaw in the setpoint quaternion, seeds it from
the heading the vehicle actually climbed with, and only ever changes it in
`ROTATE`. Clockwise **subtracts**: the map frame is ENU and yaw runs
counter-clockwise from east.

---

## 9. The fallback

Eight turns with nothing new in the map means the search has stopped producing.
The fallback is deliberately the dullest thing available: **land where you are,
take off once, land again, stop.**

No leg home. The reason the fallback fired is that the search stopped finding
anything, and a cross-arena leg on a position estimate that has been hovering
and turning for a few minutes is the last thing worth attempting. The hop exists
because it was specified as the end-of-cycle behaviour; it is also a free
end-of-run check that the vehicle still arms and climbs.

The normal ending is different: once `target_bases` landings are done, `SELECT`
sends the drone to the registered takeoff base and lands there. No confirmation
hover — we registered that base ourselves, there is nothing to prove.

---

## 10. What changed elsewhere in the stack

Everything below is a change made *for* this mission, in files that other things
also use.

| file | change | why |
|---|---|---|
| `hydrone_msgs/msg/Pad.msg` | **+ `bool is_takeoff_base`** | the map has to be able to say "this one is not a landing site". Appended at the end, so the field order of everything before it is unchanged |
| `hydrone_msgs/srv/RegisterTakeoffBase.srv` | **new** | declare the base under the drone at arm time |
| `hydrone_msgs/CMakeLists.txt` | registers the new service | |
| `hydrone_nav/pad_map_node.py` | `require_armed` gate; `RegisterTakeoffBase` service; `takeoff_base_radius`; ratio-based association; takeoff base exempt from pruning and from rangefinder height refinement; orange marker + `takeoff-base` label | §4 |
| `hydrone_nav/package.xml` | **+ `mavros_msgs`** | the node now subscribes to `/mavros/state` |
| `hydrone_mission/phase1_mission_node.py` | **new** | the mission |
| `hydrone_mission/setup.py` | registers the entry point | |
| `hydrone_bringup/launch/phase1.launch.py` | **new** | autonomy layer |
| `hydrone_bringup/launch/phase1_sim.launch.py` | **new** | sim wrapper |
| `scripts/docker_up.sh` | **+ `--phase1`**, and any `name:=value` argument is now forwarded to the launch instead of to compose | one command from a cold host, and mission tuning without editing a file |
| `docker-compose.yml` | the `command:` gains `${HYDRONE_LAUNCH_ARGS:-}` | how those pairs reach `ros2 launch`. Deliberately unquoted — compose splits the interpolated string shell-style, which is what turns several pairs into several arguments and an empty value into nothing |
| `hydrone_bringup/launch/landing_sites_sim.launch.py` | stopped re-declaring and forwarding the mission's arguments | the identical shadowing defect. Its mirrored defaults happened to match `landing_sites.launch.py`'s exactly, so this changes nothing today and unbreaks every future edit |
| `hydrone_bringup/test/test_launch_arguments.py` | **new** (4 tests) | a sim wrapper must not re-declare, or forward, an argument its autonomy layer declares. Both pairs checked |
| `hydrone_bringup/config/params/holybro_sitl.parm` | `WP_SPD`, `WP_ACC`, `ATC_RATE_WPY_MAX` | §8 |
| `hydrone_nav/test/test_pad_pipeline.py` | its `FakeSim` now publishes `/mavros/state` armed | it stands in for MAVROS, and with the arm gate a stack that never arms maps nothing. Five tests failed on this and now pass for the right reason |

**Nothing about the detector changed.** The HSV thresholds in
`phase1.launch.py` are copied verbatim from `landing_sites.launch.py`, including
the measurement that produced them. The detector is the partially-validated part
of this stack and forking its tuning would be the fastest way to lose that.

`landing_sites.launch.py` and `pad_mission_node.py` are untouched and still
runnable. Note that `landing_sites.launch.py:279` has its mission node commented
out, so that launch currently flies nothing — that is pre-existing, not a change
made here.

### A naming error found on the way

`docs/PARAMS-DIFF-SITL.md` §1 lists the removed tuning block's speed limits as
`WPNAV_SPD` and `WPNAV_ACC`. Neither is a real parameter in this firmware: the
group prefix is `WP_` (`ArduCopter/Parameters.cpp:369`), so the names are
`WP_SPD` and `WP_ACC`. If that block is ever re-applied verbatim, those two lines
would be silently ignored — a `.parm` file does not complain about a name it does
not recognise.

---

## 11. Tests

69 new tests, none needing UE5 or a flight:

```bash
docker run --rm -v $PWD:/repo -w /repo <image> bash -c \
  '. /ws/install/setup.sh && python3 -m pytest \
   src/hydrone_mission/test/test_phase1_mission.py \
   src/hydrone_nav/test/test_takeoff_base.py -q'
```

| file | what it covers |
|---|---|
| `hydrone_mission/test/test_phase1_mission.py` (41) | candidate selection (takeoff base, visited, blacklisted, unconfirmed, near-home all refused; nearest wins); the settle gate; clockwise turns that wrap and do not translate; termination after exactly `max_rotations`; travel and arrival; the confirmation quota and its three refusal cases; the blacklist; the fallback hop; the yaw carried on the wire |
| `hydrone_nav/test/test_takeoff_base.py` (24) | the arm gate and its latch; registration creating, claiming and re-flagging; the wider claim radius and that it cannot steal a nearer pad's detection; never pruned; height not rewritten by a fly-over; the flag surviving into the published map; the marker colour; **every rejection gate naming itself in the log, an accepted detection staying quiet, and two gates not sharing one throttle bucket** |
| `hydrone_bringup/test/test_launch_arguments.py` (4) | neither sim wrapper re-declares or forwards an argument its autonomy layer declares — the tuning-does-not-apply bug |

The full suite is **166 tests** and all pass.

The flight states — arming, takeoff, landing — are **not** unit-tested. They are
conversations with ArduPilot, and mocking one proves nothing about the real
vehicle. They are exercised by flying the sim.

---

## 12. "The detector sees it, the map does not"

There is **no distance cap anywhere near the size of the arena.** Every gate a
detection passes, and the value it is checked against:

| gate | where | default | notes |
|---|---|---|---|
| blue-blob area | `pad_detector.py` | `min_area_px` 150 | a 1 m base at 8 m spans ~40 px across, ~1600 px² — clears it by 10× |
| detector confidence | `pad_detector.py` | `min_confidence` 0.50 | |
| pose freshness | `pad_detector_node` | `max_pose_age_s` 1.0 | |
| projection possible | `pad_detector_node._project` | — | fails with no `camera_info`, no/stale pose, no mount TF, or a ray at or above the horizon. Sets `position_valid = false` |
| `position_valid` | `pad_map_node` | — | |
| map confidence | `pad_map_node` | `min_confidence` 0.50 | |
| **range** | `pad_map_node` | **`max_range_m` 30.0** | the only distance cap there is. The 8×8 m arena's diagonal is 11.3 m |
| projected height | `pad_map_node` | `min_pad_height` −0.5 … `max_pad_height` 2.0 | catches a reflection or a bad depth sample |

So raising a cap to 8 m would change nothing: the one distance cap is already
30 m. Something else is dropping the detection, and until 2026-08-22 nothing
said what — the detector kept drawing a confident box while the map stayed
empty, which from outside is indistinguishable from "the detector never saw it".

**`pad_map_node` now names the gate.** Every rejection logs, throttled to one
line per 5 s:

```
detection from forward at (7.31, 0.42, 0.00) conf 0.70 range 7.8 m REJECTED:
position_valid is false — the detector could not project it (no camera_info,
no/stale pose, or a ray at or above the horizon)
```

**Where it appears.** `pad_map` is launched with `output="screen"`, so the line
goes to the container's stdout:

```bash
# docker_up.sh runs compose in the foreground — it is already on that terminal.
# From anywhere else, or if you passed -d:
docker compose logs -f hydrone | grep --line-buffered REJECTED

# ROS keeps its own copy inside the container:
docker compose exec -u hydrone hydrone \
    bash -c 'tail -f ~/.ros/log/latest_launch/pad_map-*-stdout.log'
```

Note the `-u hydrone`: `docker compose exec` is root by default, and a root ROS
process in this container receives nothing over FastDDS shared memory.

Repeats are throttled per **(camera, gate)** — 5 s by default,
`reject_log_period_s:=0` to see every one. The pairing matters: rclpy's own
`throttle_duration_sec` keys on the call site, and all four gates share one
logging line, so the built-in throttle would have let a belly-camera rejection
at 10 Hz hide the forward camera's reason for 5 s at a time.

Read the line and it says which of the eight rows above closed. The likely ones
at range, in order:

1. **`position_valid is false`.** The forward camera projects through the ZED's
   depth, and `zed_mimic_node` turns the sim's far-plane sentinel into NaN
   (`depth[depth >= 655.0] = np.nan`). Where depth is unusable the detector
   falls back to intersecting the ray with the ground plane — and that fallback
   **refuses a ray at or above the horizon**. A base at the far end of the arena
   sits only a few degrees below the horizon, so a small pitch-up, or a
   `ground_z` that does not match the real floor, flips the ray the wrong side
   of level and the projection is refused outright.
2. **`range ... > max_range_m`.** Not from honest distance, but from a
   near-level ground-plane ray: the intersection runs away and the range comes
   out in the hundreds of metres. Same root cause as 1, one degree the other
   side.
3. **`projected z outside [...]`.** A depth sample from the floor beyond the pad,
   or from the pad's edge.

All three point at the same place — the forward camera's projection geometry at
a shallow look-down angle — not at a threshold that needs relaxing. **Altitude
is the lever**: at 1 m a base 8 m away is 7° below the horizon; at 4 m it is 27°,
which is far outside the noise. That is worth knowing before assuming the
detector is at fault.

If the line says `max_range_m`, raise it — but check `range_m` in the message
first: a plausible number means a genuine cap, an absurd one means the geometry
above.

Also worth ruling out first, because it costs nothing: **the marker may exist and
be drawn somewhere you are not looking.** `/hydrone/pads/markers` is only a view
of `/hydrone/pads/map`; echo the map itself before concluding the entry is
missing.

---

## 13. What to watch on the first flight

Nothing in this document has been observed in the air. The state machine, the
map changes and the selection logic are tested; the flight is not. In rough
order of how likely each is to be the thing that bites:

1. **The turn itself.** Rotating in place with few features is the motion visual
   odometry handles worst, and this mission is mostly that motion. Watch
   `odom_error_node`'s output across a full eight-turn circle: if the estimate
   walks by more than the arrive tolerance over one search, the map positions
   collected on the first heading are not usable by the time the drone gets
   round to flying to them, and the search needs re-anchoring (a re-detection
   pass on arrival, not a bigger tolerance).
2. **`CONFIRM` timing out on real bases.** The ~2 m footprint at 1 m altitude is
   the tight constraint (§6). Symptom: good bases blacklisted one after another.
   First remedy: raise `takeoff_alt`.
3. **The `provisional_ttl_s` / wall-clock trap.** BiguaSim runs 5–8× below real
   time and every timeout here is wall-clock. `pad_map`'s TTL is already raised
   to 120 s for this reason; `confirm_timeout_s` and `travel_timeout_s` have not
   been flown and may well be short in sim terms.
4. **`WP_SPD 0.5` against the actuation lag.** The speed cap is new and untested
   in flight. If the vehicle oscillates on a leg, that is the position
   controller and the lag, not the mission — and it is the parameter to move,
   not the attitude gains (§8).
5. **Whether the forward camera actually finds bases across the arena.** The
   geometry says yes comfortably (a 1 m base at 4 m spans ~80 px against an
   ~18 px floor), and detection is partially validated, but it has been
   validated on the belly camera. If the search never produces a lead, look at
   `/hydrone/pads/forward/debug_image` before touching the state machine.
