# `holybro_sitl.parm`: remote `develop-pipelines` vs. what is running now

Written 2026-08-19. Scope: every difference between the params file as committed
on `origin/develop-pipelines` and the file the SITL actually boots with today.

## 0. Where the file comes from, and is the working copy really the one in use?

Yes. Verified:

- `sources_sim.launch.py:227` builds the path from
  `get_package_share_directory('hydrone_bringup')` and passes it to the SITL as
  `defaults='<holybro_sitl.parm>,<dds_udp.parm>'` (line ~256).
- `install/hydrone_bringup/share/hydrone_bringup/config/params/holybro_sitl.parm`
  is a **symlink** into `build/`, and `build/`'s copy is byte-identical to
  `src/`. So editing the source file changes the running config with no rebuild.
- The launch also passes `wipe: 'True'`, so the defaults file wins over anything
  previously stored, and a live `param set` does not survive a restart.
- The second defaults file, `dds_udp.parm`, contains only `DDS_ENABLE` and
  `DDS_UDP_PORT` — it overrides nothing discussed here.

**All of the differences below are uncommitted working-tree edits, dated
2026-08-18 13:37.** The two local commits ahead of the remote
(`ac11cc450`, `88c2e695a`) do **not** touch this file. The last commit that did
is `6b8f3124e`, which is already on the remote.

Diff size: `92 ++-` (85 insertions, 7 deletions), of which ~55 lines are comments.

---

## 1. Flight-tuning block — ADDED (this is the part that changes how it flies)

Inserted after `ARSPD_BUS 2`. None of these parameters existed in the remote
file, so before this edit each one sat at the firmware default.

| Parameter | Remote (firmware default) | Now | Change | Verified in `src/ardupilot` |
|---|---|---|---|---|
| `PSC_ANGLE_MAX` | `0` → falls back to `ATC_ANGLE_MAX` = **30 deg** | `10` deg | **-67 % lean authority** | `AC_PosControl.cpp:125` (`_ANGLE_MAX`, 0 = use `ANGLE_MAX`); `AC_AttitudeControl.cpp:26` (`ANGLE_MAX` default 30) |
| `WPNAV_SPD` | **10.0 m/s** | `1.5` m/s | **-85 %** | `AC_WPNav.cpp:56`, `WP_SPD_DEFAULT` = 10.0 m/s |
| `WPNAV_ACC` | **2.5 m/s²** | `0.8` m/s² | **-68 %** | `AC_WPNav.h:15`, `WPNAV_ACCELERATION_MS` = 2.5 |
| `WPNAV_SPD_UP` | **2.5 m/s** | `1.0` m/s | -60 % | `AC_WPNav.cpp:74` |
| `WPNAV_SPD_DN` | **1.5 m/s** | `0.75` m/s | -50 % | `AC_WPNav.cpp:83` |
| `WPNAV_JERK` | **1.0 m/s³** | `0.5` m/s³ | -50 % | `AC_WPNav.cpp:37` |
| `ATC_INPUT_TC` | **0.15 s** ("Medium") | `0.4` s | **+167 %, softer than the "Very Soft" 0.5 preset's neighbour** | `AC_AttitudeControl.cpp:11,134` |
| `ATC_ANG_RLL_P` | **4.5** | `1.5` | **-67 %** | `AC_AttitudeControl.h:15`, `AC_ATTITUDE_CONTROL_ANGLE_P` = 4.5 |
| `ATC_ANG_PIT_P` | **4.5** | `1.5` | **-67 %** | same |
| `ATC_RAT_RLL_P` | **0.135** | `0.08` | -41 % | `AC_AttitudeControl_Multi.h:6` |
| `ATC_RAT_RLL_I` | **0.135** | `0.08` | -41 % | `AC_AttitudeControl_Multi.h:14` |
| `ATC_RAT_RLL_D` | **0.0036** | `0.002` | -44 % | `AC_AttitudeControl_Multi.h:17` |
| `ATC_RAT_PIT_P` | **0.135** | `0.08` | -41 % | same |
| `ATC_RAT_PIT_I` | **0.135** | `0.08` | -41 % | same |
| `ATC_RAT_PIT_D` | **0.0036** | `0.002` | -44 % | same |

Unchanged and pre-existing on the remote: `ATC_RAT_YAW_P 0.3`, `ATC_RAT_YAW_I 0.02`.
Untouched by the edit (still at firmware default): all `ATC_RAT_*_FLTD/FLTT`
(20 Hz), `ATC_ACC_*_MAX`, `ATC_ANG_YAW_P`, `ATC_ANGLE_MAX`, `PSC_*` gains.

### 1a. Things that check out

- The renamed, unit-aware `WPNAV_*` names in the added comment are **correct for
  this build**: `AC_WPNav.cpp` defines `SPD` (m/s), `SPD_UP`, `SPD_DN`, `ACC`
  (m/s²), `JERK`. The old `WPNAV_SPEED` (cm/s) does **not** exist here —
  `grep '"SPEED"'` in `AC_WPNav.cpp` returns nothing. So the claim that
  old-name lines were being silently dropped is right.
- `PSC_ANGLE_MAX` really is degrees, and `0` means "use `ATC_ANGLE_MAX`".
- No duplicate keys in the file (checked: `uniq -d` over all keys is empty), so
  nothing later in the file quietly overrides these.

### 1b. Things that are suspicious — the likely cause of the bad handling

**Four independent softenings are stacked on the same loop.** The rate-loop
reduction (-41 %) is defensible on its own for a laggy plant. What is layered on
top of it is not obviously intended:

1. `ATC_ANG_RLL_P` / `ATC_ANG_PIT_P` 4.5 → 1.5 slows the **outer angle loop 3x**.
2. `ATC_INPUT_TC` 0.15 → 0.4 slows the **command shaper another ~2.7x**.
   This is *not* a pilot-input-only parameter in autonomous flight:
   `input_thrust_vector_rate_heading_rads()` and
   `input_thrust_vector_heading_rad()` — the entry points the position
   controller uses in Guided/Auto — both feed `_input_tc` into
   `attitude_command_model()` (`AC_AttitudeControl.cpp:859-860, 911-912`). So
   it applies to every autonomous maneuver.
3. `PSC_ANGLE_MAX` 30 → 10 deg then **caps the authority available to recover**
   from whatever error the now-sluggish loop accumulates.

Net effect: the attitude loop tracks demands far more slowly, the position
controller sees a persistent tracking lag, its error and integrator grow, and
the correction it finally asks for is clipped at 10 deg. That is a textbook
recipe for slow position-loop oscillation and sloppy/divergent behaviour on
simple moves — which matches the symptom, and it is a *different* failure from
the fast attitude flip the comment says it was fixing.

For scale, `WPNAV_ACC 0.8 m/s²` needs only `atan(0.8/9.81) ≈ 4.7 deg` of lean in
steady state, so the 10 deg cap is not limiting in cruise — it is limiting
exactly when things go wrong.

**There is an untapped reference tune sitting in the repo.**
`src/hydrone_bringup/config/params/holybro_x500_tuned.parm` holds a real
autotune result for this airframe:

```
ATC_ANG_RLL_P    4.5          ATC_RAT_RLL_P 0.1255951   ATC_RAT_RLL_D 0.004905871
ATC_ANG_PIT_P    4.5          ATC_RAT_PIT_P 0.1466683   ATC_RAT_PIT_D 0.005201788
ATC_RAT_RLL_FLTD/FLTT 21      MOT_THST_HOVER 0.2905434
```

`grep -rn x500_tuned src scripts docs` returns **nothing** — no launch file, no
script loads it. It is dead weight in the tree, and its gains are ~1.6x the ones
now in `holybro_sitl.parm` at the rate loop and 3x at the angle loop.

---

## 2. IMU / arming block — CHANGED

| Parameter | Remote | Now | Note |
|---|---|---|---|
| `SIM_IMU_COUNT` | *(absent → SITL default 2)* | `1` | new |
| `SIM_ACC1_RND` | *(absent → default)* | `0` | new |
| `SIM_ACC2_RND` | *(absent)* | `0` | new |
| `SIM_GYR1_RND` | *(absent)* | `0` | new |
| `SIM_GYR2_RND` | *(absent)* | `0` | new |
| `INS_USE2` | *(absent → 1)* | `0` | new; redundant once `SIM_IMU_COUNT 1` |
| `INS_USE3` | *(absent → 1)* | `0` | new; redundant |
| `EK3_IMU_MASK` | *(absent → 3)* | `1` | new; EKF3 runs one core |
| `INS_ACC2OFFS_X/Y/Z` | `0.001` | `0.000` | **modified** |
| `INS_ACC2SCAL_X/Y/Z` | `1.001` | `1.000` | **modified** |
| `INS_ACCOFFS_*`, `INS_ACCSCAL_*` | `0.001` / `1.001` | unchanged | |
| `INS_ACC3*` | `0.000` / `1.000` | unchanged | |

This block is internally consistent and its stated reasoning holds:
`accel_calibrated_ok_all()` requires instances at or beyond `get_accel_count()`
to have zero offsets and unit scale, so with `SIM_IMU_COUNT 1` the `INS_ACC2*`
values **must** be zeroed. It is arming-gate work, not tuning work, and it does
not explain the handling regression. The one operational caveat is the coupling
the comment already calls out: if `SIM_IMU_COUNT` ever goes back to 2,
`INS_ACC2OFFS/SCAL` must go back to `0.001`/`1.001` or arming breaks the other way.

Side effect worth knowing: with `EK3_IMU_MASK 1` there is a single EKF core and
no IMU affinity/failover, so an EKF lane-switch bug can no longer be reproduced
in sim.

---

## 3. Comment-only changes

The `# we need small INS_ACC offsets ...` one-liner was replaced by a ~30-line
explanation of the arming hold-down timer, and a ~13-line rationale block was
added above the tuning parameters. No behavioural effect.

---

## 4. Not changed (rules these out as causes)

Byte-identical to the remote: `EK3_SRC1_POSXY/VELXY/POSZ/VELZ/YAW`, `VISO_TYPE`,
`GPS1_TYPE`, `RNGFND1_*`, `EK3_RNG_USE_HGT`, `SCHED_LOOP_RATE 100`, `FRAME_CLASS/TYPE`,
`MOT_THST_EXPO/HOVER`, `MOT_BAT_VOLT_MIN/MAX`, `FENCE_RADIUS`, all `RC*`, all
`FLTMODE*`, `COMPASS_OFS*`, `BATT_MONITOR`, `DDS_DOMAIN_ID`, `FLTMODE_CH`.

So the estimator source configuration is untouched. If the EKF is being fed
ground-truth position and the vehicle still loses control, the estimator is not
the variable that changed — the tune is.

Related uncommitted change in the same session, same date, that interacts with
this (`src/biguasim-ros2/biguasim_main/config/config.yaml`):

- ZED RGB + Depth cameras `Hz: 20 → 10`, and a third `DownCamera` @ 10 Hz added.
- Spawn `location: [0, 0, 0.3] → [0, -3, 0.27]`.

The render-load change is the stated *reason* for the retune, so the two edits
are coupled: if the camera budget is now back under control, the aggressive
softening may no longer be needed at all.

---

## 5. Suggested way to bisect this (smallest step first)

1. Revert only the tuning block, keep the IMU/arming block:
   `git diff origin/develop-pipelines -- <file>` shows the two hunks separately;
   drop the first hunk. Fly the simple move. If it flies clean, the tune is
   confirmed as the regression.
2. If the flip returns, re-apply **only** the rate-loop reduction
   (`ATC_RAT_*_P/I/D`), leaving `ATC_ANG_*_P` at 4.5 and `ATC_INPUT_TC` at 0.15.
3. Only if that still flips, add `ATC_INPUT_TC 0.25` (not 0.4), and leave
   `PSC_ANGLE_MAX` at 20 rather than 10 so there is recovery authority.
4. Consider seeding from `holybro_x500_tuned.parm` instead of from hand-picked
   numbers — or delete that file if it is genuinely obsolete, because right now
   it reads as authoritative and is loaded by nothing.

---

## 6. Appendix — the removed tuning block, verbatim

Removed from `holybro_sitl.parm` on 2026-08-19 as bisect step 1. It was never
committed, so this appendix is the only copy. Re-apply by pasting it back after
the `ARSPD_BUS 2` line — or, for step 2, paste back only the `ATC_RAT_*_P/I/D`
lines and leave `ATC_ANG_*_P`, `ATC_INPUT_TC` and `PSC_ANGLE_MAX` out.

```
# --- Fly SLOW: the BiguaSim X500 answers rate demands with ~0.3 s of lag ---
# Measured in the dataflash RATE stream (2026-08-18, three separate crashes,
# ground-truth odometry, so not an estimator fault): the gyro follows the
# demanded rate ~0.3-0.5 s late with ~2x overshoot. Hover is fine (near-zero
# demand), but the FIRST aggressive position-controller demand — the first
# search leg — drives the default-tuned attitude loop into a growing
# oscillation that flips the airframe in seconds ("Crash: AngErr=170>30",
# every run, always right after lateral motion starts). With that much plant
# lag the default tune is simply outside its stable envelope, so: cap the
# tilt, cap the speeds/accels, soften the attitude loops. Slow and stable
# beats fast and upside down; retune toward "fast" only in small steps.
# NOTE this ArduPilot build (4.8-dev) uses the RENAMED, unit-aware parameter
# names: WPNAV_SPD is m/s (not WPNAV_SPEED in cm/s), WPNAV_ACC is m/s^2,
# PSC_ANGLE_MAX is degrees. The old names DO NOT EXIST here and a defaults
# file line with an unknown name is silently ignored — verified 2026-08-18 by
# reading the PARM records out of the dataflash log: the ATC_* gains loaded,
# the old-name limit lines did not, and the flight demanded 22 deg of lean at
# full speed and diverged exactly like the untuned runs.
PSC_ANGLE_MAX   10
WPNAV_SPD       1.5
WPNAV_ACC       0.8
WPNAV_SPD_UP    1.0
WPNAV_SPD_DN    0.75
WPNAV_JERK      0.5
ATC_INPUT_TC    0.4
ATC_ANG_RLL_P   1.5
ATC_ANG_PIT_P   1.5
ATC_RAT_RLL_P   0.08
ATC_RAT_RLL_I   0.08
ATC_RAT_RLL_D   0.002
ATC_RAT_PIT_P   0.08
ATC_RAT_PIT_I   0.08
ATC_RAT_PIT_D   0.002
```
