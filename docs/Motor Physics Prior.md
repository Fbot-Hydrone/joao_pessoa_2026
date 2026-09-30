---
tags: [hydrone, phase4, plan]
---
# Motor Physics Prior

Back to [[Hydrone]]. The cheapest sanity check available to Phase 4: estimate
where the drone should be moving from motor RPM and Pixhawk attitude, and gate
lidar-derived motion against it. See [[Phase 4 Pipeline]] for where this sits
in the roadmap and [[Sensor Fallbacks]] for where it becomes the last resort.

## The model

1. **Thrust per motor** from RPM: `T ≈ k_T · RPM²` (quadratic, standard
   propeller thrust model). Sum and combine with each motor's known mount
   position/orientation on the X8 frame to get total thrust vector and net
   torque in the body frame.
2. **Rotate to world frame** using the Pixhawk's attitude (quaternion from
   `/mavros/imu/data` or the EKF's own attitude estimate — attitude is
   trustworthy even when position is not, since it does not depend on GPS or
   lidar).
3. **Expected acceleration** = thrust vector (world frame) / mass − gravity −
   modelled drag. Integrate once for expected velocity direction, twice
   (short horizon only) for expected displacement direction.
4. **Error cone**: the expected displacement direction carries an uncertainty
   margin (drag, motor response lag, RPM measurement noise). Lidar-derived
   motion whose direction falls outside that cone in the gating window is
   flagged inconsistent.
5. **Gate**: `motion_prior_node` publishes the expected direction plus a
   consistency flag. Downstream (LIO gating, or later the degraded-sensing
   layer) uses the flag to discard or down-weight a lidar displacement
   estimate that disagrees with the physics prior.

This is intentionally crude — it is not meant to replace the LIO, only to
catch the case where the LIO's motion estimate is *obviously* wrong (e.g. a
degenerate geometry solution, or a lying/dead sensor upstream).

## What real hardware needs that sim does not

- **ESC RPM telemetry** over DShot/BDShot to the Pixhawk.
- ArduPilot relaying that as MAVLink `ESC_TELEMETRY`.
- MAVROS exposing it as a ROS topic `motion_prior_node` can subscribe to.
- **Thrust coefficient calibration** (`k_T` per motor/propeller combo,
  probably assumed uniform across the X8's eight arms as a first pass) — a
  bench test (known load cell reading vs. commanded RPM) or a flight-derived
  fit against a trusted odometry source during initial LIO validation.

None of this exists yet in the repo; ESC telemetry plumbing is a real-hardware
prerequisite this note flags but does not implement.

## Known limits

- **Drag** is not modelled beyond a coarse constant/linear term — good enough
  for a direction check, not for precise dead reckoning.
- **Wind** is invisible to this model entirely; it will bias the expected
  direction against real motion whenever wind is nontrivial relative to the
  drone's airspeed.
- **Ground effect** near the floor or near the maze structure changes the
  thrust-to-RPM relationship; the model does not account for it, so the error
  cone should be widened during takeoff/landing and close-proximity flight.
- **Battery sag** changes the RPM↔thrust relationship over a flight (lower
  voltage, same RPM commanded, less thrust available at saturation) — worth
  revisiting once the model is in closed loop and drift is being measured
  against ground truth.

## Status

**Now:** design only, captured here. **Next:** implement `motion_prior_node`
against sim RPM/attitude topics as part of [[Phase 4 Pipeline]] step 2.
**Later:** real-hardware ESC telemetry wiring and thrust coefficient
calibration.
