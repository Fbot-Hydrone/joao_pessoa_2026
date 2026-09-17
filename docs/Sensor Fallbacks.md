---
tags: [hydrone, phase4, plan]
---
# Sensor Fallbacks

Back to [[Hydrone]]. The fallback ladder Phase 4 falls back through as sensors
are declared suspect/lying/dead by [[Degraded Sensing]], ending in motor RPM +
physics dead reckoning whose only goal is recovering the hardware, not
completing the mission. See [[Phase 4 Pipeline]] for where this sits.

## The ladder

| level | sources used | expected accuracy | enter when | leave when |
|---|---|---|---|---|
| 0 — full | Mid-360 LIO + Pixhawk attitude + [[Motor Physics Prior]] gate + (future) RGB | best available — this is [[LIO Odometry]] at full health | default state | — |
| 1 — lidar degraded | LIO with degenerate-geometry compensation (reduced confidence in the underconstrained axis) + physics prior weighted up | reduced, direction-of-travel still usable | [[Degraded Sensing]] flags lidar suspect (corridor/open-field geometry, dust/glass/sun) | lidar returns to healthy |
| 2 — lidar dead, camera live | RGB-only odometry / place recognition ([[RGB Camera Enhancement]]) + physics prior | coarse, texture-dependent (arena is texture-poor — see [[RGB Camera Enhancement]]) | lidar declared dead, RGB camera present and healthy | lidar recovers, or RGB also degrades |
| 3 — vision-denied | physics prior only, with attitude from Pixhawk | dead-reckoning drift, no bound | both lidar and camera dead/lying | any vision source recovers |
| 4 — recovery only | motor RPM + rigid-body physics simulation, no sensing input at all | not navigation — just "keep the airframe under control and land/hold" | attitude source itself is untrustworthy or all exteroceptive sensing is gone | never — this is the floor |

Level 4 is deliberately not a navigation mode: its only goal is to stop the
airframe doing something that damages the hardware — hold attitude, descend,
or hold position as well as open-loop physics allows — while whatever is left
of telemetry gets the vehicle home or lets a human intervene.

## Notes

- Levels are per-capability, not global — the drone might run lidar odometry
  at level 0 while the RGB camera (once it exists) is separately flagged
  unhealthy and contributing nothing; the ladder describes what each level
  *would* fall back to if the level above stopped working, not necessarily
  a single global mode variable.
- Entering and leaving a level should have separate triggers (hysteresis) —
  a sensor that flaps between healthy and suspect should not thrash the whole
  pipeline between levels every cycle. Exact thresholds are implementation
  detail, deferred until [[Degraded Sensing]] has something to gate on.
- This ladder assumes the Pixhawk's attitude estimate (from its own IMU) stays
  trustworthy longer than anything else — it is the one signal every level
  above 4 still relies on. If attitude itself is compromised (e.g. IMU
  saturation, see [[Degraded Sensing]]), there is nothing below level 4 to
  fall back to.

## Status

Design only. Depends on [[Degraded Sensing]] existing to produce the health
flags this ladder reacts to, and on [[LIO Odometry]] existing as level 0.
