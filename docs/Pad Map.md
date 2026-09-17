---
tags: [hydrone, mapping, mission]
---
# Pad Map

Back to [[Hydrone]]. How `pad_map_node` fuses the [[Pad Detector]]'s
per-frame detections into a handful of persistent, mission-usable entries.

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
  why — see [[Phase 1 Mission#"The detector sees it, the map does not"|Phase 1 Mission §12]] for the gate table and
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
  [[Phase 1 Mission#Why the takeoff base is declared instead of detected|Phase 1 Mission §4]]. `pad_mission_node` is unaffected
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
