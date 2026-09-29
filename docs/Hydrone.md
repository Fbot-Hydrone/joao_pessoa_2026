---
tags: [hydrone, index]
---
# Hydrone

ROS 2 Humble drone stack for the CBR 2026 Flying Robot League, simulated in
BiguaSim (Unreal) + ArduPilot SITL and run in Docker. Phase 1 lands on marked
pads using ZED visual odometry; Phase 4 is a second airframe (Kopis X8 +
Livox Mid-360) that must navigate a small maze on its own lidar-inertial
sensing, no GPS, no ground truth in the loop.

## Notes

### Phase 1
- [[Phase 1 Mission]]: take off, turn until you see a base, land on it, come home
- [[Develop Pipelines]]: GPS-denied flight on ZED visual odometry — the plumbing Phase 1 flies on
- [[Params Diff SITL]]: `holybro_sitl.parm` tuning history and the bisect for a flip-on-first-maneuver bug

### Landing sites
- [[Landing Sites]]: the earlier find/land/take-off/repeat mission, and the machinery it shares with Phase 1
- [[Pad Detector]]: the OpenCV pad-detection pipeline, both sim and real-arena tuning
- [[Pad Map]]: fusing per-frame detections into a persistent, mission-usable pad map
- [[ZED Feature Map]]: accumulating the ZED's point cloud into a world map + coverage grid
- [[ZED Visual Odometry]]: the real VO estimate vs. ground truth, and how it drifts

### Phase 4
- [[Phase 4 Pipeline]]: overview and layered roadmap, data-flow table
- [[Livox Mid-360 Sim]]: TODO — sim replica of the Mid-360
- [[LIO Selection]]: TODO — choosing the LIO core
- [[LIO Odometry]]: TODO — closed-loop LIO + drift measurement
- [[Persistent Map]]: TODO — voxel map + deduplicated cloud + keyframes
- [[Motor Physics Prior]]: RPM² thrust model, attitude, expected-displacement gating
- [[Degraded Sensing]]: sensor health states, detecting lying sensors, online error modelling
- [[Sensor Fallbacks]]: the fallback ladder down to RPM + physics dead reckoning
- [[RGB Camera Enhancement]]: future roles for an RGB camera, once mounted
- [[Phase 4 Maze Mission]]: the maze goal, a minimal approach, open questions

### Sensors & calibration
- [[Sensor Config]]: the authoritative sensor spec for Phases 1 & 2 — mounting, EKF3 sources, sim wiring
- [[Calibration]]: calibrating the belly camera (ChArUco capture + offline solve)
- [[Rangefinder Sim]] (pt-BR): how the nadir rangefinder is injected in sim and why it's sim-only
- [[Config Single Source]]: audit of hardcoded sim values that duplicated `config.yaml`
- [[Sim Settings]]: `sim_settings.yaml`, the airframe-independent sim settings (UE5 viewport resolution)

### Infra & scripts
- [[Scripts]]: what every script in `scripts/` is for
- [[Jetson Real Stack]]: running the stack on real hardware — legacy Jetson, ZED 1, USB belly camera
- [[Remote Install Nautec]]: install debrief for the `nautec` lab machine

### History
- [[Fixes 2026-07-28]]: the session that took SITL bring-up from "won't fly" to flyable
- [[Chore Branch Archive]]: two commits from a deleted branch, kept so the work can be redone
- [[Onboarding Notes]] (outdated): pre-refactor exploration notes, kept for context only

## Decisions taken with the user

- Phase 4 flies on LIO, never on ground truth — ground truth is only used to
  measure drift.
- Compute target for Phase 4 is the Jetson Nano, or at worst a Raspberry Pi 5 —
  the pipeline must be CPU-light throughout.
- Adopt an existing LIO core and add team-specific hooks, rather than writing
  scan registration from scratch. Chosen: FAST-LIO2 ROS 2 branch, vendored
  ([[LIO Selection]]).
- The simulated Mid-360 is BiguaSim's engine RaycastLidar, wobbled ±3° per tick
  so scans don't repeat, instead of spinning depth cameras
  ([[Livox Mid-360 Sim]]).
- The real drone's ESCs report RPM; in sim the bridge publishes the same
  `mavros_msgs/ESCTelemetry` because ArduPilot's JSON SITL has no RPM input.
- Both IMUs are used: the Mid-360's internal IMU feeds the LIO core; the
  Pixhawk's IMU + ESC RPM feed the motor physics prior.
- The persistent map is a voxel map plus a deduplicated plain point cloud (not
  a dense map or a full pose graph, at least for the minimal version).
- Local sim only for Phase 4 — the remote-world mode can't rotate sensors,
  which the Mid-360's scan pattern (real or simulated) needs.
- Docs follow `modele`'s Obsidian style: flat `docs/`, Title Case filenames,
  tag-only frontmatter, `Back to [[Hydrone]].` on every note, wikilinks between
  notes, no callouts.
- `docs/` was restructured from 16 flat, differently-cased files into this
  vault; `LANDING-SITES.md` (902 lines) was split by topic into
  [[Landing Sites]], [[Pad Detector]] and [[Pad Map]] since the detector and
  the map are independently linked from [[Phase 1 Mission]]. No other note was
  split — the rest read better as one piece even over ~400 lines.
- [[Onboarding Notes]] is kept and tagged `outdated` rather than deleted — it
  predates the workspace refactor and says so in its first line.
- [[Rangefinder Sim]] stays in Portuguese, matching the original.
