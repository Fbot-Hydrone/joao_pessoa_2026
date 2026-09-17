---
tags: [hydrone, phase4, plan]
---
# RGB Camera Enhancement

Back to [[Hydrone]]. Future roles for the RGB camera Phase 4 does not have yet.
It is a complement to the lidar pipeline, not a replacement — see
[[Phase 4 Pipeline]] for where it sits in the roadmap.

## Why complement, not replacement

The arena is texture-poor: ORB found only 46 keypoints on a whole frame in
sim (see [[Phase 1 Mission#Why the search is a turn, not a pattern]], measured
on the Phase 1 arena — Phase 4's arena is expected to be similar). A
vision-only pipeline starves in exactly the conditions this arena presents.
The lidar is the primary sense for Phase 4; RGB earns its place by covering
cases where lidar itself is degenerate, not by competing with it.

## Planned roles

1. **Visual features where lidar is degenerate.** Corridors and open fields
   (see [[Degraded Sensing]]) underconstrain lidar-only registration along one
   axis; visual features on walls/structure, where present, can constrain
   exactly that axis.
2. **LIVO-style fusion.** FAST-LIVO2 (or an equivalent lidar-inertial-visual
   odometry core) is the expected direction if/when RGB is fused directly into
   the odometry core, rather than bolted on as a side channel. Not yet
   evaluated against the arena's texture — see [[LIO Selection]] for the
   lidar-only core decision this would extend.
3. **Loop closure / place recognition.** Feeds directly into
   [[Degraded Sensing]]'s "revisit known places and compare" check, and into keeping the
   [[Persistent Map]] consistent over a long flight.
4. **Lying-lidar detection.** An independent-of-lidar motion/place estimate is
   exactly the kind of second opinion [[Degraded Sensing]]'s cross-checking
   needs — when it disagrees with the lidar, it is evidence (not proof) that
   the lidar is the one that is wrong.

## What is not planned

Replacing the Livox Mid-360 as the primary sensor, or running a vision-only
odometry mode as anything other than a fallback rung (see
[[Sensor Fallbacks]] level 2) — the texture-poor arena makes that a worse bet than
lidar in normal conditions.

## Status

Not implemented. No RGB camera is mounted on the Kopis X8 yet. This note
exists to fix the intended roles before the hardware and the fusion approach
are chosen, so the choice can be made against these requirements rather than
against whatever camera happens to be on hand.
