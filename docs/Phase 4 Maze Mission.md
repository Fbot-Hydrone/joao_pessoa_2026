---
tags: [hydrone, phase4, plan]
---
# Phase 4 Maze Mission

Back to [[Hydrone]]. The Phase 4 mission goal and a minimal approach to it. See
[[Phase 4 Pipeline]] for the odometry/mapping stack this depends on — this is
the stretch goal (step 4) that sits on top of it.

## Goal

To the right of the drone's spawn point there is a small 3D "maze" structure.
The drone must enter it and exit at the other end without colliding, flying on
its own sensing (see [[Phase 4 Pipeline]] — never on ground truth).

## A minimal approach

1. **Take off on LIO.** Requires [[LIO Odometry]] closed-loop and validated —
   nothing here works before that exists.
2. **Approach the entrance.** Fly toward the known (or roughly known) entrance
   heading from spawn; refine against the lidar's own view of the structure as
   it comes into range.
3. **Plan through the voxel map, inflated by drone radius.** Use the
   [[Persistent Map]]'s voxel grid, inflated by the drone's physical radius
   (plus a safety margin) as an obstacle map, and plan a corridor-following or
   simple free-space path through it — this does not need a general-purpose
   planner for a small, mostly-corridor structure.
4. **Exit and land.** Continue the plan through to the far opening, then land
   once clear of the structure.

This deliberately mirrors the "smallest thing that flies a full cycle"
philosophy already used for [[Landing Sites]] and [[Phase 1 Mission]]: no
search, no retry logic, no dynamic replanning beyond following the map — get
one clean traversal working before adding robustness.

## Open questions

- **How narrow is the maze, relative to the Kopis X8's frame + prop guards?**
  Drives how much inflation margin is affordable versus how tight the planned
  path has to hug the walls.
- **Is the structure geometry known ahead of time** (a CAD/measured model), or
  must the drone build the map of it purely from its own first pass? If known,
  a pre-loaded map could seed [[Persistent Map]] and simplify this considerably.
- **What happens on a [[Degraded Sensing]] event inside the maze?** Corridors
  are exactly the degenerate geometry LIO struggles with (see
  [[Degraded Sensing]]), and the maze is the one place in the mission where that matters
  most, with the least room to recover from a bad estimate.
- **Collision margin vs. mission time.** A wide margin is safer but may not fit
  through a tight maze section; this needs a real measurement of the structure
  before it can be tuned rather than guessed.
- **Does RGB help here specifically?** The maze's walls may be more
  textured than the open arena — worth checking once
  [[RGB Camera Enhancement]] exists, since loop closure/relocalization inside a maze is a
  plausible high-value case for it.

## Status

Not started. Depends on [[LIO Odometry]] and [[Persistent Map]] both being
validated first; this is the last step in [[Phase 4 Pipeline]] and explicitly
a stretch goal, not a prerequisite for the rest of Phase 4 to be considered
working.
