# Vendored Super-LIO (ROS 2 branch)

- Upstream: https://github.com/Liansheng-Wang/Super-LIO, branch `ros2`
- Commit: f89f48d ("fix: relocation issue, fixes #29")
- Paper: Super-LIO, IEEE RA-L 2026
- License: GPL-3.0 (see LICENSE). Kept untouched.
- What is here: the two ROS 2 packages, `basic` and `super_lio`, plus LICENSE.
  Upstream's `docs/` (images) is dropped. No source changes.
- The Kopis configs live in `hydrone_lio/config/superlio_mid360_real_{fine,coarse}.yaml`,
  not in `super_lio/config/`, so this tree stays as upstream ships it.
- `hydrone_lio/superlio_bridge` republishes its output (`/lio/odom`,
  `/lio/cloud_world`, gravity-levelled `world`) as FAST-LIO's `/Odometry` and
  `/cloud_registered` in `camera_init`, so the rest of the stack is unchanged.
- Careful with upstream configs: their `lio.output.robot: true` makes Super-LIO
  publish `/mavros/vision_pose/pose` itself. Ours set it false.

Why this core: on the Jetson Nano, replaying the same Mid-360 bag at 1x, alone
(2026-11-18): FAST-LIO kept up only at 0.4 m voxels (53% of a core); at 0.2 m
its lag grew to ~1.5 s and at 0.1 m it dropped 40% of the scans and diverged.
Super-LIO kept up at 0.4/0.5 m (29%), 0.2 m (37%) and 0.1 m (51%), with 3-4 cm
loop error. Its map is a hash grid of voxels holding one averaged point per
sub-voxel (OctVoxMap) and its neighbour search a fixed voxel order, where
FAST-LIO's ikd-tree gets deeper and rebuilds as the map grows.
