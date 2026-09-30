# Vendored FAST-LIO (ROS2 branch)

- Upstream: https://github.com/hku-mars/FAST_LIO, branch `ROS2`
- Commit: a4743b095409588842a5b30ddfa27e29d2f99164 (2025-01-15), ikd-Tree submodule included
- License: GPL-2.0 (see LICENSE). Kept untouched.
- Local changes: `doc/` (128 MB of gifs) dropped. The sim config lives in `hydrone_lio/config/fast_lio_mid360_sim.yaml`, not here, so this tree stays close to upstream.
- `src/laserMapping.cpp` `publish_odometry`: fills `twist.twist.linear` with the filter velocity **in camera_init (world) axes** (upstream leaves it empty). `hydrone_lio/lio_odom_adapter` depends on that.

Why this core: `docs/LIO Selection.md`.
