---
tags: [hydrone, zed, tf, hardware]
---
# ZED Mount

Back to [[Hydrone]]. How the forward ZED's position and tilt reach the stack, and
what to change when the camera moves on the airframe.

## The one knob

The mount is `xyz` (metres, base_link) plus `rpy_deg` = [roll, pitch, yaw] in
degrees. **+pitch tips the lens down**, the same as BiguaSim's sensor `rotation`
and the belly camera ([[Landing Sites]]). An upside-down camera is roll 180.

| where | position | rotation |
|---|---|---|
| sim | RGBCamera `location` in `config-<agent>.yaml` | RGBCamera `rotation` in the same block |
| real drone | `zed_offset_xyz:=` on `sources_real.launch.py` | `zed_rpy_deg:=` |

In sim, a single YAML edit moves both the rendered camera and the TF. The math
lives in `hydrone_bringup/camera_mount.py` (no ROS, tested by
`test/test_camera_mount.py`, for tilts 0 to 45 degrees, upright and inverted).

> On `main` (branch `zed2i-mount`, 2026-11-05) only the real-drone half is in:
> `zed_sdk_node` and `sources_real`. The sim half (`zed_mimic_node`,
> `sources_sim` reading the YAML rotation) is still on `phase4-lio`.

## What follows the mount by itself

Both ZED sources publish the same static chain:

```
base_link -> zed_camera_link              mount (xyz + rpy)
          -> zed_left_camera_frame        + half baseline in y (real), identity (sim)
          -> zed_left_camera_optical_frame
```

All of these read the mount from TF, so they need no change for a tilt:
`pad_detector_node`, `feature_map_node`, and `map_odom_node`.

**Point cloud.** Each consumer transforms the cloud through TF using its
`frame_id`. On the real ZED, the SDK gives XYZ in the left lens frame with
camera_link axes (X forward, Z up), *not* optical, so it is stamped
`zed_left_camera_frame`. Until 2026-11-05 it was stamped as the optical frame,
which turned every real cloud by 90 degrees.

## The real drone: t15 support, upside down

`drones/supports/parts/zed_mount.py` (t15 block) holds the ZED 2i **upside down
and 15 degrees nose-down**. From the CAD, the body centre is 8 cm ahead of the
frame centre and 3.7 cm below the bottom plate. The launch defaults are
`zed_offset_xyz:=[0.080, 0.0, -0.037]` and `zed_rpy_deg:=[180.0, 15.0, 0.0]`.
For the t00 and t25 blocks, change the pitch.

`zed_sdk_node` handles the inversion itself:

- The SDK image flip stays **off**. With the flip on, the SDK prints "IMU Fusion
  is not handled in FLIP mode" and tracking goes visual-only and is no longer
  levelled by gravity.
- With the flip off, IMU fusion keeps working on the inverted camera. On the
  bench (2026-11-05), the tracked world's up direction matched the
  accelerometer's within 0.3 degrees once it had settled.
- `upright_images` (default true) rotates the image and depth by 180 degrees
  when the mount is upside down. It moves cx/cy and turns the optical frame
  180 degrees about its Z axis to match. The cloud is not rotated.
- **Odometry.** The SDK pose is the left lens'. The mount and the lever arm are
  removed, giving `T_world_base = T_world_lens · T_lens_base`. Without an IMU
  (first-gen ZED), the world is the lens' start pose, and the pose is
  re-expressed from the body's start pose instead.
- **Settling time.** The IMU-levelled world takes **3 to 6 s** to settle after
  start. One run was 10 degrees off at 3 s. Do not take off in the first
  seconds.

**Bench check, 2026-11-05.** The drone sat on a desk. The ZED accelerometer
measured the lens 11.7 degrees down, against 15 degrees by design, and
base_link came out 3.5 degrees nose-up. That is consistent with the bench
rather than the support. Position at rest equals the lever arm with its sign
flipped, as expected.

## Not done: sim visual odometry

`visual_odometry_node` still uses a constant `R_BASE_FROM_OPTICAL` and no lever
arm ([[ZED Visual Odometry]]), so its odometry is wrong for a tilted sim camera.
`sources_sim` logs a WARNING when the YAML rotation is non-zero and VO is
flying the drone. Use `odom_source:=ground_truth` for tilted-camera runs.
The fix is to look up `base_link -> zed_left_camera_optical_frame` once and
use it in place of the constant.
