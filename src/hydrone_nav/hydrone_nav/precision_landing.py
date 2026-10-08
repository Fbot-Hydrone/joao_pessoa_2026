"""precision_landing — the geometry of "am I over the base?". ROS-free.

What the confirmation hover over a base needs, pulled out of the mission so any
phase that lands on something can reuse it. Every OPTIONAL behaviour is
switched by a launch argument of the phase that uses it:

  centre_on_pad          visual servo nudges the vehicle over the pad
                         (hydrone_nav.servo). OFF in phase 1: with several
                         bases in frame it chased the neighbour (2026-10-01).
  land_centre_max_cm     veto: refuse to land while the pad sits more than
                         this many cm off centre. <= 0 is OFF.
  trust_map_observations land on the MAP position when the belly cannot
                         confirm but the map has this many looks (glare on
                         tall bases). <= 0 is OFF.
  belly_offset_xy        where the belly camera sits from the vehicle centre.

Frames: image +u right, +v down. Body x forward, y right (nadir camera).
"""

import math


def height_over_pad(vehicle_z, pad_top=None, ground_z=0.0, floor=0.2):
    """Camera height above the surface being judged, never below `floor`.

    The pad's own measured top when known, else the arena floor — the pixel
    to metre scale depends on the distance to the SURFACE, not the altitude.
    """
    top = ground_z if pad_top is None else pad_top
    return max(vehicle_z - top, floor)


def target_uv(servo_uv, belly_offset_xy, fx, height):
    """Where the pad must sit in the image for the VEHICLE to be over it.

    Equal to `servo_uv` (the optical centre) when the camera is on the axis.
    The shift is computed at the CURRENT height: a fixed offset in metres
    subtends fewer pixels as the vehicle comes down.
    """
    u0, v0 = servo_uv
    ox, oy = belly_offset_xy
    if (ox == 0.0 and oy == 0.0) or fx <= 0.0 or height <= 0.0:
        return u0, v0
    px_per_m = fx / height
    # A camera AHEAD of the centre must see the pad BEHIND image centre.
    return u0 - oy * px_per_m, v0 + ox * px_per_m


def offset_px(det_uv, tgt_uv):
    return math.hypot(det_uv[0] - tgt_uv[0], det_uv[1] - tgt_uv[1])


def offset_cm(off_px, fx, height):
    """Pinhole: pixels to centimetres on the surface. None if fx unknown."""
    if off_px is None or fx is None or fx <= 0.0:
        return None
    return off_px * (height / fx) * 100.0


def body_to_world(step_xy, yaw):
    """Rotate a body-frame (dx, dy) by the vehicle yaw into the world."""
    dx, dy = step_xy
    return (dx * math.cos(yaw) - dy * math.sin(yaw),
            dx * math.sin(yaw) + dy * math.cos(yaw))
