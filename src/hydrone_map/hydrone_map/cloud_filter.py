"""cloud_filter — turning a depth camera's cloud into points worth mapping.

Plain numpy: no rclpy, no message imports. A PointCloud2 is read by attribute
(`fields`, `data`, `width`, `height`, `point_step`), so a test can drive this
with a stub and two nodes can share it without either importing the other.

Why this is a library and not a method
--------------------------------------
Two maps want the same points from the same cloud: feature_map_node, which
accumulates them into a voxel grid, and whatever feeds an occupancy map. The
filtering is the part that took measurement to get right, and it must not be
reimplemented per consumer — a second, slightly different rejection rule is a
second, slightly different map.

The flying-pixel problem
------------------------
A pixel that lands on a silhouette returns neither the foreground nor the
background but a blend of the two. MEASURED on a live sim frame (2026-08-20),
drone on the ground 4.86 m from the maze wall: the depth under edge pixels
ranged 5.36..18.07 m. Those points are not noise around the wall, they are
points in empty space.

In a voxel grid a flying pixel is one wrong cell. In an OCCUPANCY map it is
much worse: the ray traced from the camera to a phantom point at 18 m carves
everything it passes through as free — including the real wall at 4.86 m. A
planner then reads a doorway where there is masonry. That is why `edge_mask`
is not optional for anything doing ray casting.
"""

import cv2
import numpy as np

# sensor_msgs/PointField.FLOAT32. Spelled out rather than imported so this
# module stays free of ROS.
FLOAT32 = 7


def layout_problem(msg) -> str | None:
    """Why this cloud cannot be read, or None if it can.

    Deliberately narrow: FLOAT32 x/y/z at 4-byte-aligned offsets in a
    4-byte-aligned point. That is what the ZED publishes (x, y, z, rgb — four
    floats, point_step 16) and what zed_mimic_node publishes. A cloud shaped
    otherwise is one we have not been told about, and guessing at it would put
    silently wrong geometry into a map.
    """
    fields = {f.name: f for f in msg.fields}
    try:
        xyz = [fields[n] for n in ("x", "y", "z")]
    except KeyError:
        return "has no x/y/z fields"
    if any(f.datatype != FLOAT32 or f.count != 1 or f.offset % 4
           for f in xyz) or msg.point_step % 4:
        return ("is not float32 x/y/z on a 4-byte grid "
                f"(point_step {msg.point_step})")
    return None


def xyz_view(msg) -> np.ndarray | None:
    """The cloud's x/y/z as an (H, W, 3) float32 view, or None if truncated.

    Call layout_problem() first; this assumes the layout is one we support.
    """
    fields = {f.name: f for f in msg.fields}
    cols = [fields[n].offset // 4 for n in ("x", "y", "z")]

    floats = np.frombuffer(msg.data, dtype=np.float32)
    stride = msg.point_step // 4
    n = msg.width * msg.height
    if floats.size < n * stride:
        return None
    pts = floats[:n * stride].reshape(n, stride)[:, cols]
    return pts.reshape(msg.height, msg.width, 3)


def edge_mask(rng: np.ndarray, valid: np.ndarray, max_edge_step: float,
              reject_hole_borders: bool = False) -> np.ndarray:
    """True where range is locally smooth enough to trust.

    A 3x3 min and max filter bracket each pixel's neighbourhood; a spread wider
    than `max_edge_step` means the pixel straddles a silhouette and its value
    is a foreground/background blend rather than a surface.

    Invalid neighbours are pushed to sentinels that the min/max filters then
    IGNORE: +1e6 is above every valid range, so the erode never picks it, and
    -1e6 is below every one, so the dilate never picks it. A pixel bordering a
    hole is therefore judged only on its valid neighbours, and survives.

    `reject_hole_borders` flips those sentinels, so a pixel touching a hole
    fails and the wall's outline is shaved off. It is OFF by default because
    that is the behaviour feature_map_node has been mapping with, and turning
    it on there would silently change the map. It is ON for occupancy mapping,
    where the sky border is the worst place to be wrong: a ray to a
    foreground/background blend carves free space straight through the outline
    of the wall it belongs to.

    NOTE (2026-08-26): the default contradicts what feature_map_node's
    docstring claimed before this code moved here — it said a pixel next to a
    hole "also fails". It never did. The claim is now the opt-in.
    """
    kernel = np.ones((3, 3), np.uint8)
    # Inert sentinels (+1e6 never wins a min, -1e6 never wins a max) leave a
    # hole's neighbours judged only on their valid neighbours; swapping them
    # makes any hole in the 3x3 blow the spread wide open.
    miss_lo, miss_hi = ((-1e6, 1e6) if reject_hole_borders else (1e6, -1e6))
    lo = np.where(valid, rng, np.float32(miss_lo))
    hi = np.where(valid, rng, np.float32(miss_hi))
    local_min = cv2.erode(lo, kernel)
    local_max = cv2.dilate(hi, kernel)
    return (local_max - local_min) <= max_edge_step


def sample(pts: np.ndarray, *, min_depth: float, max_depth: float,
           max_edge_step: float, stride: int,
           reject_hole_borders: bool = False) -> tuple[np.ndarray | None, bool]:
    """(H, W, 3) camera-frame cloud -> (Mx3 points, was_unorganized).

    Drops holes and silhouettes, then thins. `points` is None when nothing
    survives. `was_unorganized` says the cloud arrived flat (height 1), so
    flying-pixel rejection could not run — the caller decides whether to warn.
    """
    if pts is None or pts.size == 0:
        return None, False
    h, _w, _ = pts.shape

    # Range, not one axis: the cloud's frame convention is the wrapper's
    # business, and the distance from the camera is the same number in any of
    # them. NaN (no return) fails every comparison, which is the answer.
    rng = np.sqrt((pts.astype(np.float32) ** 2).sum(axis=2))
    with np.errstate(invalid="ignore"):
        valid = np.isfinite(rng) & (rng >= min_depth) & (rng <= max_depth)
    if not valid.any():
        return None, False

    unorganized = h <= 1
    if not unorganized:
        keep = valid & edge_mask(rng, valid, max_edge_step,
                                 reject_hole_borders)
    else:
        # An unorganized cloud has no neighbours to compare against, so the
        # flying pixels stay in. Usable, but visibly noisier — the real wrapper
        # publishes an organized cloud, so this is a fallback for a cloud that
        # has been through a filter that flattened it.
        keep = valid

    # Subsample AFTER filtering: the edge test needs full-resolution neighbours
    # to see a discontinuity at all. On an organized cloud this takes one pixel
    # in stride^2; on a flat one there is a single axis to stride along, so it
    # thins by stride. Both are just density.
    keep = keep[::stride, ::stride]
    if not keep.any():
        return None, unorganized
    return pts[::stride, ::stride][keep].astype(np.float64), unorganized
