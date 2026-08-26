"""cloud_filter is plain numpy, so it is tested with a stub cloud.

The case that matters is the flying pixel: a point on a silhouette whose depth
is a blend of foreground and background. In an occupancy map that point carves
free space through a real wall, so its rejection is pinned here rather than
left to whatever the mapping node happens to do.

    python3 -m pytest src/hydrone_map/test/test_cloud_filter.py -q
"""

from types import SimpleNamespace

import numpy as np
import pytest

from hydrone_map import cloud_filter

FLOAT32 = cloud_filter.FLOAT32


def field(name, offset, datatype=FLOAT32, count=1):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype,
                           count=count)


def cloud(points_hw3, *, extra_float=True):
    """A PointCloud2-shaped stub: float32 x,y,z (+ a padding float, like rgb)."""
    pts = np.asarray(points_hw3, dtype=np.float32)
    h, w, _ = pts.shape
    n_floats = 4 if extra_float else 3
    buf = np.zeros((h * w, n_floats), dtype=np.float32)
    buf[:, :3] = pts.reshape(h * w, 3)
    return SimpleNamespace(
        fields=[field("x", 0), field("y", 4), field("z", 8)],
        data=buf.tobytes(),
        width=w, height=h, point_step=4 * n_floats,
    )


def wall(h, w, distance):
    """A flat wall straight ahead: z is the range."""
    pts = np.zeros((h, w, 3), dtype=np.float32)
    pts[:, :, 2] = distance
    return pts


DEFAULTS = dict(min_depth=0.4, max_depth=20.0, max_edge_step=0.30, stride=1)


# ── reading the cloud ────────────────────────────────────────────────────────

def test_a_well_formed_cloud_has_no_layout_problem():
    assert cloud_filter.layout_problem(cloud(wall(4, 4, 3.0))) is None


def test_a_cloud_without_xyz_is_refused():
    msg = cloud(wall(4, 4, 3.0))
    msg.fields = [field("intensity", 0)]
    assert "no x/y/z" in cloud_filter.layout_problem(msg)


def test_a_non_float32_cloud_is_refused():
    msg = cloud(wall(4, 4, 3.0))
    msg.fields = [field("x", 0, datatype=8), field("y", 4), field("z", 8)]
    assert "float32" in cloud_filter.layout_problem(msg)


def test_xyz_view_keeps_the_image_shape():
    pts = cloud_filter.xyz_view(cloud(wall(3, 5, 2.0)))
    assert pts.shape == (3, 5, 3)
    assert np.allclose(pts[:, :, 2], 2.0)


def test_a_truncated_cloud_reads_as_none():
    msg = cloud(wall(4, 4, 3.0))
    msg.data = msg.data[: len(msg.data) // 2]
    assert cloud_filter.xyz_view(msg) is None


# ── the depth window ─────────────────────────────────────────────────────────

def test_points_beyond_max_depth_are_dropped():
    pts, _ = cloud_filter.sample(wall(8, 8, 25.0), **DEFAULTS)
    assert pts is None


def test_points_nearer_than_min_depth_are_dropped():
    pts, _ = cloud_filter.sample(wall(8, 8, 0.2), **DEFAULTS)
    assert pts is None


def test_a_plain_wall_survives():
    pts, _ = cloud_filter.sample(wall(8, 8, 4.86), **DEFAULTS)
    assert pts is not None and len(pts) > 0
    assert np.allclose(pts[:, 2], 4.86)


def test_nan_returns_are_dropped():
    w = wall(8, 8, 4.0)
    w[3, 3, :] = np.nan
    pts, _ = cloud_filter.sample(w, **DEFAULTS)
    assert np.isfinite(pts).all()


# ── the flying pixel: the reason this module exists ──────────────────────────

def test_a_flying_pixel_is_rejected():
    """The measured case: wall at 4.86 m, an edge pixel reporting 18.07 m."""
    w = wall(9, 9, 4.86)
    w[4, 4, 2] = 18.07
    pts, _ = cloud_filter.sample(w, **DEFAULTS)
    assert not (pts[:, 2] > 5.0).any(), "flying pixel reached the map"


def test_the_pixels_around_a_flying_pixel_go_too():
    """Its 3x3 neighbourhood straddles the same discontinuity."""
    w = wall(9, 9, 4.86)
    w[4, 4, 2] = 18.07
    pts, _ = cloud_filter.sample(w, **DEFAULTS)
    # 81 pixels, minus the 3x3 block centred on the bad one.
    assert len(pts) == 81 - 9


def test_a_wall_bordering_a_hole_keeps_its_outline():
    """Pins CURRENT behaviour, which is not what the old docstring claimed.

    The sentinels for invalid neighbours are inert (see edge_mask), so the
    column touching the sky is judged only on its valid neighbours and
    survives. Shaving it is arguable — for an occupancy map, quite arguable —
    but it would change what gets mapped, so it is a decision, not a fix.
    """
    w = wall(9, 9, 4.86)
    w[:, 6:, :] = np.nan          # sky beyond the wall's edge
    pts, _ = cloud_filter.sample(w, **DEFAULTS)
    assert len(pts) == 9 * 6      # columns 0..5, the border column included


def test_a_gentle_slope_is_not_an_edge():
    """A ramp within max_edge_step is a surface, not a silhouette."""
    w = wall(9, 9, 4.0)
    w[:, :, 2] += np.linspace(0, 0.4, 9, dtype=np.float32)[None, :]
    pts, _ = cloud_filter.sample(w, **DEFAULTS)
    assert len(pts) == 81


def test_an_unorganized_cloud_says_so_and_keeps_the_flying_pixel():
    flat = wall(1, 81, 4.86)
    flat[0, 40, 2] = 18.07
    pts, unorganized = cloud_filter.sample(flat, **DEFAULTS)
    assert unorganized
    assert (pts[:, 2] > 5.0).any(), "nothing to reject it with, and that is the point"


def test_an_organized_cloud_does_not_claim_to_be_flat():
    _pts, unorganized = cloud_filter.sample(wall(8, 8, 4.0), **DEFAULTS)
    assert not unorganized


# ── thinning ─────────────────────────────────────────────────────────────────

def test_stride_thins_by_its_square_on_an_organized_cloud():
    opts = dict(DEFAULTS, stride=2)
    pts, _ = cloud_filter.sample(wall(8, 8, 4.0), **opts)
    assert len(pts) == 16          # 64 / 2^2


def test_filtering_happens_before_thinning():
    """Otherwise the edge test loses the neighbours that reveal the edge."""
    w = wall(9, 9, 4.86)
    w[4, 4, 2] = 18.07             # a pixel stride=2 would sample
    pts, _ = cloud_filter.sample(w, **dict(DEFAULTS, stride=2))
    assert not (pts[:, 2] > 5.0).any()


def test_an_empty_cloud_is_none():
    assert cloud_filter.sample(np.zeros((0, 0, 3), np.float32), **DEFAULTS) == (None, False)
    assert cloud_filter.sample(None, **DEFAULTS) == (None, False)
