import numpy as np

from hydrone_lio.lio_map_node import key_centers, read_pcd, voxel_keys, write_pcd


def test_voxel_key_roundtrip():
    pts = np.array([[0.01, -3.27, 12.5], [-40.0, 7.77, -0.02]], dtype=np.float32)
    centers = key_centers(voxel_keys(pts, 0.05), 0.05)
    assert np.all(np.abs(centers - pts) <= 0.025 + 1e-5)


def test_nearby_points_share_a_dedup_cell():
    a = voxel_keys(np.array([[1.001, 2.001, 3.001], [1.009, 2.009, 3.009]]), 0.02)
    assert a[0] == a[1]


def test_pcd_roundtrip(tmp_path):
    pts = np.random.default_rng(0).normal(size=(100, 3)).astype(np.float32)
    hits = np.arange(100, dtype=np.float32)
    path = tmp_path / 'x.pcd'
    write_pcd(str(path), pts, {'hits': hits})
    names, data = read_pcd(str(path))
    assert names == ['x', 'y', 'z', 'hits']
    assert np.allclose(data[:, :3], pts) and np.allclose(data[:, 3], hits)
