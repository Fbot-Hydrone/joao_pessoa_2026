"""arena: Figure 7 layout -> map frame, anchored at the takeoff base."""
import pytest

from hydrone_nav.arena import arena_to_map, load_layout


def test_the_takeoff_maps_onto_home():
    assert arena_to_map((1.03, 0.75), (-3.39, 3.0), (1.03, 0.75)) == \
        pytest.approx((-3.39, 3.0))


def test_sim_rotation_matches_the_measured_frame():
    """Sim: map = (y' - 4, 4 - x'). With home where Fig. 7 puts the takeoff,
    pickup 1 (3.21, 1.35) lands at (-2.65, 0.79)."""
    home = (0.75 - 4.0, 4.0 - 1.03)
    assert arena_to_map((3.21, 1.35), home, (1.03, 0.75)) == \
        pytest.approx((1.35 - 4.0, 4.0 - 3.21))


def test_the_default_layout_loads(tmp_path=None):
    import os
    here = os.path.dirname(__file__)
    lay = load_layout(os.path.join(
        here, "..", "..", "hydrone_bringup", "config", "phase2_bases.yaml"))
    assert len(lay.pickup) == 3 and len(lay.delivery) == 3
    assert all(abs(p[2] - 1.6) < 1e-9 for p in lay.pickup)


def _layout(tmp_path, text):
    f = tmp_path / "bases.yaml"
    f.write_text(text)
    return load_layout(str(f))


def test_grid_entries_land_on_the_line_crossings(tmp_path):
    """Figure 7: base A sits on lines 9 and 14 of the 0.5 m grid."""
    lay = _layout(tmp_path, """
takeoff: [1.03, 0.75]
pickup:
  - {grid: [6, 3], z: 1.6, yaw: 45}
delivery:
  - {grid: [9, 14]}
  - {grid: [3, 9], z: 0.5}
  - [4.98, 4.49]
""")
    assert lay.pickup == [pytest.approx((3.0, 1.5, 1.6, 45.0))]
    assert lay.delivery[0] == pytest.approx((4.5, 7.0))
    assert lay.delivery[1] == pytest.approx((1.5, 4.5, 0.5))
    assert lay.delivery[2] == pytest.approx((4.98, 4.49))


def test_grid_by_square_and_shifted_origin(tmp_path):
    lay = _layout(tmp_path, """
grid: {step_m: 0.5, origin_m: [0.1, -0.2], centre: cell}
takeoff: {grid: [2, 1]}
delivery:
  - {grid: [8, 13]}
""")
    assert lay.takeoff == pytest.approx((1.35, 0.55))
    assert lay.delivery[0] == pytest.approx((4.35, 6.55))
