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
