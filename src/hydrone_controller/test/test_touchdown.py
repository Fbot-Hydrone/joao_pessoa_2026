"""TouchdownDetector: ported from phase1_mission_node, same semantics."""
from hydrone_controller.touchdown import TouchdownDetector


def feed(d, samples):
    out = False
    for t, z in samples:
        out = d.update(t, z)
    return out


def test_a_full_still_window_after_a_descent_is_touchdown():
    d = TouchdownDetector(settle_s=2.0, still_tol_m=0.05, min_descent_m=0.3)
    d.reset(2.5)
    assert feed(d, [(t * 0.1, 0.83) for t in range(25)])


def test_still_but_never_descended_is_a_hover_not_a_landing():
    d = TouchdownDetector(settle_s=2.0, still_tol_m=0.05, min_descent_m=0.3)
    d.reset(2.5)
    assert not feed(d, [(t * 0.1, 2.45) for t in range(25)])


def test_entering_land_never_reads_as_already_stopped():
    d = TouchdownDetector(settle_s=2.0)
    d.reset(2.5)
    assert not d.update(0.0, 0.5)


def test_still_descending_is_not_touchdown():
    d = TouchdownDetector(settle_s=2.0, still_tol_m=0.05)
    d.reset(2.5)
    assert not feed(d, [(t * 0.1, 2.5 - t * 0.05) for t in range(25)])
