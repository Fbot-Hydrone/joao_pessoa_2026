"""datum: pin the EKF z to the floor while disarmed.

    python3 -m pytest src/hydrone_localization/test/test_datum.py -q
"""

from hydrone_localization.datum import DatumCal, DatumParams


class FakeEkf:
    """z = fed_range - T, following a change with a first-order lag."""

    def __init__(self, T, tau=3.0):
        self.T, self.tau, self.z = T, tau, None

    def step(self, fed, dt):
        target = fed - self.T
        if self.z is None:
            self.z = target
        self.z += (target - self.z) * min(1.0, dt / self.tau)
        return self.z


def run(cal, ekf, h, seconds, armed=False, follow=True, t0=0.0):
    dt, t = 0.05, t0
    while t < t0 + seconds:
        fed = h + cal.bias              # range*cos + surface + bias
        z = ekf.step(fed if follow else h, dt)
        cal.update(t, h, z, armed)
        t += dt
    return t


def test_bench_case_is_pinned_in_one_step():
    # 2026-11-18 bench: base 0.5 m, raw 0.18, T = 1.14 -> z was -0.46
    cal, ekf = DatumCal(), FakeEkf(T=1.14)
    h = 0.5 + 0.18
    run(cal, ekf, h, 40.0)
    assert abs(ekf.z - (h - 0.17)) < 0.03          # 0.51: gear on the base
    assert len(cal.adjustments) == 1
    assert cal.disabled is None
    assert cal.ok


def test_waits_for_the_ekf_to_come_back_from_the_barometer():
    # z starts far off and walks slowly towards the feed: must not overshoot
    cal, ekf = DatumCal(), FakeEkf(T=0.40, tau=4.0)
    ekf.z = -0.30
    h = 0.68
    run(cal, ekf, h, 60.0)
    assert abs(ekf.z - (h - 0.17)) < 0.03
    assert len(cal.adjustments) <= 2 and cal.disabled is None


def test_frozen_while_armed():
    cal, ekf = DatumCal(), FakeEkf(T=1.14)
    run(cal, ekf, 0.68, 40.0, armed=True)
    assert cal.bias == 0.0
    assert not cal.ok                   # never judged: not confirmed


def test_no_change_when_moving():
    cal, ekf = DatumCal(), FakeEkf(T=1.14)
    dt, t = 0.05, 0.0
    while t < 40.0:
        h = 0.68 + 0.2 * ((t * 3) % 1.0)          # carried by hand
        cal.update(t, h, ekf.step(h + cal.bias, dt), False)
        t += dt
    assert cal.bias == 0.0


def test_already_right_is_left_alone():
    cal, ekf = DatumCal(), FakeEkf(T=0.17)
    run(cal, ekf, 0.18, 40.0)
    assert cal.adjustments == []


def test_gives_up_when_the_ekf_does_not_follow():
    # e.g. the EKF is on the barometer: the bias would wind up forever
    cal, ekf = DatumCal(), FakeEkf(T=1.14)
    run(cal, ekf, 0.68, 120.0, follow=False)
    assert cal.disabled and "does not follow" in cal.disabled
    assert len(cal.adjustments) <= DatumParams().max_misses + 1


def test_refuses_absurd_bias():
    cal, ekf = DatumCal(), FakeEkf(T=5.0)
    run(cal, ekf, 0.68, 30.0)
    assert cal.disabled and cal.bias == 0.0


def test_small_hunting_around_the_target_never_gives_up():
    """Bench: errors of -0.04 / +0.05 after the big correction must not count
    as the EKF ignoring the feed (that would block arming for nothing)."""
    cal = DatumCal()
    import itertools
    noise = itertools.cycle([0.045, -0.045])
    t, h, z_off = 0.0, 0.68, -0.81 - 0.17    # z = h + bias + z_off
    while t < 200.0:
        if int(t / 9.0) != int((t - 0.05) / 9.0):
            z_off_now = z_off + next(noise)    # EKF settles slightly off
        else:
            z_off_now = locals().get("z_off_now", z_off)
        cal.update(t, h, h + cal.bias + z_off_now, False)
        t += 0.05
    assert cal.disabled is None
