"""hydrone_nav.route is plain Python, so it is tested with plain fakes.

The same rules are exercised through the mission node in
hydrone_mission/test/test_phase1_mission.py. These tests pin the library
itself, so a phase that reuses it does not depend on a mission's tests.

    python3 -m pytest src/hydrone_nav/test/test_route.py -q
"""

from types import SimpleNamespace

from hydrone_nav import route


def pad(pad_id, x, y, *, takeoff_base=False, visited=False, observations=3):
    return SimpleNamespace(
        id=pad_id,
        position=SimpleNamespace(x=x, y=y),
        is_takeoff_base=takeoff_base,
        visited=visited,
        observations=observations,
    )


def test_the_takeoff_base_is_never_a_candidate():
    assert not route.is_candidate(pad(1, 2.0, 0.0, takeoff_base=True))


def test_a_visited_pad_is_never_a_candidate():
    assert not route.is_candidate(pad(1, 2.0, 0.0, visited=True))


def test_a_blacklisted_pad_is_never_a_candidate():
    assert not route.is_candidate(pad(7, 2.0, 0.0), blacklist={7})


def test_a_pad_needs_more_than_one_sighting():
    assert not route.is_candidate(pad(1, 2.0, 0.0, observations=1))
    assert route.is_candidate(pad(1, 2.0, 0.0, observations=2))


def test_a_pad_on_top_of_home_is_never_a_candidate():
    assert not route.is_candidate(pad(1, 0.3, -0.2), home=(0.0, 0.0))
    assert route.is_candidate(pad(1, 2.0, 0.0), home=(0.0, 0.0))


def test_the_nearest_eligible_pad_wins():
    pads = [pad(1, 5.0, 0.0), pad(2, 2.0, 0.0), pad(3, 9.0, 0.0)]
    assert route.nearest_candidate(pads, 0.0, 0.0).id == 2


def test_distance_is_measured_from_the_drone_not_the_origin():
    pads = [pad(1, 5.0, 0.0), pad(2, 2.0, 0.0)]
    assert route.nearest_candidate(pads, 6.0, 0.0).id == 1


def test_a_nearer_but_ineligible_pad_is_skipped():
    pads = [pad(1, 1.0, 0.0, visited=True), pad(2, 4.0, 0.0)]
    assert route.nearest_candidate(pads, 0.0, 0.0).id == 2


def test_no_eligible_pad_is_none():
    assert route.nearest_candidate([pad(1, 2.0, 0.0, visited=True)], 0.0, 0.0) is None
    assert route.nearest_candidate([], 0.0, 0.0) is None


def test_takeoff_base_is_read_from_the_map_when_registered():
    pads = [pad(1, 2.0, 0.0), pad(2, -1.0, 3.0, takeoff_base=True)]
    assert route.takeoff_base_xy(pads) == (-1.0, 3.0)


def test_takeoff_base_falls_back_when_the_map_has_none():
    assert route.takeoff_base_xy([pad(1, 2.0, 0.0)], fallback=(9.0, 9.0)) == (9.0, 9.0)


def test_two_sightings_are_enough_to_be_worth_a_leg():
    """This needed three until 2026-08-27, and three cost real bases: two pads
    seen twice at 6.9 m and 5.7 m never got a third look because the search
    turned away, so they were never flown to. The confirmation hover is the
    real filter — a metre up, where the pad is hundreds of pixels across."""
    assert route.is_candidate(pad(1, 2.0, 0.0, observations=2))


def test_a_single_sighting_is_still_not_enough():
    """One frame of blue noise reaches the map."""
    assert not route.is_candidate(pad(1, 2.0, 0.0, observations=1))
