#!/usr/bin/env python3
"""
Pins which estimator the aircraft's life depends on.

With GPS disabled, whatever is published on /zed/zed_node/odom is relayed by
vision_odom_bridge as VISION_POSITION_ESTIMATE and *becomes* the EKF's position.
Getting this wiring wrong does not produce an error message — it produces a
vehicle that flies away.

That is not hypothetical here. Measured on the CompetionMap superflat
(2026-08-17): visual_odometry_node logged 302 starved frames (4-11 feature
matches against a 12 minimum, because the forward camera stares at bare ground)
and accumulated 0.39 m and 8.6 deg of error in 78 s **while the drone had not
moved a centimetre** — ground truth read 0.000 for all 121 samples. Arming on
that estimate, GUIDED chased a position error that did not exist until the
vehicle had travelled 26 m and flipped:

    AP: Crash: Disarming: AngErr=165>30, Accel=1.0<3.0

Hence odom_source — which defaults to the real VO, because flying on truth
would make a green simulator run prove nothing about the drone. Ground truth is
opt-in, for telling an autonomy bug apart from a localization one.

Hence also this test.

Run inside the stack container, with the workspace built:

    docker run --rm -v $PWD:/repo -w /repo <image> bash -c \
      '. /ws/install/setup.sh && python3 -m pytest \
       src/hydrone_bringup/test/test_odom_source.py -q'
"""

import importlib.util
import os

import pytest

from ament_index_python.packages import get_package_share_directory
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


# The topic the FCU actually navigates on.
FLIGHT_TOPIC = "/zed/zed_node/odom"


def _load_sources_sim():
    """Import the installed sources_sim.launch.py as a module.

    By path, because 'sources_sim.launch.py' is not a legal module name.
    """
    path = os.path.join(get_package_share_directory("hydrone_bringup"),
                        "launch", "sources_sim.launch.py")
    spec = importlib.util.spec_from_file_location("sources_sim", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def wiring(mode: str):
    """Resolve odom_wiring() for a given odom_source into plain strings."""
    module = _load_sources_sim()
    ctx = LaunchContext()
    ctx.launch_configurations["odom_source"] = mode
    subs = module.odom_wiring(LaunchConfiguration("odom_source"))
    mimic_odom, mimic_tf, vo_odom, vo_tf = (s.perform(ctx) for s in subs)
    return {
        "mimic_odom": mimic_odom, "mimic_tf": mimic_tf,
        "vo_odom": vo_odom, "vo_tf": vo_tf,
    }


@pytest.mark.parametrize("mode", ["ground_truth", "vo"])
def test_exactly_one_source_flies_the_vehicle(mode):
    """Someone must own the flight topic, and only one someone."""
    w = wiring(mode)
    owners = [k for k in ("mimic_odom", "vo_odom") if w[k] == FLIGHT_TOPIC]
    assert len(owners) == 1, f"{mode}: {len(owners)} nodes own {FLIGHT_TOPIC}"


@pytest.mark.parametrize("mode", ["ground_truth", "vo"])
def test_the_two_sources_never_collide(mode):
    """The one not flying must be parked on its own topic, not shouting over
    the other — two publishers on one odom topic is an interleaved pose."""
    w = wiring(mode)
    assert w["mimic_odom"] != w["vo_odom"]


@pytest.mark.parametrize("mode", ["ground_truth", "vo"])
def test_exactly_one_tf_broadcaster(mode):
    """Two broadcasters of odom->base_link is a corrupt TF tree; zero breaks
    every consumer that looks the transform up."""
    w = wiring(mode)
    assert {w["mimic_tf"], w["vo_tf"]} == {"true", "false"}


@pytest.mark.parametrize("mode", ["ground_truth", "vo"])
def test_the_tf_owner_is_the_one_flying(mode):
    """The node providing the pose must be the one publishing the transform,
    or TF describes a different vehicle than the EKF is flying."""
    w = wiring(mode)
    assert (w["mimic_tf"] == "true") == (w["mimic_odom"] == FLIGHT_TOPIC)
    assert (w["vo_tf"] == "true") == (w["vo_odom"] == FLIGHT_TOPIC)


def test_vo_mode_flies_on_the_real_vo():
    """The default, and the only setting under which a run means anything: the
    drone will fly on the ZED SDK's VIO, so the simulator has to fly on an
    estimator too."""
    w = wiring("vo")
    assert w["vo_odom"] == FLIGHT_TOPIC
    assert w["mimic_odom"] == "/zed/zed_node/odom_GT"


def test_ground_truth_mode_flies_on_ground_truth():
    """The debugging tool: isolates an autonomy bug from a localization one."""
    w = wiring("ground_truth")
    assert w["mimic_odom"] == FLIGHT_TOPIC
    assert w["vo_odom"] == "/zed/zed_node/odom_VO"


def test_ground_truth_is_opt_in_and_never_reached_by_accident():
    """A typo must never silently hand the vehicle perfect localization.

    That is the dangerous direction here: flying on truth does not crash, it
    quietly succeeds, and a green run then proves nothing about the real drone —
    which has no ground truth at all. So ONLY the exact string selects it, and
    everything else falls through to the honest estimator.
    """
    for typo in ("GROUND_TRUTH", "ground truth", "groundtruth", "gt", "", "true"):
        w = wiring(typo)
        assert w["mimic_odom"] != FLIGHT_TOPIC, (
            f"{typo!r} silently flew the vehicle on ground truth")
        assert w["vo_odom"] == FLIGHT_TOPIC


@pytest.mark.parametrize("mode", ["ground_truth", "vo"])
def test_publish_tf_survives_as_a_real_bool(mode):
    """The substitution produces the STRING 'true'/'false'; a bool ROS parameter
    given a string is a type error at node start, so ParameterValue has to
    coerce it. This is the step that would break the launch, not the logic."""
    module = _load_sources_sim()
    ctx = LaunchContext()
    ctx.launch_configurations["odom_source"] = mode
    _, mimic_tf, _, vo_tf = module.odom_wiring(LaunchConfiguration("odom_source"))
    for sub in (mimic_tf, vo_tf):
        value = ParameterValue(sub, value_type=bool).evaluate(ctx)
        assert isinstance(value, bool), f"{mode}: got {type(value).__name__}"


def test_the_launch_file_still_builds_in_both_modes():
    """Guards against the helper drifting away from its caller."""
    module = _load_sources_sim()
    for mode in ("ground_truth", "vo"):
        ctx = LaunchContext()
        ctx.launch_configurations["odom_source"] = mode
        assert module.generate_launch_description() is not None
