#!/usr/bin/env python3
"""
A sim wrapper must not re-declare the arguments of the launch file it includes.

This pins a bug that cost a flight session on 2026-08-22. `phase1_sim.launch.py`
mirrored every one of `phase1.launch.py`'s arguments and forwarded them, so:

    DeclareLaunchArgument("takeoff_alt", default_value="1.0")   # wrapper
    ...
    launch_arguments={"takeoff_alt": LaunchConfiguration("takeoff_alt")}

meant the wrapper's default was passed down and **overwrote** whatever
`phase1.launch.py` declared. `takeoff_alt` was edited to 4 m in the file that
documents it; the vehicle climbed to 1 m and nothing warned. Same for
`settle_s`, `max_rotations` and `target_bases` — the mission looked like it was
ignoring its own configuration.

Nothing needed to be mirrored. Launch configurations are inherited by an
included description, so an argument the wrapper never mentions:

  * falls through to the inner file's default when not given, and
  * still arrives from the command line when it is.

That inheritance is the load-bearing claim here, so it was checked against a
running `ros2 launch` rather than reasoned about (2026-08-22, three throwaway
launch files: a wrapper that declares and forwards `alt`, a wrapper that says
nothing, and an inner file that declares `alt` with default `INNER`):

    forwarding wrapper, no CLI argument   -> inner sees WRAPPER_DEFAULT   (the bug)
    silent wrapper,     no CLI argument   -> inner sees INNER_DEFAULT
    silent wrapper,     alt:=FROM_CLI     -> inner sees FROM_CLI

and then end to end on the real files: `phase1_sim.launch.py` with no arguments
brought the mission up at `phase1.launch.py`'s edited defaults (4.0 m, 20 turns,
9 bases), and `phase1_sim.launch.py takeoff_alt:=1.5 target_bases:=2` overrode
them. The tests below pin the structure that makes that hold; they do not
re-derive launch's scoping rules in Python.

Run inside the stack container:

    docker run --rm -v $PWD:/repo -w /repo <image> bash -c \\
      '. /ws/install/setup.sh && python3 -m pytest \\
       src/hydrone_bringup/test/test_launch_arguments.py -q'
"""

import importlib.util
import os

import pytest

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions


LAUNCH_DIR = os.path.join(
    get_package_share_directory("hydrone_bringup"), "launch")

# (sim wrapper, the autonomy layer it includes)
WRAPPER_PAIRS = [
    ("phase1_sim.launch.py", "phase1.launch.py"),
    ("landing_sites_sim.launch.py", "landing_sites.launch.py"),
    # The real-hardware wrapper includes TWO files and must not shadow either.
    ("phase1_real.launch.py", "phase1.launch.py"),
    ("phase1_real.launch.py", "sources_real.launch.py"),
]


def load(name: str) -> LaunchDescription:
    """Import an installed launch file and build its description.

    Reads from the INSTALL tree, not from src/, because that is what
    `ros2 launch` reads. The difference is load-bearing with a dev bind mount:
    --symlink-install creates one symlink per file AT BUILD TIME, so a launch
    file added since the image was built is simply absent here. Skipping says
    that out loud — an unhelpful FileNotFoundError deep in importlib was the
    alternative, and it reads like a broken test rather than a stale image.
    """
    path = os.path.join(LAUNCH_DIR, name)
    if not os.path.exists(path):
        pytest.skip(
            f"{name} is not in the install tree. It is newer than this image: "
            "rebuild (scripts/dev_rebuild.sh, or jetson_up.sh --rebuild) to "
            "cover it.")
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def declared(description: LaunchDescription) -> dict[str, str]:
    """{argument name: default}, for the arguments declared at the top level."""
    context = LaunchContext()
    out = {}
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            default = entity.default_value
            out[entity.name] = (
                perform_substitutions(context, default)
                if default is not None else None)
    return out


@pytest.mark.parametrize("wrapper_name,inner_name", WRAPPER_PAIRS)
def test_the_wrapper_does_not_shadow_the_inner_arguments(wrapper_name,
                                                         inner_name):
    """The regression itself.

    An argument declared in both places means the wrapper's default wins and the
    inner file's is dead. Delete it from the wrapper — inheritance already
    carries both the default and any command-line override.
    """
    shared = sorted(set(declared(load(wrapper_name)))
                    & set(declared(load(inner_name))))
    assert not shared, (
        f"{wrapper_name} re-declares {shared}, which are also declared in "
        f"{inner_name}. The wrapper's defaults will silently override the "
        f"inner file's. Remove them from {wrapper_name}; they reach the "
        "included description by inheritance.")


@pytest.mark.parametrize("wrapper_name,inner_name", WRAPPER_PAIRS)
def test_the_wrapper_forwards_nothing_to_the_autonomy_layer(wrapper_name,
                                                            inner_name):
    """Belt and braces: even a non-shadowing forward re-introduces the coupling.

    Passing `launch_arguments` to the autonomy include is how the shadowing did
    its damage. The wrapper may still forward to `sources_sim.launch.py` — that
    is the half it genuinely owns.
    """
    for entity in load(wrapper_name).entities:
        if not isinstance(entity, IncludeLaunchDescription):
            continue
        source = entity.launch_description_source
        if not isinstance(source, PythonLaunchDescriptionSource):
            continue
        if os.path.basename(source.location) != inner_name:
            continue
        forwarded = [name for name, _ in entity.launch_arguments]
        assert not forwarded, (
            f"{wrapper_name} forwards {forwarded} into {inner_name}. Forwarding "
            "a configuration the wrapper declared overrides the inner file's "
            "default; forwarding one it did not declare is redundant, because "
            "inheritance already carries it.")


# ── Every launch file's parameters must actually evaluate ───────────────────
#
# A second bug, same day, same shape: invisible until something ran it.
# `sources_real.launch.py` declared its list-valued parameters as
#
#     ParameterValue(LaunchConfiguration("down_cam_mount_xyz"),
#                    value_type=list([float]))          # WRONG
#
# `list([float])` is `[float]` — a list holding the *type* float — where launch
# wants the typing generic `List[float]`. `generate_launch_description()` builds
# fine, the file imports fine, and the existing tests above all passed, because
# the type is not looked at until launch EVALUATES the parameter. Only then:
#
#     [ERROR] [launch]: Caught exception in launch (...):
#             Unrecognized data type: [<class 'float'>]
#
# and no node starts at all. It survived every unit test and was found by
# running `sources_real.launch.py` on the drone for the first time — the bench
# validation before it had started the nodes individually, never the launch.
#
# So: walk every launch file, find every Node, and force its parameters through
# the same evaluation launch would do.

LAUNCH_FILES = [
    "phase1.launch.py",
    "phase1_sim.launch.py",
    "phase1_real.launch.py",
    "sources_real.launch.py",
    "sources_sim.launch.py",
    "landing_sites.launch.py",
    "landing_sites_sim.launch.py",
    "hydrone_sim.launch.py",
    "hydrone_bringup.launch.py",
]


def _nodes_of(description: LaunchDescription):
    """Every Node in a description, including inside group/conditional actions."""
    from launch_ros.actions import Node

    seen = []

    def walk(entities):
        for entity in entities:
            if isinstance(entity, Node):
                seen.append(entity)
            # GroupAction and friends hold their children in different
            # attributes; try the two that matter and ignore the rest.
            for attr in ("entities", "_GroupAction__actions"):
                child = getattr(entity, attr, None)
                if child:
                    walk(child)

    walk(description.entities)
    return seen


@pytest.mark.parametrize("launch_file", LAUNCH_FILES)
def test_every_node_parameter_evaluates(launch_file):
    """Force each Node's parameters through launch's own type machinery.

    This is the check that `value_type=list([float])` fails and
    `value_type=List[float]` passes. It does not assert the VALUES — those come
    from arguments and are the subject of the tests above — only that every
    declared type is one launch recognises.
    """
    from launch.utilities import type_utils

    # Some launch files reach for packages that exist only in the simulator
    # image (ardupilot_sitl, biguasim_*). On the drone's image that is correct
    # rather than broken, so skip instead of failing — the sim container runs
    # the same test and covers them there.
    from ament_index_python.packages import PackageNotFoundError

    context = LaunchContext()
    # Give every declared argument its default, so substitutions resolve.
    try:
        description = load(launch_file)
    except PackageNotFoundError as exc:
        pytest.skip(f"{launch_file} needs a package absent from this image: {exc}")
    for name, default in declared(description).items():
        if default is not None:
            context.launch_configurations[name] = default

    for node in _nodes_of(description):
        for entry in (node._Node__parameters or []):
            if not isinstance(entry, dict):
                continue          # a YAML path; nothing to type-check
            for key, value in entry.items():
                # Node normalises parameter names into tuples of
                # substitutions, so a raw key reprs as
                # "(<TextSubstitution object at 0x...>,)" and names nothing.
                try:
                    key = perform_substitutions(context, list(key))
                except (TypeError, AttributeError):
                    key = str(key)
                value_type = getattr(value, "_ParameterValue__value_type", None)
                if value_type is None:
                    continue      # a plain literal or substitution
                try:
                    type_utils.extract_type(value_type)
                except ValueError as exc:
                    pytest.fail(
                        f"{launch_file}: parameter '{key}' declares "
                        f"value_type={value_type!r}, which launch rejects at "
                        f"RUN time ({exc}). Use the typing generic — "
                        f"List[float], not list([float]) or [float].")
