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
]


def load(name: str) -> LaunchDescription:
    """Import an installed launch file and build its description."""
    path = os.path.join(LAUNCH_DIR, name)
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
