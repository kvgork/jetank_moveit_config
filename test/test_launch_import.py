"""
Import + structural tests for the jetank_moveit_config launch files.

The package ships three launch files (``demo``, ``moveit_bringup``,
``moveit_sim``). These tests load each launch module from disk, confirm it
exposes a callable ``generate_launch_description``, and -- when the ROS
overlay is available -- assert the real call returns a
``launch.LaunchDescription`` declaring the expected launch arguments.

``moveit_configs_utils`` is imported lazily inside the launch files (only
when ``OpaqueFunction``/``launch_setup`` runs), so building the top-level
``LaunchDescription`` never needs MoveIt on the path. If a sibling package
share dir is missing (bare env), the real-call assertions are skipped and
the test still verifies the module imports and the entry point is callable.
"""

import importlib.util
import os

import pytest

from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


_LAUNCH_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'launch'
)

# (filename, module name, expected declared launch-argument names)
_LAUNCH_FILES = [
    ('demo.launch.py', 'jetank_demo_launch',
     {'use_sim_time', 'use_rviz', 'hardware'}),
    ('moveit_bringup.launch.py', 'jetank_moveit_bringup_launch',
     {'use_sim_time', 'use_rviz', 'hardware'}),
    ('moveit_sim.launch.py', 'jetank_moveit_sim_launch',
     {'use_rviz', 'headless', 'start_gazebo', 'rviz_config'}),
]


def _load(filename, module_name):
    path = os.path.join(_LAUNCH_DIR, filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('filename,module_name,expected_args', _LAUNCH_FILES)
def test_launch_module_exposes_entry_point(filename, module_name, expected_args):
    """Each launch module imports and exposes callable generate_launch_description."""
    module = _load(filename, module_name)
    assert callable(getattr(module, 'generate_launch_description', None))


@pytest.mark.parametrize('filename,module_name,expected_args', _LAUNCH_FILES)
def test_generate_launch_description(filename, module_name, expected_args):
    """generate_launch_description() returns a LaunchDescription with expected args."""
    module = _load(filename, module_name)
    try:
        launch_description = module.generate_launch_description()
    except PackageNotFoundError:
        pytest.skip('ROS overlay / sibling packages not on AMENT_PREFIX_PATH')

    assert isinstance(launch_description, LaunchDescription)

    declared = {
        action.name
        for action in launch_description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert declared == expected_args
