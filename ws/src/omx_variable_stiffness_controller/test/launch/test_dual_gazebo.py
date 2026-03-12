import os
import time

import pytest

import launch
import launch.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_testing.actions import ReadyToTest

from ament_index_python.packages import get_package_share_directory


def generate_test_description():
    pkg_share = get_package_share_directory('omx_variable_stiffness_controller')
    launch_file = os.path.join(pkg_share, 'launch', 'dual_hardware_variable_stiffness.launch.py')

    ld = launch.LaunchDescription()
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'launch_gazebo': 'true',
            'gui': 'false',
            'enable_logger': 'false',
            'enable_live_plot': 'false',
        }.items(),
    ))

    ld.add_action(ReadyToTest())
    return ld


@pytest.mark.launch_test
def test_gazebo_dual_spawn(node):
    """Simple smoke test: ensure Gazebo launch results in robot topics appearing."""
    # Node fixture is provided by launch_testing; use topic list polling
    ok = False
    for _ in range(60):
        try:
            topics = node.get_topic_names_and_types()
            names = [t for (t, _) in topics]
            # look for robot1 variable stiffness topic
            if any('/robot1/robot1_variable_stiffness' in n for n in names):
                ok = True
                break
        except Exception:
            pass
        time.sleep(1.0)

    assert ok, 'Expected robot1 variable stiffness topics to appear in Gazebo launch'
