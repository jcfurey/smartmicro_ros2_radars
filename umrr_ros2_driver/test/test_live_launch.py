# SPDX-License-Identifier: Apache-2.0
"""Resolve umrr96_live.launch.py like ros2 launch does, without starting any process."""

from pathlib import Path

from launch import LaunchContext
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import pytest

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'umrr96_live.launch.py'


def resolve(arguments):
    """Return (context, [(package, executable)]) of the processes a launch would start."""
    context = LaunchContext()
    processes = []

    def visit(entity):
        # Node is an ExecuteProcess: evaluate its condition where the launch service would,
        # but record it instead of executing it.
        if isinstance(entity, ExecuteProcess):
            if entity.condition is None or entity.condition.evaluate(context):
                processes.append((entity.node_package, entity.node_executable))
            return
        for child in entity.visit(context) or ():
            visit(child)

    # ros2 launch passes command-line arguments through an unscoped include, as here.
    visit(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(LAUNCH)),
                                   launch_arguments=arguments.items()))
    return context, processes


@pytest.mark.parametrize('description', ['false', 'true'])
def test_rviz_starts_with_and_without_description(description):
    # C52: the description include's rviz:=false leaked into this file and hid its RViz.
    context, processes = resolve({'publish_description': description})
    assert processes.count(('rviz2', 'rviz2')) == 1, processes  # C37: never a second one.
    assert (('robot_state_publisher', 'robot_state_publisher') in processes) == (
        description == 'true'), processes
    assert ('umrr_ros2_driver', 'smartmicro_radar_node_exe') in processes
    assert context.launch_configurations['rviz'] == 'true'


def test_headless_with_description():
    _, processes = resolve({'publish_description': 'true', 'rviz': 'false'})
    assert ('rviz2', 'rviz2') not in processes, processes
    assert ('robot_state_publisher', 'robot_state_publisher') in processes


def test_description_arguments_stay_in_the_include():
    context, _ = resolve({'publish_description': 'true', 'description_sensor_name': 'front'})
    for name in ('frame_id', 'sensor_name', 'measurement_xyz'):
        assert name not in context.launch_configurations, name
