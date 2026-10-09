# SPDX-License-Identifier: Apache-2.0
"""Resolve umrr96_viz.launch.py like ros2 launch does, without starting any process."""

from pathlib import Path
import tempfile

from launch import LaunchContext
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import pytest

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'umrr96_viz.launch.py'
SENSOR_SIDE = {'smartmicro_radar_node_exe', 'smartmicro_radar_readback_node',
               'umrr96_processing'}


@pytest.fixture
def resolve(tmp_path, monkeypatch):
    """Return a function mapping launch arguments to (context, {executable: node})."""
    # Dictionary parameters become temporary files, as when ros2 launch runs.
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))

    def run(arguments):
        context = LaunchContext()
        nodes = {}

        def visit(entity):
            # Evaluate a node's condition, name and namespace where the launch service
            # would, but record the node instead of executing it.
            if isinstance(entity, ExecuteProcess):
                if entity.condition is None or entity.condition.evaluate(context):
                    entity._perform_substitutions(context)
                    assert entity.node_executable not in nodes, entity.node_executable
                    nodes[entity.node_executable] = entity
                return
            for child in entity.visit(context) or ():
                visit(child)

        visit(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(LAUNCH)),
                                       launch_arguments=arguments.items()))
        return context, nodes
    return run


def test_views_and_rviz_without_any_sensor_side_node(resolve):
    _, nodes = resolve({})
    assert set(nodes) == {'umrr96_views', 'rviz2'}, sorted(nodes)
    assert not SENSOR_SIDE & set(nodes)


def test_namespace_reaches_views_rviz_and_description(resolve):
    context, nodes = resolve({'namespace': 'front', 'publish_description': 'true',
                              'description_sensor_name': 'front_radar'})
    assert set(nodes) == {'umrr96_views', 'rviz2', 'robot_state_publisher'}, sorted(nodes)
    for executable, node in nodes.items():
        assert node.expanded_node_namespace == '/front', executable
    # RViz's relative description topic follows the description's sensor name.
    assert ('umrr96/robot_description', 'front_radar/robot_description') in (
        nodes['rviz2'].expanded_remapping_rules)
    # C52: the scoped include's arguments do not leak into this file.
    for name in ('frame_id', 'sensor_name'):
        assert name not in context.launch_configurations, name
    assert context.launch_configurations['rviz'] == 'true'


@pytest.mark.parametrize('arguments, expected', [
    ({'rviz': 'false'}, {'umrr96_views'}),
    ({'views': 'false'}, {'rviz2'}),
])
def test_views_and_rviz_are_optional(resolve, arguments, expected):
    _, nodes = resolve(arguments)
    assert set(nodes) == expected, sorted(nodes)
