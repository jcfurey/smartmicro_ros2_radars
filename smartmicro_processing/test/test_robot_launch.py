# SPDX-License-Identifier: Apache-2.0
"""Resolve umrr96_robot.launch.py like ros2 launch does, without starting any process."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import pytest

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'umrr96_robot.launch.py'
ROBOT = {'smartmicro_radar_node_exe', 'smartmicro_radar_readback_node', 'umrr96_processing'}


@pytest.fixture
def resolve(tmp_path, monkeypatch):
    """Return a function mapping launch arguments to (context, {executable: node})."""
    # Dictionary parameters become temporary files, as when ros2 launch runs.
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    driver_params = str(Path(get_package_share_directory('umrr_ros2_driver')) / 'param'
                        / 'radar.params.umrr96_38553.yaml')

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

        visit(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(LAUNCH)),
            launch_arguments={'driver_params': driver_params, **arguments}.items()))
        return context, nodes
    return run


def test_headless_driver_readback_and_processing(resolve):
    context, nodes = resolve({})
    assert set(nodes) == ROBOT, sorted(nodes)  # No rviz2, views or description.
    # C52: the includes' rviz:=false/views:=false stay scoped.
    for name in ('rviz', 'views', 'params_file', 'input_topic'):
        assert name not in context.launch_configurations, name


def test_namespace_reaches_every_node(resolve):
    _, nodes = resolve({'namespace': 'front', 'publish_description': 'true'})
    assert set(nodes) == ROBOT | {'robot_state_publisher'}, sorted(nodes)
    for executable, node in nodes.items():
        assert node.expanded_node_namespace == '/front', executable


def test_driver_params_has_no_bench_default():
    # The bench file hard-codes one host's interface and addresses (M6).
    with pytest.raises(RuntimeError, match='driver_params'):
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(LAUNCH))).visit(
            LaunchContext())
