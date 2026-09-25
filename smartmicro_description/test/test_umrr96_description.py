# SPDX-License-Identifier: Apache-2.0
"""Check physical dimensions, standalone roots and composition into a robot."""
import importlib.util
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from launch import LaunchContext
import pytest
import xacro


PACKAGE = Path(__file__).resolve().parents[1]


def expand(mappings=None):
    return xacro.process_file(str(PACKAGE / 'urdf' / 'umrr96.urdf.xacro'),
                              mappings=mappings or {})


def check_tree(document, tmp_path):
    path = tmp_path / 'sensor.urdf'
    path.write_text(document.toxml())
    result = subprocess.run(['check_urdf', str(path)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    return ET.fromstring(document.toxml())


def vector(element, attribute):
    return [float(value) for value in element.get(attribute).split()]


def test_standalone_has_no_invented_robot_pose(tmp_path):
    root = check_tree(expand(), tmp_path)
    assert {link.get('name') for link in root.findall('link')} == {'umrr96', 'umrr96_link'}
    joints = root.findall('joint')
    assert len(joints) == 1 and joints[0].get('type') == 'fixed'
    assert joints[0].find('parent').get('link') == 'umrr96_link'
    assert joints[0].find('child').get('link') == 'umrr96'
    assert vector(joints[0].find('origin'), 'xyz') == [0, 0, 0]
    assert vector(joints[0].find('origin'), 'rpy') == [0, 0, 0]
    assert not root.findall('.//inertial')  # The datasheet gives no measured COM/inertia.


def test_envelope_uses_front_drawing_axes_and_covers_visuals(tmp_path):
    root = check_tree(expand(), tmp_path)
    body = root.find("link[@name='umrr96_link']")
    nominal = body.find("visual[@name='umrr96_housing']/geometry/box")
    assert vector(nominal, 'size') == pytest.approx([.0177, .097, .076])
    collision = body.find('collision')
    center = vector(collision.find('origin'), 'xyz')
    size = vector(collision.find('geometry/box'), 'size')
    assert size == pytest.approx([.02245, .1038, .076])
    for visual in body.findall('visual'):
        origin = visual.find('origin')
        xyz = vector(origin, 'xyz') if origin is not None else [0, 0, 0]
        dimensions = vector(visual.find('geometry/box'), 'size')
        for c, extent, v, width in zip(center, size, xyz, dimensions):
            assert v - width / 2 >= c - extent / 2 - 1e-12
            assert v + width / 2 <= c + extent / 2 + 1e-12


def test_custom_frame_keeps_calibration_separate_from_housing(tmp_path):
    root = check_tree(expand({'sensor_name': 'rear_radar', 'frame_id': 'robot/rear_radar',
                              'measurement_xyz': '.01 -.02 .03',
                              'measurement_rpy': '.1 -.2 .3'}), tmp_path)
    joint = root.find("joint[@name='rear_radar_measurement_joint']")
    assert joint.find('parent').get('link') == 'rear_radar_link'
    assert joint.find('child').get('link') == 'robot/rear_radar'
    assert vector(joint.find('origin'), 'xyz') == pytest.approx([.01, -.02, .03])
    assert vector(joint.find('origin'), 'rpy') == pytest.approx([.1, -.2, .3])


def test_two_radars_compose_without_duplicate_links_or_materials(tmp_path):
    path = PACKAGE / 'urdf' / 'umrr96_macro.urdf.xacro'
    document = xacro.parse(f"""<robot name="rig" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{path}"/>
      <link name="base_link"/>
      <xacro:umrr96_sensor name="front" parent="base_link" xyz=".2 0 .4"/>
      <xacro:umrr96_sensor name="rear" parent="base_link" xyz="-.2 0 .4"
                          rpy="0 0 3.141592653589793" measurement_xyz=".01 0 0"/>
    </robot>""")
    xacro.process_doc(document)
    root = check_tree(document, tmp_path)
    names = [link.get('name') for link in root.findall('link')]
    assert len(names) == len(set(names)) == 5
    materials = [m.get('name') for m in root.findall('.//material')]
    assert len(materials) == len(set(materials))
    rear = root.find("joint[@name='rear_mount_joint']")
    assert rear.find('parent').get('link') == 'base_link'
    assert vector(rear.find('origin'), 'xyz') == [-.2, 0, .4]
    assert vector(rear.find('origin'), 'rpy')[2] == pytest.approx(3.141592653589793)


@pytest.mark.parametrize('parameter,value', [('measurement_xyz', 'nan 0 0'),
                                             ('measurement_rpy', '0 0'),
                                             ('frame_id', 'umrr96_link'),
                                             ('frame_id', '/umrr96')])
def test_launch_rejects_invalid_tf_before_starting_a_publisher(parameter, value, monkeypatch,
                                                               tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'roslog'))
    path = PACKAGE / 'launch' / 'umrr96_description.launch.py'
    spec = importlib.util.spec_from_file_location('description_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(PACKAGE))
    context = LaunchContext()
    context.launch_configurations.update({'sensor_name': 'umrr96', 'frame_id': 'umrr96',
                                          'measurement_xyz': '0 0 0', 'measurement_rpy': '0 0 0',
                                          parameter: value})
    with pytest.raises(ValueError):
        module._description(context)


def test_rviz_description_topic_is_relative_so_launch_namespace_applies():
    text = (PACKAGE / 'rviz' / 'umrr96_description.rviz').read_text()
    assert 'Value: robot_description' in text
    assert '/umrr96/robot_description' not in text
    launch = (PACKAGE / 'launch' / 'umrr96_description.launch.py').read_text()
    assert "('/umrr96/robot_description'" not in launch
