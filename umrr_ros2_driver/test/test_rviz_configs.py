# SPDX-License-Identifier: Apache-2.0
"""Check the shipped RViz configurations against the clouds the nodes publish."""

from pathlib import Path
import re

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = sorted((ROOT / 'config' / 'rviz').glob('*.rviz')) + [
    ROOT.parent / 'smart_rviz_plugin' / 'config' / 'rviz' / 'rviz_config.rviz']
# Driver target clouds (doc/interfaces.md) and the views node's fan cloud.
TARGET_FIELDS = {
    'x', 'y', 'z', 'radial_speed', 'power', 'rcs', 'noise', 'snr', 'azimuth_angle',
    'elevation_angle', 'range', 'variance_range', 'variance_speed', 'variance_azimuth_angle',
    'variance_elevation_angle', 'false_alarm_probability', 'flags', 'peak_idx'}
VIEW_FIELDS = {'/smart_radar/fan_targets': {'x', 'y', 'z', 'snr'}}
TARGETS = re.compile(r'/smart_radar/(port|can)_targets_\d+$')


def config_id(path):
    return f'{path.parents[2].name}/{path.name}'


def displays(path):
    """Yield every display of a configuration, including those inside groups."""
    pending = list(yaml.safe_load(path.read_text())['Visualization Manager']['Displays'])
    while pending:
        display = pending.pop()
        pending.extend(display.get('Displays', []))
        yield display


@pytest.mark.parametrize('path', CONFIGS, ids=config_id)
def test_point_cloud_color_channels_exist(path):
    for display in displays(path):
        if display['Class'] != 'rviz_default_plugins/PointCloud2':
            continue
        topic = display['Topic']['Value']
        fields = TARGET_FIELDS if TARGETS.match(topic) else VIEW_FIELDS[topic]
        if display['Color Transformer'] == 'Intensity':
            # S27: 'intensity' is not a field of the target cloud.
            assert display['Channel Name'] in fields, (display['Name'], topic)


@pytest.mark.parametrize('path', CONFIGS, ids=config_id)
def test_driver_clouds_accept_best_effort_publishers(path):
    # S25: a reliable subscription receives nothing from a best-effort override.
    for display in displays(path):
        topic = display.get('Topic', {})
        if TARGETS.match(str(topic.get('Value', ''))):
            assert topic['Reliability Policy'] == 'Best Effort', (display['Name'], path.name)


@pytest.mark.parametrize('path', CONFIGS, ids=config_id)
def test_robot_model_follows_the_publish_description_default(path):
    # umrr96_live.launch.py does not publish the description by default.
    for display in displays(path):
        if display['Class'] == 'rviz_default_plugins/RobotModel':
            assert display['Enabled'] is False, (display['Name'], path.name)
