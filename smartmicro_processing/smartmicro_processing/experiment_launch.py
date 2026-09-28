# SPDX-License-Identifier: Apache-2.0
"""Optional tracker overrides shared by processing and live-view launches."""
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


EXPERIMENTS = ('evidence_confirmation', 'standing_support', 'joint_association')


def experiment_arguments():
    """Preserve the parameter file unless a caller explicitly chooses true or false."""
    return [DeclareLaunchArgument(
        name, default_value='config', choices=['config', 'true', 'false'],
        description='Experimental tracker option; config uses the parameter file (default off)')
        for name in EXPERIMENTS]


def experiment_overrides(context):
    """Return only explicit boolean overrides, applied after the parameter file."""
    values = {name: LaunchConfiguration(name).perform(context) for name in EXPERIMENTS}
    return {name: value == 'true' for name, value in values.items() if value != 'config'}
