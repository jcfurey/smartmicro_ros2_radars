# SPDX-License-Identifier: Apache-2.0
"""Publish a standalone UMRR-96 description, optionally showing it in RViz."""

import math
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def _description(context):
    share = Path(get_package_share_directory('smartmicro_description'))
    keys = ('sensor_name', 'frame_id', 'measurement_xyz', 'measurement_rpy')
    mappings = {key: LaunchConfiguration(key).perform(context) for key in keys}
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', mappings['sensor_name']):
        raise ValueError('sensor_name must be a ROS name without slashes')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_/]*', mappings['frame_id']):
        raise ValueError('frame_id must be nonempty and cannot start with a slash')
    if mappings['frame_id'] == mappings['sensor_name'] + '_link':
        raise ValueError('frame_id must differ from the housing link name')
    for key in ('measurement_xyz', 'measurement_rpy'):
        try:
            values = [float(value) for value in mappings[key].split()]
        except ValueError as error:
            raise ValueError(f'{key} must contain three finite numbers') from error
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise ValueError(f'{key} must contain three finite numbers')

    document = xacro.process_file(str(share / 'urdf' / 'umrr96.urdf.xacro'), mappings=mappings)
    topic = mappings['sensor_name'] + '/robot_description'
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name=mappings['sensor_name'] + '_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'robot_description': ParameterValue(document.toxml(), value_type=str),
                         'use_sim_time': use_sim_time}],
            remappings=[('robot_description', topic)],
            output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2',
            name=mappings['sensor_name'] + '_description_rviz',
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', str(share / 'rviz' / 'umrr96_description.rviz'),
                       '-f', mappings['frame_id']],
            parameters=[{'use_sim_time': use_sim_time}],
            # The RViz config subscribes to the relative 'robot_description'; resolve it
            # to this sensor's description inside the launch namespace.
            remappings=[('robot_description', topic)],
            output='log',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sensor_name', default_value='umrr96',
                              description='Unique housing/joint/node name prefix'),
        DeclareLaunchArgument(
            'frame_id', default_value=LaunchConfiguration('sensor_name'),
            description='Measurement frame; must match radar PointCloud2.header.frame_id'),
        DeclareLaunchArgument(
            'measurement_xyz', default_value='0 0 0',
            description='Housing-to-measurement origin in metres; '
                        'zero is an uncalibrated convention'),
        DeclareLaunchArgument('measurement_rpy', default_value='0 0 0',
                              description='Housing-to-measurement roll/pitch/yaw in radians'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='ROS node/topic namespace; does not prefix TF frames'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='Open a close-up model viewer; does not start the radar driver'),
        OpaqueFunction(function=_description),
    ])
