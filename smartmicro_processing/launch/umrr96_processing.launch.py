# SPDX-License-Identifier: Apache-2.0
"""Start the experimental Doppler processing node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Remap the relative input name; a namespace moves inputs and outputs together."""
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('smartmicro_processing'), 'config', 'umrr96_processing.yaml'])),
        DeclareLaunchArgument('namespace', default_value='',
                              description='ROS namespace for the node and its topics'),
        DeclareLaunchArgument('input_topic', default_value='smart_radar/port_targets_0',
                              description='Driver target cloud; relative names resolve '
                                          'in the namespace'),
        DeclareLaunchArgument('expected_frame_id', default_value='umrr96'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        Node(package='smartmicro_processing', executable='umrr96_processing',
             name='umrr96_processing', namespace=LaunchConfiguration('namespace'), output='log',
             remappings=[('smart_radar/port_targets_0', LaunchConfiguration('input_topic'))],
             parameters=[LaunchConfiguration('params_file'), {
                 'expected_frame_id': LaunchConfiguration('expected_frame_id'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                value_type=bool),
             }]),
    ])
