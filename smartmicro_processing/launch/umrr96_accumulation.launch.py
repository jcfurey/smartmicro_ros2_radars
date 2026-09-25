# SPDX-License-Identifier: Apache-2.0
"""Start bounded radar evidence accumulation and, optionally, its preview viewer."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Remap the relative input name; a namespace moves inputs and outputs together."""
    share = FindPackageShare('smartmicro_processing')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            share, 'config', 'umrr96_accumulation.yaml'])),
        DeclareLaunchArgument('mode', default_value='pose_compensated',
                              choices=['pose_compensated', 'stationary_preview']),
        DeclareLaunchArgument('namespace', default_value='',
                              description='ROS namespace for the nodes and their topics'),
        DeclareLaunchArgument('node_name', default_value='umrr96_accumulation'),
        DeclareLaunchArgument('input_topic', default_value='umrr96_processing/doppler_inliers',
                              description='Doppler inlier cloud; relative names resolve '
                                          'in the namespace'),
        DeclareLaunchArgument('expected_frame_id', default_value='umrr96'),
        DeclareLaunchArgument('fixed_frame', default_value='odom'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                              description='Open the stationary preview viewer '
                                          '(preview mode only)'),
        Node(package='smartmicro_processing', executable='umrr96_accumulation',
             name=LaunchConfiguration('node_name'), namespace=LaunchConfiguration('namespace'),
             output='log',
             remappings=[('umrr96_processing/doppler_inliers',
                          LaunchConfiguration('input_topic'))],
             parameters=[LaunchConfiguration('params_file'), {
                 name: ParameterValue(LaunchConfiguration(name), value_type=str)
                 for name in ('mode', 'expected_frame_id', 'fixed_frame')
             }, {'use_sim_time': use_sim_time}]),
        Node(package='rviz2', executable='rviz2', name='umrr96_accumulation_rviz', output='log',
             namespace=LaunchConfiguration('namespace'),
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('rviz'), "' == 'true' and '",
                 LaunchConfiguration('mode'), "' == 'stationary_preview'"
             ])),
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'umrr96_accumulation.rviz']),
                        '-f', LaunchConfiguration('expected_frame_id')],
             remappings=[
                 ('/umrr96_accumulation/stationary_preview/accumulated_targets',
                  [LaunchConfiguration('node_name'), '/stationary_preview/accumulated_targets']),
                 ('/umrr96_accumulation/stationary_preview/confirmed_targets',
                  [LaunchConfiguration('node_name'), '/stationary_preview/confirmed_targets']),
                 ('/umrr96_processing/doppler_inliers', LaunchConfiguration('input_topic'))],
             parameters=[{'use_sim_time': use_sim_time}]),
    ])
