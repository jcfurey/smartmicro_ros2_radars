# SPDX-License-Identifier: Apache-2.0
"""Start the UMRR-96 driver, Doppler processing/tracking and the tracking RViz view together."""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Closing RViz stops the driver and processing; use rviz:=false for headless runs."""
    driver_share = FindPackageShare('umrr_ros2_driver')
    processing_share = FindPackageShare('smartmicro_processing')
    rviz = Node(
        package='rviz2', executable='rviz2', name='umrr96_tracking_rviz', output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')))
    return LaunchDescription([
        # Distinct names: parent launch configurations are visible in included files,
        # and each include is scoped so its arguments (e.g. rviz:=false) do not leak back.
        DeclareLaunchArgument(
            'driver_params', default_value=PathJoinSubstitution([
                driver_share, 'param', 'radar.params.umrr96_38553.yaml']),
            description='Driver, readback and views parameters (host/sensor specific)'),
        DeclareLaunchArgument(
            'processing_params', default_value=PathJoinSubstitution([
                processing_share, 'config', 'umrr96_processing.yaml']),
            description='Doppler fit, ghost, tracker and obstacle parameters'),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false'],
            description='Start the tracking RViz view; closing it stops the launch'),
        DeclareLaunchArgument(
            'rviz_config', default_value=PathJoinSubstitution([
                processing_share, 'rviz', 'umrr96_moving.rviz']),
            description='RViz display configuration (absolute topic names, no namespace)'),
        DeclareLaunchArgument(
            'publish_description', default_value='true', choices=['true', 'false'],
            description='Publish the standalone sensor URDF shown in RViz; set false if '
                        'another robot description already publishes its TF'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false'],
            description='Use /clock (for replay)'),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                driver_share, 'launch', 'umrr96_live.launch.py'])),
            launch_arguments={
                'params_file': LaunchConfiguration('driver_params'),
                'rviz': 'false',
                'publish_description': LaunchConfiguration('publish_description'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items())]),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                processing_share, 'launch', 'umrr96_processing.launch.py'])),
            launch_arguments={
                'params_file': LaunchConfiguration('processing_params'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items())]),
        rviz,
        RegisterEventHandler(OnProcessExit(
            target_action=rviz, on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))])),
    ])
