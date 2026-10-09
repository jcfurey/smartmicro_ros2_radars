# SPDX-License-Identifier: Apache-2.0
"""
Operator views of a UMRR-96 whose driver runs elsewhere: views node and RViz.

Run it on any machine in the robot's ROS domain with the namespace of the robot
launch (smartmicro_processing umrr96_robot.launch.py). It starts no driver,
readback or processing node and opens no sensor connection; RViz's UMRR-96 panel
reaches the readback node's services over ROS. Closing RViz stops this launch
only. Use rviz:=false to run just the views node, e.g. for a recorded fan image.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription,
    RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Declare the arguments and the views and RViz nodes."""
    share = get_package_share_directory('umrr_ros2_driver')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    views = Node(
        package='umrr_ros2_driver',
        executable='umrr96_views',
        name='umrr96_views',
        namespace=namespace,
        parameters=[LaunchConfiguration('views_params'), use_sim_time],
        remappings=[('smart_radar/port_targets_0', LaunchConfiguration('targets_topic'))],
        condition=IfCondition(LaunchConfiguration('views')),
        output='log',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='umrr96_rviz',
        namespace=namespace,
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[use_sim_time],
        # Relative configuration names (C39) resolve in the namespace; the description
        # topic is named after the description's sensor name.
        remappings=[('umrr96/robot_description',
                     [LaunchConfiguration('description_sensor_name'), '/robot_description'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace of the radar: the same as the robot launch.',
        ),
        DeclareLaunchArgument(
            'views_params',
            default_value=os.path.join(share, 'param', 'radar.params.umrr96_38553.yaml'),
            description='Parameter file with an umrr96_views section; only that section is '
                        'read, so the host settings of the default bench file do not matter.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false'],
            description='Use /clock (for replay).',
        ),
        DeclareLaunchArgument(
            'views', default_value='true', choices=['true', 'false'],
            description='Start the views node; false to only open RViz.',
        ),
        DeclareLaunchArgument(
            'targets_topic', default_value='smart_radar/port_targets_0',
            description='Target cloud consumed by the views node (relative to namespace).',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false'],
            description='Start RViz; closing it stops this launch.',
        ),
        DeclareLaunchArgument(
            'view', default_value='grid', choices=['grid', 'fan', 'live'],
            description='Detection density grid, 2D fan, or original 3D point view',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'rviz', ['umrr96_', LaunchConfiguration('view'), '.rviz']]),
            description='RViz display configuration, e.g. rviz/umrr96_classified.rviz of '
                        'smartmicro_processing for the processing outputs',
        ),
        DeclareLaunchArgument(
            'publish_description', default_value='false', choices=['true', 'false'],
            description='Publish the standalone sensor URDF; only if neither the robot launch '
                        'nor a robot description publishes its TF',
        ),
        DeclareLaunchArgument(
            'description_frame_id', default_value='umrr96',
            description='Must match sensor_0.frame_id of the driver',
        ),
        DeclareLaunchArgument(
            'description_sensor_name', default_value='umrr96',
            description='Prefix of the housing frames and of <name>/robot_description',
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))],
        )),
        # Scoped (C52): include arguments must not become this file's configurations.
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('smartmicro_description'),
                'launch', 'umrr96_description.launch.py'])),
            condition=IfCondition(LaunchConfiguration('publish_description')),
            # Explicit rviz:=false: the scope still forwards this file's rviz:=true.
            launch_arguments={
                'frame_id': LaunchConfiguration('description_frame_id'),
                'sensor_name': LaunchConfiguration('description_sensor_name'),
                'namespace': namespace,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rviz': 'false',
            }.items(),
        )]),
        views,
        rviz,
    ])
