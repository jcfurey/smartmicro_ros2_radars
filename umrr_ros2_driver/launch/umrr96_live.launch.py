"""
Run the connected UMRR-96: data driver, readback/control node, views and RViz.

With rviz:=true (the default) closing RViz stops the whole launch; use
rviz:=false on headless machines. The default params_file is the cam-ripper
bench configuration (host interface enp68s0f0, 192.168.11.x addresses); pass
params_file:=<your file> for any other host or sensor.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Declare the arguments and the driver, readback, views and RViz nodes."""
    share = get_package_share_directory('umrr_ros2_driver')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    radar = Node(
        package='umrr_ros2_driver',
        executable='smartmicro_radar_node_exe',
        name='smart_radar',
        namespace=namespace,
        parameters=[params_file, use_sim_time],
        remappings=[
            ('smart_radar/set_radar_mode', 'smart_radar/data_receiver/set_radar_mode'),
            ('smart_radar/get_radar_mode', 'smart_radar/data_receiver/get_radar_mode'),
            ('smart_radar/get_radar_status', 'smart_radar/data_receiver/get_radar_status'),
        ],
        output='log',
    )
    readback = Node(
        package='umrr_ros2_driver',
        executable='smartmicro_radar_readback_node',
        name='smart_radar_readback',
        namespace=namespace,
        parameters=[params_file, use_sim_time],
        output='log',
    )
    views = Node(
        package='umrr_ros2_driver',
        executable='umrr96_views',
        name='umrr96_views',
        namespace=namespace,
        parameters=[params_file, use_sim_time],
        # Relative names: the views follow the driver into the namespace.
        remappings=[('smart_radar/port_targets_0', LaunchConfiguration('targets_topic'))],
        output='log',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='umrr96_rviz',
        namespace=namespace,
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'param', 'radar.params.umrr96_38553.yaml'),
            description='Driver, readback and views parameters. The default is the '
                        'cam-ripper bench file (enp68s0f0, 192.168.11.17 -> .11); '
                        'supply your own for other hosts or sensors.',
        ),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace for all nodes, e.g. front_radar. Topics become '
                        '<namespace>/smart_radar/...; parameter keys use /** wildcards.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false'],
            description='Use /clock (for replay).',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false'],
            description='Start RViz; closing it stops the launch. false for headless use.',
        ),
        DeclareLaunchArgument(
            'targets_topic', default_value='smart_radar/port_targets_0',
            description='Target cloud consumed by the views node (relative to namespace).',
        ),
        DeclareLaunchArgument(
            'publish_description', default_value='false', choices=['true', 'false'],
            description='Publish the standalone sensor URDF; '
                        'disable if another publisher owns its TF',
        ),
        DeclareLaunchArgument(
            'description_frame_id', default_value='umrr96',
            description='Must match sensor_0.frame_id in params_file '
                        'when publishing the description',
        ),
        DeclareLaunchArgument(
            'view',
            default_value='grid',
            choices=['grid', 'fan', 'live'],
            description='Detection density grid, 2D fan, or original 3D point view',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'rviz', ['umrr96_', LaunchConfiguration('view'), '.rviz']]),
            description='RViz display configuration',
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))],
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('smartmicro_description'),
                'launch', 'umrr96_description.launch.py'])),
            condition=IfCondition(LaunchConfiguration('publish_description')),
            launch_arguments={
                'frame_id': LaunchConfiguration('description_frame_id'),
                'namespace': namespace,
            }.items(),
        ),
        radar,
        readback,
        views,
        rviz,
    ])
