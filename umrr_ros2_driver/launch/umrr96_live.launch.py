"""Show the connected UMRR-96 in RViz; closing RViz stops the driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('umrr_ros2_driver')
    radar = Node(
        package='umrr_ros2_driver',
        executable='smartmicro_radar_node_exe',
        name='smart_radar',
        parameters=[LaunchConfiguration('params_file')],
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
        parameters=[LaunchConfiguration('params_file')],
        output='log',
    )
    views = Node(
        package='umrr_ros2_driver',
        executable='umrr96_views',
        name='umrr96_views',
        parameters=[LaunchConfiguration('params_file')],
        output='log',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='umrr96_rviz',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='log',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'param', 'radar.params.umrr96_38553.yaml'),
            description='Radar driver parameters',
        ),
        DeclareLaunchArgument(
            'publish_description', default_value='false', choices=['true', 'false'],
            description='Publish the standalone sensor URDF; disable if another publisher owns its TF',
        ),
        DeclareLaunchArgument(
            'description_frame_id', default_value='umrr96',
            description='Must match sensor_0.frame_id in params_file when publishing the description',
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
                FindPackageShare('smartmicro_description'), 'launch', 'umrr96_description.launch.py'])),
            condition=IfCondition(LaunchConfiguration('publish_description')),
            launch_arguments={'frame_id': LaunchConfiguration('description_frame_id')}.items(),
        ),
        radar,
        readback,
        views,
        rviz,
    ])
