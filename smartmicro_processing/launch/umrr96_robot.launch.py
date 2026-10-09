# SPDX-License-Identifier: Apache-2.0
"""
Headless UMRR-96 for a robot: data driver, readback/control node and processing.

Starts no RViz, views node or other Qt process. For operator views run
umrr_ros2_driver's umrr96_viz.launch.py with the same namespace on any machine.
driver_params has no default: the shipped radar.params.umrr96_38553.yaml is a
bench file (host interface, addresses, startup CAN-output write) to copy, not a
robot setting. The topics a robot consumes are listed in the driver's
doc/interfaces.md, "Robot integration".
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from smartmicro_processing.experiment_launch import experiment_arguments, EXPERIMENTS


def generate_launch_description():
    """Include the driver (without views and RViz) and processing in one namespace."""
    driver_share = FindPackageShare('umrr_ros2_driver')
    processing_share = FindPackageShare('smartmicro_processing')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'driver_params',
            description='Driver and readback parameters for this host and sensor (required)'),
        DeclareLaunchArgument(
            'processing_params', default_value=PathJoinSubstitution([
                processing_share, 'config', 'umrr96_processing.yaml']),
            description='Doppler fit, ghost, tracker and obstacle parameters'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace of all nodes and topics, e.g. front_radar'),
        DeclareLaunchArgument(
            'frame_id', default_value='umrr96',
            description='Must match sensor_0.frame_id in driver_params; processing accepts '
                        'only this frame'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false'],
            description='Use /clock (for replay)'),
        DeclareLaunchArgument(
            'publish_description', default_value='false', choices=['true', 'false'],
            description='Publish the standalone sensor URDF; leave false when the robot '
                        'description provides the radar frames'),
        DeclareLaunchArgument(
            'description_sensor_name', default_value='umrr96',
            description='Prefix of the housing frames (<name>_link); unique per radar'),
        *experiment_arguments(),
        # Scoped (C52): the includes' arguments, e.g. rviz:=false, stay in the includes.
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                driver_share, 'launch', 'umrr96_live.launch.py'])),
            launch_arguments={
                'params_file': LaunchConfiguration('driver_params'),
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'rviz': 'false',
                'views': 'false',
                'publish_description': LaunchConfiguration('publish_description'),
                'description_frame_id': LaunchConfiguration('frame_id'),
                'description_sensor_name': LaunchConfiguration('description_sensor_name'),
            }.items())]),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                processing_share, 'launch', 'umrr96_processing.launch.py'])),
            launch_arguments={
                'params_file': LaunchConfiguration('processing_params'),
                'namespace': namespace,
                'expected_frame_id': LaunchConfiguration('frame_id'),
                'use_sim_time': use_sim_time,
                **{name: LaunchConfiguration(name) for name in EXPERIMENTS},
            }.items())]),
    ])
