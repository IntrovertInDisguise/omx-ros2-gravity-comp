#!/usr/bin/env python3
"""
Coordinated dual-hardware launch

This launch starts the existing dual hardware launch and then starts a
coordinator that publishes a simultaneous waypoint to both robots after a
short delay.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('enable_logger', default_value='false'),
        DeclareLaunchArgument('robot1_port', default_value=''),
        DeclareLaunchArgument('robot2_port', default_value=''),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    enable_logger = LaunchConfiguration('enable_logger')
    robot1_port = LaunchConfiguration('robot1_port')
    robot2_port = LaunchConfiguration('robot2_port')

    # Include the existing dual hardware launch so substitutions are handled
    dual_launch_path = os.path.join(
        get_package_share_directory('omx_variable_stiffness_controller'),
        'launch', 'dual_hardware_variable_stiffness.launch.py')

    dual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dual_launch_path),
        launch_arguments={
            'start_rviz': start_rviz,
            'enable_logger': enable_logger,
            'robot1_port': robot1_port,
            'robot2_port': robot2_port,
        }.items(),
    )

    # Start coordinator after a short delay to allow controllers to come up
    coordinator_exec = ExecuteProcess(
        cmd=['/usr/bin/env', 'python3',
             os.path.join('/workspaces/omx_ros2', 'tools', 'dual_press_coordinator.py'),
             '--namespace1', '/robot1', '--namespace2', '/robot2', '--wait', '6.5'],
        output='screen',
    )

    # Give extra time for controller managers and helper spawners to stabilise
    delay_coordinator = TimerAction(
        period=18.0,
        actions=[coordinator_exec],
    )

    return LaunchDescription(
        [
            *declared_arguments,
            dual_launch,
            delay_coordinator,
        ]
    )
