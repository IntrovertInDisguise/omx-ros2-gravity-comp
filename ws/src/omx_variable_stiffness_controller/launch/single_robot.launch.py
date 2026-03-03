#!/usr/bin/env python3
"""
Single-robot convenience wrapper for the variable-stiffness Gazebo launch.

Includes the package's `gazebo_variable_stiffness.launch.py` and exposes a
small set of launch arguments useful for quick single-robot testing.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_namespace', default_value='omx', description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'launch_gazebo', default_value='true', description='Start Gazebo server'
        ),
    ]

    robot_namespace = LaunchConfiguration('robot_namespace')
    launch_gazebo = LaunchConfiguration('launch_gazebo')

    pkg_share = get_package_share_directory('omx_variable_stiffness_controller')
    included_launch = os.path.join(pkg_share, 'launch', 'gazebo_variable_stiffness.launch.py')

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'launch_gazebo': launch_gazebo,
        }.items(),
    )

    return LaunchDescription([
        *declared_arguments,
        include,
    ])
