#!/usr/bin/env python3
"""Single robot Gazebo test for gravity compensation."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Gazebo environment setup
    plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        [TextSubstitution(text='/opt/ros/humble/lib:'), EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value='')]
    )
    
    controller_config = PathJoinSubstitution([
        FindPackageShare('omx_dual_bringup'),
        'config', 'single_robot_gravity_comp.yaml'
    ])

    urdf = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro'
        ]),
        ' use_sim:=true',
        ' use_fake_hardware:=false',
        ' controller_config:=', controller_config,
        ' robot_namespace:=omx',  # Use 'omx' namespace for single robot
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='omx',
        parameters=[{
            'robot_description': urdf,
            'use_sim_time': True,
        }],
        output='screen'
    )

    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        additional_env={'GAZEBO_PLUGIN_PATH': '/opt/ros/humble/lib'},
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-entity', 'open_manipulator_x',
            '-topic', '/omx/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-robot_namespace', 'omx',
        ],
        output='screen'
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/omx/controller_manager'],
        output='screen'
    )

    gravity_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gravity_comp_controller', '--controller-manager', '/omx/controller_manager'],
        output='screen'
    )

    load_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_spawner],
        )
    )

    load_gravity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[gravity_spawner],
        )
    )

    return LaunchDescription([
        plugin_path,
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_robot,
        load_jsb,
        load_gravity,
    ])
