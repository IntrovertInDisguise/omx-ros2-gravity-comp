#!/usr/bin/env python3
"""Single robot hardware launch for gravity compensation.

Connects directly to Dynamixel servos — no Gazebo needed.
"""

import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def detect_serial_port():
    """Return the first available serial port, or /dev/ttyUSB0 as fallback."""
    for pattern in ["/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*"]:
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return "/dev/ttyUSB0"


def generate_launch_description():
    default_port = detect_serial_port()

    declared_arguments = [
        DeclareLaunchArgument(
            'port',
            default_value=default_port,
            description='Serial port for the robot (auto-detected)'
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether to start RViz2'
        ),
    ]

    port = LaunchConfiguration('port')
    start_rviz = LaunchConfiguration('start_rviz')

    controller_config = PathJoinSubstitution([
        FindPackageShare('omx_dual_bringup'),
        'config', 'single_robot_hardware_gravity_comp.yaml'
    ])

    urdf = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro'
        ]),
        ' use_sim:=false',
        ' use_fake_hardware:=false',
        ' port_name:=', port,
        ' controller_config:=', controller_config,
        ' robot_namespace:=omx',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='omx',
        parameters=[{
            'robot_description': urdf,
            'use_sim_time': False,
        }],
        output='screen'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='omx',
        parameters=[
            {'robot_description': urdf, 'use_sim_time': False},
            controller_config
        ],
        output='both',
    )

    # Load controllers after controller_manager has time to start
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '-c', '/omx/controller_manager',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_gravity = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '-c', '/omx/controller_manager',
             'gravity_comp_controller'],
        output='screen'
    )

    delay_controllers = TimerAction(
        period=3.0,
        actions=[load_jsb, load_gravity],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(start_rviz)
    )

    return LaunchDescription([
        *declared_arguments,
        robot_state_publisher,
        controller_manager,
        delay_controllers,
        rviz_node,
    ])
