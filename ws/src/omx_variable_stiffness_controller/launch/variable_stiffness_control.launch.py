#!/usr/bin/env python3
"""
Launch file for Variable Stiffness Controller on OpenManipulator-X.

This launch file starts:
- Robot state publisher
- ros2_control controller manager
- Joint state broadcaster
- Variable stiffness controller
- Optional: Stiffness profile loader
- Optional: Data logger
"""

import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
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
            'sim',
            default_value='false',
            description='Whether to run in simulation mode'
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether to start RViz2'
        ),
        DeclareLaunchArgument(
            'csv_file',
            default_value='',
            description='Path to CSV file with stiffness profiles (optional)'
        ),
        DeclareLaunchArgument(
            'enable_logger',
            default_value='true',
            description='Whether to enable data logging'
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='omx',
            description='Robot namespace'
        ),
    ]

    port = LaunchConfiguration('port')
    sim = LaunchConfiguration('sim')
    start_rviz = LaunchConfiguration('start_rviz')
    csv_file = LaunchConfiguration('csv_file')
    enable_logger = LaunchConfiguration('enable_logger')
    robot_namespace = LaunchConfiguration('robot_namespace')

    # Controller configuration file
    controller_config = PathJoinSubstitution([
        FindPackageShare('omx_variable_stiffness_controller'),
        'config', 'variable_stiffness_controller.yaml'
    ])

    # Generate URDF from xacro
    urdf = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro'
        ]),
        ' use_sim:=', sim,
        ' use_fake_hardware:=', sim,
        ' port_name:=', port,
        ' controller_config:=', controller_config,
        ' robot_namespace:=', robot_namespace,
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_namespace,
        parameters=[{
            'robot_description': urdf,
            'use_sim_time': PythonExpression(["'", sim, "' == 'true'"]),
        }],
        output='screen'
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=robot_namespace,
        parameters=[
            {'robot_description': urdf},
            {'use_sim_time': PythonExpression(["'", sim, "' == 'true'"])},
            controller_config
        ],
        output='both',
    )

    # Load joint state broadcaster
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '-c', ['/', robot_namespace, '/controller_manager'],
             'joint_state_broadcaster'],
        output='screen'
    )

    # Load variable stiffness controller
    load_vs_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '-c', ['/', robot_namespace, '/controller_manager'],
             'variable_stiffness_controller'],
        output='screen'
    )

    # Delay controller loading to ensure controller_manager is ready
    delay_controllers = TimerAction(
        period=3.0,
        actions=[load_jsb, load_vs_controller],
    )

    # Stiffness profile loader node (optional, if CSV file is specified)
    stiffness_loader = Node(
        package='omx_variable_stiffness_controller',
        executable='load_stiffness.py',
        namespace=robot_namespace,
        name='stiffness_loader',
        parameters=[{
            'csv_path': csv_file,
            'controller_name': 'variable_stiffness_controller',
        }],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", csv_file, "' != ''"])
        ),
    )

    # Delay stiffness loader to ensure controller is configured
    delay_stiffness_loader = TimerAction(
        period=5.0,
        actions=[stiffness_loader],
    )

    # Data logger node (optional)
    logger_node = Node(
        package='omx_variable_stiffness_controller',
        executable='logger.py',
        namespace=robot_namespace,
        name='csv_data_logger',
        parameters=[{
            'controller_name': 'variable_stiffness_controller',
        }],
        output='screen',
        condition=IfCondition(enable_logger),
    )

    # Delay logger to ensure controller is active
    delay_logger = TimerAction(
        period=5.0,
        actions=[logger_node],
    )

    # RViz node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{
            'use_sim_time': PythonExpression(["'", sim, "' == 'true'"]),
        }],
        output='screen',
        condition=IfCondition(start_rviz)
    )

    return LaunchDescription([
        *declared_arguments,
        robot_state_publisher,
        controller_manager,
        delay_controllers,
        delay_stiffness_loader,
        delay_logger,
        rviz_node,
    ])
