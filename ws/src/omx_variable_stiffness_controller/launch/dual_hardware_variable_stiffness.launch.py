#!/usr/bin/env python3
"""
Dual Open Manipulator X with Variable Stiffness Control using Hardware.

Launches two independent robots with variable Cartesian impedance control.
Each arm has its own independent trajectory and stiffness profile.
"""

import os
import glob
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def detect_serial_ports():
    """Attempt to detect available serial ports."""
    ports = []
    
    # Check /dev/serial/by-id/ first (most reliable)
    byid = sorted(glob.glob("/dev/serial/by-id/*"))
    ports.extend(byid[:2])
    
    # Fallback to USB/ACM devices
    if len(ports) < 2:
        for pattern in ["/dev/ttyUSB*", "/dev/ttyACM*"]:
            hits = sorted(glob.glob(pattern))
            for hit in hits:
                if hit not in ports:
                    ports.append(hit)
                if len(ports) >= 2:
                    break
    
    return ports


def generate_launch_description():
    detected_ports = detect_serial_ports()
    default_port1 = detected_ports[0] if len(detected_ports) > 0 else "/dev/ttyUSB0"
    default_port2 = detected_ports[1] if len(detected_ports) > 1 else "/dev/ttyUSB1"

    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether to start RViz2'
        ),
        DeclareLaunchArgument(
            'robot1_port',
            default_value=default_port1,
            description='Serial port for robot1 (left arm)'
        ),
        DeclareLaunchArgument(
            'robot2_port',
            default_value=default_port2,
            description='Serial port for robot2 (right arm)'
        ),
        DeclareLaunchArgument(
            'robot1_csv_file',
            default_value='',
            description='Path to Robot 1 stiffness profile CSV (optional)'
        ),
        DeclareLaunchArgument(
            'robot2_csv_file',
            default_value='',
            description='Path to Robot 2 stiffness profile CSV (optional)'
        ),
        DeclareLaunchArgument(
            'enable_logger',
            default_value='false',
            description='Whether to enable data logging for both arms'
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    robot1_port = LaunchConfiguration('robot1_port')
    robot2_port = LaunchConfiguration('robot2_port')
    robot1_csv_file = LaunchConfiguration('robot1_csv_file')
    robot2_csv_file = LaunchConfiguration('robot2_csv_file')
    enable_logger = LaunchConfiguration('enable_logger')

    # Get URDF via xacro for both robots
    urdf_robot1 = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro'
        ]),
        ' use_sim:=false',
        ' use_fake_hardware:=false',
        ' port_name:=', robot1_port,
        ' prefix:=robot1_',
    ])

    urdf_robot2 = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro'
        ]),
        ' use_sim:=false',
        ' use_fake_hardware:=false',
        ' port_name:=', robot2_port,
        ' prefix:=robot2_',
    ])

    # Controller configurations
    controller_config_robot1 = PathJoinSubstitution([
        FindPackageShare('omx_variable_stiffness_controller'),
        'config', 'robot1_variable_stiffness.yaml'
    ])

    controller_config_robot2 = PathJoinSubstitution([
        FindPackageShare('omx_variable_stiffness_controller'),
        'config', 'robot2_variable_stiffness.yaml'
    ])

    # RViz configuration
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('omx_dual_bringup'),
        'rviz', 'dual_robots.rviz'
    ])

    # Robot State Publishers
    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot1',
        parameters=[{
            'robot_description': urdf_robot1,
            'use_sim_time': False,
            'frame_prefix': 'robot1_'
        }],
        output='screen'
    )

    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot2',
        parameters=[{
            'robot_description': urdf_robot2,
            'use_sim_time': False,
            'frame_prefix': 'robot2_'
        }],
        output='screen'
    )

    # Controller Manager for Robot 1
    robot1_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='robot1',
        parameters=[
            {'robot_description': urdf_robot1, 'use_sim_time': False},
            controller_config_robot1
        ],
        output='both',
        remappings=[
            ('/robot1/controller_manager/robot_description', '/robot1/robot_description'),
        ]
    )

    # Controller Manager for Robot 2
    robot2_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='robot2',
        parameters=[
            {'robot_description': urdf_robot2, 'use_sim_time': False},
            controller_config_robot2
        ],
        output='both',
        remappings=[
            ('/robot2/controller_manager/robot_description', '/robot2/robot_description'),
        ]
    )

    # Load controllers for Robot 1
    load_joint_state_broadcaster_robot1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '-c', '/robot1/controller_manager',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_variable_stiffness_controller_robot1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '-c', '/robot1/controller_manager',
             'robot1_variable_stiffness'],
        output='screen'
    )

    # Load controllers for Robot 2
    load_joint_state_broadcaster_robot2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '-c', '/robot2/controller_manager',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_variable_stiffness_controller_robot2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '-c', '/robot2/controller_manager',
             'robot2_variable_stiffness'],
        output='screen'
    )

    # Delay controller loading
    delay_robot1_controllers = TimerAction(
        period=3.0,
        actions=[load_joint_state_broadcaster_robot1, load_variable_stiffness_controller_robot1],
    )

    delay_robot2_controllers = TimerAction(
        period=3.5,
        actions=[load_joint_state_broadcaster_robot2, load_variable_stiffness_controller_robot2],
    )

    # Stiffness profile loaders (optional, if CSV files are specified)
    # Robot 1 stiffness loader
    robot1_stiffness_loader = Node(
        package='omx_variable_stiffness_controller',
        executable='load_stiffness.py',
        namespace='robot1',
        name='stiffness_loader',
        parameters=[{
            'csv_path': robot1_csv_file,
            'controller_name': 'robot1_variable_stiffness',
        }],
        output='screen',
    )

    # Robot 2 stiffness loader
    robot2_stiffness_loader = Node(
        package='omx_variable_stiffness_controller',
        executable='load_stiffness.py',
        namespace='robot2',
        name='stiffness_loader',
        parameters=[{
            'csv_path': robot2_csv_file,
            'controller_name': 'robot2_variable_stiffness',
        }],
        output='screen',
    )

    # Delay stiffness loaders
    delay_stiffness_loaders = TimerAction(
        period=6.0,
        actions=[robot1_stiffness_loader, robot2_stiffness_loader],
    )

    # Data loggers (optional)
    robot1_logger = Node(
        package='omx_variable_stiffness_controller',
        executable='logger.py',
        namespace='robot1',
        name='csv_data_logger',
        parameters=[{
            'controller_name': 'robot1_variable_stiffness',
            'output_dir': '/tmp/variable_stiffness_logs/robot1',
        }],
        output='screen',
        condition=IfCondition(enable_logger),
    )

    robot2_logger = Node(
        package='omx_variable_stiffness_controller',
        executable='logger.py',
        namespace='robot2',
        name='csv_data_logger',
        parameters=[{
            'controller_name': 'robot2_variable_stiffness',
            'output_dir': '/tmp/variable_stiffness_logs/robot2',
        }],
        output='screen',
        condition=IfCondition(enable_logger),
    )

    delay_loggers = TimerAction(
        period=6.0,
        actions=[robot1_logger, robot2_logger],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(start_rviz)
    )

    return LaunchDescription([
        *declared_arguments,
        # Robot state publishers
        robot1_state_publisher,
        robot2_state_publisher,
        # Controller managers
        robot1_controller_manager,
        robot2_controller_manager,
        # Delayed controller loading
        delay_robot1_controllers,
        delay_robot2_controllers,
        # Delayed stiffness loaders
        delay_stiffness_loaders,
        # Delayed loggers
        delay_loggers,
        # RViz
        rviz_node,
    ])
