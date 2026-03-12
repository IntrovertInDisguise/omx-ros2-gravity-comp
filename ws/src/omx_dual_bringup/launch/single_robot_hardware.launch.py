#!/usr/bin/env python3
"""Single robot hardware launch for gravity compensation.

Connects directly to Dynamixel servos — no Gazebo needed.
"""

import glob
import os
from datetime import datetime
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
        DeclareLaunchArgument(
            'enable_live_plot',
            default_value='true',
            description='Start live timeseries plotter (gravity_comp, /omx)'
        ),
        DeclareLaunchArgument(
            'enable_logger',
            default_value='false',
            description='Enable CSV data logging (gravity_comp)'
        ),
    ]

    port = LaunchConfiguration('port')
    start_rviz = LaunchConfiguration('start_rviz')
    enable_logger = LaunchConfiguration('enable_logger')

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

    # Load controllers with spawner Nodes after hardware has time to initialise.
    # Hardware init (5 servos × ~20 InitItem calls) takes ~12 s; use 10 s delay
    # so the spawners only fire once all state/command interfaces are available.
    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/omx/controller_manager',
        ],
        output='screen',
    )

    load_gravity = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gravity_comp_controller',
            '--controller-manager', '/omx/controller_manager',
        ],
        output='screen',
    )

    delay_controllers = TimerAction(
        period=10.0,
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

    _d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        if os.path.isfile(os.path.join(_d, 'tools', 'live_plot_logs.py')):
            break
        _d = os.path.dirname(_d)
    _live_plot_script = os.path.join(_d, 'tools', 'live_plot_logs.py')
    enable_live_plot = LaunchConfiguration('enable_live_plot')
    live_plot = TimerAction(
        period=13.0,
        actions=[ExecuteProcess(
            cmd=['python3', _live_plot_script,
                 '--controller', 'gravity_comp',
                 '--namespace', '/omx'],
            output='screen',
            condition=IfCondition(enable_live_plot),
        )],
    )

    # --- Logger (gravity_comp) ---
    _ws_root = _d  # reuse the walked-up root that found tools/
    _log_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    _log_dir = os.path.join(_ws_root, 'logs', 'single_gravity_comp', _log_stamp)

    gc_logger = Node(
        package='omx_dual_bringup',
        executable='gc_logger.py',
        namespace='omx',
        name='gc_data_logger',
        parameters=[{'output_dir': _log_dir}],
        output='screen',
        condition=IfCondition(enable_logger),
    )

    delay_logger = TimerAction(
        period=13.0,
        actions=[gc_logger],
    )

    return LaunchDescription([
        *declared_arguments,
        robot_state_publisher,
        controller_manager,
        delay_controllers,
        rviz_node,
        live_plot,
        delay_logger,
    ])
