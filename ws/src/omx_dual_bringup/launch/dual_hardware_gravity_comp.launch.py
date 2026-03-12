#!/usr/bin/env python3
#
# Dual Open Manipulator X with Gravity Compensation using Hardware
# Launches two independent robots connected via serial with proper namespace isolation
#

import os
import glob
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def detect_serial_ports():
    """
    Attempt to detect available serial ports.
    Returns a list of potential device paths.
    """
    ports = []
    
    # Check /dev/serial/by-id/ first (most reliable)
    byid = sorted(glob.glob("/dev/serial/by-id/*"))
    ports.extend(byid[:2])  # Take first two if available
    
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
    # Detect available serial ports
    detected_ports = detect_serial_ports()
    default_port1 = detected_ports[0] if len(detected_ports) > 0 else "/dev/ttyUSB0"
    default_port2 = detected_ports[1] if len(detected_ports) > 1 else "/dev/ttyUSB1"

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether to start RViz2'
        ),
        DeclareLaunchArgument(
            'enable_live_plot',
            default_value='true',
            description='Start live timeseries plotter (gravity_comp, robot1+robot2)'
        ),
        DeclareLaunchArgument(
            'robot1_port',
            default_value=default_port1,
            description='Serial port for robot1'
        ),
        DeclareLaunchArgument(
            'robot2_port',
            default_value=default_port2,
            description='Serial port for robot2'
        ),
        DeclareLaunchArgument(
            'enable_logger',
            default_value='false',
            description='Enable CSV data logging (gravity_comp)'
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    robot1_port = LaunchConfiguration('robot1_port')
    robot2_port = LaunchConfiguration('robot2_port')
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

    # Controller configuration
    controller_config_robot1 = PathJoinSubstitution([
        FindPackageShare('omx_dual_bringup'),
        'config', 'robot1_gravity_comp.yaml'
    ])

    controller_config_robot2 = PathJoinSubstitution([
        FindPackageShare('omx_dual_bringup'),
        'config', 'robot2_gravity_comp.yaml'
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

    # Load controllers with spawner Nodes.
    # Hardware init (5 servos × ~20 InitItems) takes ~12 s per arm; 13 s gives
    # a safe margin.  spawner handles retry/wait-for-CM automatically.
    load_joint_state_broadcaster_robot1 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/robot1/controller_manager'],
        output='screen',
    )

    load_gravity_comp_controller_robot1 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robot1_gravity_comp',
                   '--controller-manager', '/robot1/controller_manager'],
        output='screen',
    )

    # Load and start controllers for Robot 2
    load_joint_state_broadcaster_robot2 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/robot2/controller_manager'],
        output='screen',
    )

    load_gravity_comp_controller_robot2 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robot2_gravity_comp',
                   '--controller-manager', '/robot2/controller_manager'],
        output='screen',
    )

    # Event handlers to load controllers after controller manager starts
    load_robot1_controllers = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot1_controller_manager,
            on_start=[
                TimerAction(
                    period=13.0,
                    actions=[
                        load_joint_state_broadcaster_robot1,
                        load_gravity_comp_controller_robot1,
                    ]
                )
            ],
        )
    )

    load_robot2_controllers = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot2_controller_manager,
            on_start=[
                TimerAction(
                    period=13.0,
                    actions=[
                        load_joint_state_broadcaster_robot2,
                        load_gravity_comp_controller_robot2,
                    ]
                )
            ],
        )
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

    _d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        if os.path.isfile(os.path.join(_d, 'tools', 'live_plot_logs.py')):
            break
        _d = os.path.dirname(_d)
    _live_plot_script = os.path.join(_d, 'tools', 'live_plot_logs.py')
    enable_live_plot = LaunchConfiguration('enable_live_plot')

    live_plot = TimerAction(
        period=16.0,
        actions=[ExecuteProcess(
            cmd=['python3', _live_plot_script,
                 '--controller', 'gravity_comp',
                 '--namespace', '/robot1',
                 '--namespace2', '/robot2'],
            output='screen',
        )],
        condition=IfCondition(enable_live_plot),
    )

    # --- Logger (gravity_comp) ---
    _ws_root = _d  # reuse the walked-up root that found tools/
    _log_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    _log_dir_r1 = os.path.join(_ws_root, 'logs', 'dual_gravity_comp', _log_stamp, 'robot1')
    _log_dir_r2 = os.path.join(_ws_root, 'logs', 'dual_gravity_comp', _log_stamp, 'robot2')

    robot1_gc_logger = Node(
        package='omx_dual_bringup',
        executable='gc_logger.py',
        namespace='robot1',
        name='gc_data_logger',
        parameters=[{'output_dir': _log_dir_r1}],
        output='screen',
        condition=IfCondition(enable_logger),
    )
    robot2_gc_logger = Node(
        package='omx_dual_bringup',
        executable='gc_logger.py',
        namespace='robot2',
        name='gc_data_logger',
        parameters=[{'output_dir': _log_dir_r2}],
        output='screen',
        condition=IfCondition(enable_logger),
    )

    delay_loggers = TimerAction(
        period=16.0,
        actions=[robot1_gc_logger, robot2_gc_logger],
    )

    nodes_to_start = [
        *declared_arguments,
        robot1_state_publisher,
        robot2_state_publisher,
        robot1_controller_manager,
        robot2_controller_manager,
        load_robot1_controllers,
        load_robot2_controllers,
        rviz_node,
        live_plot,
        delay_loggers,
    ]

    return LaunchDescription(nodes_to_start)
