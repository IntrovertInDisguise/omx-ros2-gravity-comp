#!/usr/bin/env python3
"""
Dual Open Manipulator X with Variable Stiffness Control in Gazebo Simulation.

Launches two independent simulated robots with variable Cartesian impedance control.
Each arm has its own independent trajectory and stiffness profile.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether to start RViz2'
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
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Gazebo world file (optional)'
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    robot1_csv_file = LaunchConfiguration('robot1_csv_file')
    robot2_csv_file = LaunchConfiguration('robot2_csv_file')
    enable_logger = LaunchConfiguration('enable_logger')
    world = LaunchConfiguration('world')

    # Get URDF via xacro for both robots (simulation mode)
    urdf_robot1 = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro'
        ]),
        ' use_sim:=true',
        ' use_fake_hardware:=false',
        ' prefix:=robot1_',
    ])

    urdf_robot2 = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro'
        ]),
        ' use_sim:=true',
        ' use_fake_hardware:=false',
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
            'use_sim_time': True,
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
            'use_sim_time': True,
            'frame_prefix': 'robot2_'
        }],
        output='screen'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
        }.items(),
    )

    # Spawn robots in Gazebo
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot1/robot_description',
            '-entity', 'robot1',
            '-x', '0.0', '-y', '0.3', '-z', '0.0',
        ],
        output='screen'
    )

    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot2/robot_description',
            '-entity', 'robot2',
            '-x', '0.0', '-y', '-0.3', '-z', '0.0',
        ],
        output='screen'
    )

    # Controller Manager for Robot 1
    robot1_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='robot1',
        parameters=[
            {'robot_description': urdf_robot1, 'use_sim_time': True},
            controller_config_robot1
        ],
        output='both',
    )

    # Controller Manager for Robot 2
    robot2_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='robot2',
        parameters=[
            {'robot_description': urdf_robot2, 'use_sim_time': True},
            controller_config_robot2
        ],
        output='both',
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

    # Delay controller loading for Gazebo startup
    delay_robot1_controllers = TimerAction(
        period=5.0,
        actions=[load_joint_state_broadcaster_robot1, load_variable_stiffness_controller_robot1],
    )

    delay_robot2_controllers = TimerAction(
        period=5.5,
        actions=[load_joint_state_broadcaster_robot2, load_variable_stiffness_controller_robot2],
    )

    # Stiffness profile loaders
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

    delay_stiffness_loaders = TimerAction(
        period=8.0,
        actions=[robot1_stiffness_loader, robot2_stiffness_loader],
    )

    # Data loggers
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
        period=8.0,
        actions=[robot1_logger, robot2_logger],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(start_rviz)
    )

    return LaunchDescription([
        *declared_arguments,
        # Gazebo
        gazebo,
        # Robot state publishers
        robot1_state_publisher,
        robot2_state_publisher,
        # Spawn robots
        spawn_robot1,
        spawn_robot2,
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
