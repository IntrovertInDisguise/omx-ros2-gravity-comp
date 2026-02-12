#!/usr/bin/env python3
"""
Single Open Manipulator X with Variable Stiffness Control in Gazebo Simulation.

This launch file starts:
- Gazebo server and client
- Robot spawner
- Robot state publisher
- Joint state broadcaster
- Variable stiffness controller
- Optional: Data logger
- Optional: RViz
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
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
            default_value='false',
            description='Whether to start RViz2'
        ),
        DeclareLaunchArgument(
            'csv_file',
            default_value='',
            description='Path to stiffness profile CSV (optional)'
        ),
        DeclareLaunchArgument(
            'enable_logger',
            default_value='true',
            description='Whether to enable data logging'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Gazebo world file (empty = default empty world)'
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='omx',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Whether to start Gazebo GUI (gzclient)'
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    csv_file = LaunchConfiguration('csv_file')
    enable_logger = LaunchConfiguration('enable_logger')
    world = LaunchConfiguration('world')
    robot_namespace = LaunchConfiguration('robot_namespace')
    gui = LaunchConfiguration('gui')

    # Controller configuration file
    controller_config = PathJoinSubstitution([
        FindPackageShare('omx_variable_stiffness_controller'),
        'config', 'variable_stiffness_controller.yaml'
    ])

    # Generate URDF from xacro (simulation mode)
    urdf = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro'
        ]),
        ' use_sim:=true',
        ' use_fake_hardware:=false',
        ' controller_config:=', controller_config,
        ' robot_namespace:=', robot_namespace,
    ])

    # Default empty world
    default_world = PathJoinSubstitution([
        FindPackageShare('open_manipulator_x_bringup'),
        'worlds', 'empty_world.model'
    ])

    # Start Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PythonExpression([
                "'", world, "' if '", world, "' != '' else '", default_world, "'"
            ]),
            'verbose': 'false',
        }.items(),
    )

    # Start Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzclient.launch.py'
            ])
        ]),
        condition=IfCondition(gui),
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_namespace,
        parameters=[{
            'robot_description': urdf,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', ['/', robot_namespace, '/robot_description'],
            '-entity', 'open_manipulator_x',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
        ],
        output='screen',
    )

    # Delay spawning until Gazebo is ready (needs time for GazeboRosFactory)
    delayed_spawn = TimerAction(
        period=8.0,
        actions=[spawn_entity],
    )

    # Load joint state broadcaster (after spawn)
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

    # Delay controller loading to ensure Gazebo + spawn are ready
    delay_controllers = TimerAction(
        period=15.0,
        actions=[load_jsb, load_vs_controller],
    )

    # Stiffness profile loader node (optional)
    stiffness_loader = Node(
        package='omx_variable_stiffness_controller',
        executable='load_stiffness.py',
        namespace=robot_namespace,
        name='stiffness_loader',
        parameters=[{
            'csv_path': csv_file,
            'controller_name': 'variable_stiffness_controller',
            'use_sim_time': True,
        }],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", csv_file, "' != ''"])
        ),
    )

    delay_stiffness_loader = TimerAction(
        period=8.0,
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
            'use_sim_time': True,
        }],
        output='screen',
        condition=IfCondition(enable_logger),
    )

    delay_logger = TimerAction(
        period=8.0,
        actions=[logger_node],
    )

    # RViz node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(start_rviz)
    )

    # Force software rendering for headless containers
    set_libgl_sw = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    return LaunchDescription([
        *declared_arguments,
        set_libgl_sw,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        delayed_spawn,
        delay_controllers,
        delay_stiffness_loader,
        delay_logger,
        rviz_node,
    ])
