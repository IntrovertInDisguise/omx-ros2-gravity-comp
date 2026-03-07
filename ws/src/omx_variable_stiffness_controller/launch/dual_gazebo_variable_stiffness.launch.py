#!/usr/bin/env python3
"""
Dual Open Manipulator X with Variable Stiffness Control in Gazebo Simulation.

Launches two independent simulated robots in one Gazebo world with variable
Cartesian impedance control.  Each arm has its own independent trajectory
and stiffness profile.

Architecture:
  - When Gazebo is available (launch_gazebo:=true AND plugin present),
    each robot's URDF includes a gazebo_ros2_control plugin that creates
    its own controller_manager in the correct namespace.  No standalone
    ros2_control_node is started.
  - When Gazebo is unavailable (launch_gazebo:=false OR plugin missing),
    the URDF is rendered with use_fake_hardware:=true and standalone
    ros2_control_node processes manage the controllers.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', default_value='false',
            description='Whether to start RViz2'),
        DeclareLaunchArgument(
            'robot1_csv_file', default_value='',
            description='Path to Robot 1 stiffness profile CSV (optional)'),
        DeclareLaunchArgument(
            'robot2_csv_file', default_value='',
            description='Path to Robot 2 stiffness profile CSV (optional)'),
        DeclareLaunchArgument(
            'enable_logger', default_value='false',
            description='Whether to enable data logging for both arms'),
        DeclareLaunchArgument(
            'world', default_value='',
            description='Gazebo world file (optional)'),
        DeclareLaunchArgument(
            'launch_gazebo', default_value='true',
            description='Whether to start gzserver (false = headless fake-hw)'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Whether to start Gazebo GUI (gzclient)'),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    robot1_csv_file = LaunchConfiguration('robot1_csv_file')
    robot2_csv_file = LaunchConfiguration('robot2_csv_file')
    enable_logger = LaunchConfiguration('enable_logger')
    world = LaunchConfiguration('world')
    launch_gazebo = LaunchConfiguration('launch_gazebo')
    gui = LaunchConfiguration('gui')

    # ── Plugin detection ─────────────────────────────────────────────────
    _PLUGIN_AVAILABLE = os.path.exists(
        '/opt/ros/humble/lib/libgazebo_ros2_control.so'
    )

    # effective_gazebo: True ONLY when the user wants Gazebo AND the plugin
    # actually exists.  ALL Gazebo-specific actions are gated on this.
    effective_gazebo = PythonExpression([
        "('", launch_gazebo, "' == 'true') and ",
        ('True' if _PLUGIN_AVAILABLE else 'False'),
    ])

    # use_fake_hw: True when NOT using real Gazebo (headless or plugin absent)
    use_fake_hw = PythonExpression([
        "not (('", launch_gazebo, "' == 'true') and ",
        ('True' if _PLUGIN_AVAILABLE else 'False'),
        ")",
    ])

    # ── Controller configs (Gazebo-tuned) ────────────────────────────────
    controller_config_r1 = PathJoinSubstitution([
        FindPackageShare('omx_variable_stiffness_controller'),
        'config', 'robot1_gazebo_variable_stiffness.yaml'
    ])
    controller_config_r2 = PathJoinSubstitution([
        FindPackageShare('omx_variable_stiffness_controller'),
        'config', 'robot2_gazebo_variable_stiffness.yaml'
    ])

    # ── URDFs via xacro ──────────────────────────────────────────────────
    urdf_robot1 = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro'
        ]),
        ' use_sim:=', effective_gazebo,
        ' use_fake_hardware:=', use_fake_hw,
        ' prefix:=robot1_',
        ' robot_namespace:=/robot1',
        ' controller_config:=', controller_config_r1,
    ])

    urdf_robot2 = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro'
        ]),
        ' use_sim:=', effective_gazebo,
        ' use_fake_hardware:=', use_fake_hw,
        ' prefix:=robot2_',
        ' robot_namespace:=/robot2',
        ' controller_config:=', controller_config_r2,
    ])

    # ── RViz config ──────────────────────────────────────────────────────
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('omx_dual_bringup'),
        'rviz', 'dual_robots.rviz'
    ])

    # ── Robot State Publishers ───────────────────────────────────────────
    # use_sim_time must match whether we're actually using Gazebo sim time.
    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot1',
        parameters=[{
            'robot_description': ParameterValue(urdf_robot1, value_type=str),
            'use_sim_time': _PLUGIN_AVAILABLE,
            'frame_prefix': 'robot1_',
        }],
        output='screen',
    )

    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot2',
        parameters=[{
            'robot_description': ParameterValue(urdf_robot2, value_type=str),
            'use_sim_time': _PLUGIN_AVAILABLE,
            'frame_prefix': 'robot2_',
        }],
        output='screen',
    )

    # ── Warning when plugin is missing ───────────────────────────────────
    plugin_warning = LogInfo(
        msg='[dual_gazebo_vs] gazebo_ros2_control plugin NOT found — '
            'falling back to fake hardware (headless mode)',
        condition=UnlessCondition(effective_gazebo),
    )

    # ── Gazebo environment ───────────────────────────────────────────────
    plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        ['/opt/ros/humble/lib:',
         EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value='')]
    )
    ld_library_path = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        ['/opt/ros/humble/lib/x86_64-linux-gnu:',
         '/opt/ros/humble/lib:',
         EnvironmentVariable('LD_LIBRARY_PATH', default_value='')]
    )
    set_libgl_sw = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    # ── Gazebo server + client ───────────────────────────────────────────
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzserver.launch.py'
            ])
        ]),
        condition=IfCondition(effective_gazebo),
        launch_arguments={'verbose': 'false'}.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzclient.launch.py'
            ])
        ]),
        condition=IfCondition(PythonExpression([
            "('", gui, "' == 'true') and ",
            ('True' if _PLUGIN_AVAILABLE else 'False'),
        ])),
    )

    # ── Spawn robots in Gazebo ───────────────────────────────────────────
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot1/robot_description',
            '-entity', 'robot1',
            '-x', '0.0', '-y', '0.3', '-z', '0.0',
            '-robot_namespace', 'robot1',
        ],
        output='screen',
    )

    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot2/robot_description',
            '-entity', 'robot2',
            '-x', '0.0', '-y', '-0.3', '-z', '0.0',
            '-robot_namespace', 'robot2',
        ],
        output='screen',
    )

    # Delay spawns to let gzserver start
    delay_spawn_r1 = TimerAction(
        period=3.0,
        actions=[spawn_robot1],
        condition=IfCondition(effective_gazebo),
    )

    # ── Controller spawners (Gazebo path) ────────────────────────────────
    #
    # CRITICAL: Both Gazebo ros2_control plugins run inside the same
    # gzserver process.  If robot2 is spawned while robot1's controllers
    # are still being configured, the plugin initialization for robot2
    # contaminates the namespace context and robot1's controller ends up
    # with robot2's parameters (causing "Parameter 'joints' is empty").
    #
    # Solution: Fully serialize the bringup chain:
    #   spawn_r1 → jsb_r1 → vs_r1 → spawn_r2 → jsb_r2 → vs_r2
    # This ensures each robot's plugin is fully initialised before the
    # next robot is spawned.
    _CM_TIMEOUT = '120'

    load_jsb_r1 = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/robot1/controller_manager',
                   '--controller-manager-timeout', _CM_TIMEOUT],
        output='screen',
    )
    load_vs_r1 = Node(
        package='controller_manager', executable='spawner',
        arguments=['robot1_variable_stiffness',
                   '--controller-manager', '/robot1/controller_manager',
                   '--param-file', controller_config_r1,
                   '--controller-manager-timeout', _CM_TIMEOUT],
        output='screen',
    )

    load_jsb_r2 = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/robot2/controller_manager',
                   '--controller-manager-timeout', _CM_TIMEOUT],
        output='screen',
    )
    load_vs_r2 = Node(
        package='controller_manager', executable='spawner',
        arguments=['robot2_variable_stiffness',
                   '--controller-manager', '/robot2/controller_manager',
                   '--param-file', controller_config_r2,
                   '--controller-manager-timeout', _CM_TIMEOUT],
        output='screen',
    )

    # Fully serialized Gazebo event chain:
    #   spawn_r1 exits → load jsb_r1
    start_jsb_r1_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot1, on_exit=[load_jsb_r1]),
        condition=IfCondition(effective_gazebo),
    )
    #   jsb_r1 exits → load vs_r1
    start_vs_r1_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=load_jsb_r1, on_exit=[load_vs_r1]),
        condition=IfCondition(effective_gazebo),
    )
    #   vs_r1 exits → NOW spawn robot2 (robot1 fully up)
    spawn_r2_after_r1_complete = RegisterEventHandler(
        OnProcessExit(target_action=load_vs_r1, on_exit=[spawn_robot2]),
        condition=IfCondition(effective_gazebo),
    )
    #   spawn_r2 exits → load jsb_r2
    start_jsb_r2_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot2, on_exit=[load_jsb_r2]),
        condition=IfCondition(effective_gazebo),
    )
    #   jsb_r2 exits → load vs_r2
    start_vs_r2_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=load_jsb_r2, on_exit=[load_vs_r2]),
        condition=IfCondition(effective_gazebo),
    )

    # ── Standalone CMs (headless / fake-hw path) ─────────────────────────
    # Pass robot_description as both top-level param and controller-prefixed
    # param so on_configure() can read it without a topic subscription
    # (avoids executor deadlock).
    ros2_control_r1 = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        namespace='robot1',
        parameters=[
            controller_config_r1,
            {'robot_description': ParameterValue(urdf_robot1, value_type=str),
             'robot1_variable_stiffness.robot_description':
                 ParameterValue(urdf_robot1, value_type=str),
             'use_sim_time': False},
        ],
        output='screen',
        condition=UnlessCondition(effective_gazebo),
    )

    ros2_control_r2 = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        namespace='robot2',
        parameters=[
            controller_config_r2,
            {'robot_description': ParameterValue(urdf_robot2, value_type=str),
             'robot2_variable_stiffness.robot_description':
                 ParameterValue(urdf_robot2, value_type=str),
             'use_sim_time': False},
        ],
        output='screen',
        condition=UnlessCondition(effective_gazebo),
    )

    # Headless controller loading (event-chained)
    load_jsb_r1_headless = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/robot1/controller_manager',
                   '--controller-manager-timeout', '120'],
        output='screen',
    )
    load_vs_r1_headless = Node(
        package='controller_manager', executable='spawner',
        arguments=['robot1_variable_stiffness',
                   '--controller-manager', '/robot1/controller_manager',
                   '--param-file', controller_config_r1,
                   '--controller-manager-timeout', '120'],
        output='screen',
    )
    load_jsb_r2_headless = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/robot2/controller_manager',
                   '--controller-manager-timeout', '120'],
        output='screen',
    )
    load_vs_r2_headless = Node(
        package='controller_manager', executable='spawner',
        arguments=['robot2_variable_stiffness',
                   '--controller-manager', '/robot2/controller_manager',
                   '--param-file', controller_config_r2,
                   '--controller-manager-timeout', '120'],
        output='screen',
    )

    delay_headless_r1 = TimerAction(
        period=3.0,
        actions=[load_jsb_r1_headless],
        condition=UnlessCondition(effective_gazebo),
    )
    delay_headless_r2 = TimerAction(
        period=3.5,
        actions=[load_jsb_r2_headless],
        condition=UnlessCondition(effective_gazebo),
    )
    start_vs_r1_headless = RegisterEventHandler(
        OnProcessExit(target_action=load_jsb_r1_headless,
                      on_exit=[load_vs_r1_headless]),
        condition=UnlessCondition(effective_gazebo),
    )
    start_vs_r2_headless = RegisterEventHandler(
        OnProcessExit(target_action=load_jsb_r2_headless,
                      on_exit=[load_vs_r2_headless]),
        condition=UnlessCondition(effective_gazebo),
    )

    # ── Stiffness profile loaders (optional CSV) ─────────────────────────
    robot1_stiffness_loader = Node(
        package='omx_variable_stiffness_controller',
        executable='load_stiffness.py',
        namespace='robot1',
        name='stiffness_loader',
        parameters=[{
            'csv_path': robot1_csv_file,
            'controller_name': 'robot1_variable_stiffness',
            'use_sim_time': _PLUGIN_AVAILABLE,
        }],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", robot1_csv_file, "' != ''"])
        ),
    )

    robot2_stiffness_loader = Node(
        package='omx_variable_stiffness_controller',
        executable='load_stiffness.py',
        namespace='robot2',
        name='stiffness_loader',
        parameters=[{
            'csv_path': robot2_csv_file,
            'controller_name': 'robot2_variable_stiffness',
            'use_sim_time': _PLUGIN_AVAILABLE,
        }],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", robot2_csv_file, "' != ''"])
        ),
    )

    delay_stiffness_loaders = TimerAction(
        period=10.0,
        actions=[robot1_stiffness_loader, robot2_stiffness_loader],
    )

    # ── Data loggers (optional) ──────────────────────────────────────────
    robot1_logger = Node(
        package='omx_variable_stiffness_controller',
        executable='logger.py',
        namespace='robot1',
        name='csv_data_logger',
        parameters=[{
            'controller_name': 'robot1_variable_stiffness',
            'output_dir': '/tmp/variable_stiffness_logs/robot1',
            'use_sim_time': _PLUGIN_AVAILABLE,
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
            'use_sim_time': _PLUGIN_AVAILABLE,
        }],
        output='screen',
        condition=IfCondition(enable_logger),
    )

    delay_loggers = TimerAction(
        period=10.0,
        actions=[robot1_logger, robot2_logger],
    )

    # ── RViz ─────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': _PLUGIN_AVAILABLE}],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        *declared_arguments,
        plugin_warning,
        set_libgl_sw,
        plugin_path,
        ld_library_path,
        # Robot state publishers (always needed)
        robot1_state_publisher,
        robot2_state_publisher,
        # Gazebo path (gated on effective_gazebo) — fully serialized:
        #   gzserver → spawn_r1 → jsb_r1 → vs_r1 → spawn_r2 → jsb_r2 → vs_r2
        gazebo_server,
        gazebo_client,
        delay_spawn_r1,
        start_jsb_r1_after_spawn,
        start_vs_r1_after_jsb,
        spawn_r2_after_r1_complete,
        start_jsb_r2_after_spawn,
        start_vs_r2_after_jsb,
        # Headless / fake-hw path (gated on NOT effective_gazebo)
        ros2_control_r1,
        ros2_control_r2,
        delay_headless_r1,
        delay_headless_r2,
        start_vs_r1_headless,
        start_vs_r2_headless,
        # Post-controller
        delay_stiffness_loaders,
        delay_loggers,
        rviz_node,
    ])
