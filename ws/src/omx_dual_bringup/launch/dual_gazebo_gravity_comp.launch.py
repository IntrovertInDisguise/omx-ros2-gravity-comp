#!/usr/bin/env python3
#
# Dual Open Manipulator X with Gravity Compensation in Gazebo
# Each robot runs in its own Gazebo instance for full isolation.
#

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _robot_group(
    robot_name,
    master_uri,
    x_pos,
    y_pos,
    start_gzclient,
):
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(start_gzclient)
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{robot_name}',
        arguments=[
            '-entity', robot_name,
            '-topic', f'/{robot_name}/robot_description',
            '-x', x_pos,
            '-y', y_pos,
            '-z', '0.01',
            '-robot_namespace', robot_name,
        ],
        output='screen'
    )

    # only spawn controllers when Gazebo is actually running; the
    # controller_manager parameters already request the two controllers so
    # on fake‑hardware runs they load themselves and the spawners only add
    # extra service traffic that tends to break the headless tests.
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name=f'spawner_{robot_name}_joint_state_broadcaster',
        arguments=['joint_state_broadcaster', '--controller-manager', f'/{robot_name}/controller_manager'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
    )

    gravity_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name=f'spawner_{robot_name}_gravity_comp',
        arguments=[f'{robot_name}_gravity_comp', '--controller-manager', f'/{robot_name}/controller_manager'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
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

    return GroupAction([
        SetEnvironmentVariable('GAZEBO_MASTER_URI', master_uri),
        *_gazebo_env(),
        gazebo_server,
        gazebo_client,
        spawn_robot,
        load_jsb,
        load_gravity,
    ])


def _build_gazebo_nodes(context, *args, **kwargs):
    gazebo_mode = LaunchConfiguration('gazebo_mode').perform(context).strip().lower()
    start_gzclient = LaunchConfiguration('start_gzclient')
    robot1_x = LaunchConfiguration('robot1_x')
    robot1_y = LaunchConfiguration('robot1_y')
    robot2_x = LaunchConfiguration('robot2_x')
    robot2_y = LaunchConfiguration('robot2_y')

    if gazebo_mode == 'single':
        gazebo_server = ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        )

        gazebo_client = ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        )

        spawn_robot1 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot1',
            arguments=[
                '-entity', 'robot1',
                '-topic', '/robot1/robot_description',
                '-x', robot1_x,
                '-y', robot1_y,
                '-z', '0.01',
                '-robot_namespace', 'robot1',
            ],
            output='screen'
        )

        spawn_robot2 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot2',
            arguments=[
                '-entity', 'robot2',
                '-topic', '/robot2/robot_description',
                '-x', robot2_x,
                '-y', robot2_y,
                '-z', '0.01',
                '-robot_namespace', 'robot2',
            ],
            output='screen'
        )

        robot1_joint_state_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_robot1_joint_state_broadcaster',
            arguments=['joint_state_broadcaster', '--controller-manager', '/robot1/controller_manager'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        )

        robot1_gravity_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_robot1_gravity_comp',
            arguments=['robot1_gravity_comp', '--controller-manager', '/robot1/controller_manager'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        )

        robot2_joint_state_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_robot2_joint_state_broadcaster',
            arguments=['joint_state_broadcaster', '--controller-manager', '/robot2/controller_manager'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        )

        robot2_gravity_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_robot2_gravity_comp',
            arguments=['robot2_gravity_comp', '--controller-manager', '/robot2/controller_manager'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        )

        load_robot1_jsb = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot1,
                on_exit=[robot1_joint_state_spawner],
            )
        )

        spawn_robot2_after_robot1 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot1,
                on_exit=[spawn_robot2],
            )
        )

        load_robot1_gravity = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot1_joint_state_spawner,
                on_exit=[robot1_gravity_spawner],
            )
        )

        load_robot2_jsb = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot2,
                on_exit=[robot2_joint_state_spawner],
            )
        )

        load_robot2_gravity = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot2_joint_state_spawner,
                on_exit=[robot2_gravity_spawner],
            )
        )

        return [
            SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://127.0.0.1:11345'),
            *_gazebo_env(),
            gazebo_server,
            gazebo_client,
            spawn_robot1,
            spawn_robot2_after_robot1,
            load_robot1_jsb,
            load_robot1_gravity,
            load_robot2_jsb,
            load_robot2_gravity,
        ]

    robot1_group = _robot_group(
        'robot1',
        'http://127.0.0.1:11345',
        robot1_x,
        robot1_y,
        start_gzclient,
    )

    robot2_group = _robot_group(
        'robot2',
        'http://127.0.0.1:11346',
        robot2_x,
        robot2_y,
        start_gzclient,
    )

    return [robot1_group, robot2_group]


def _gazebo_env():
    omx_share = PathJoinSubstitution([
        FindPackageShare('open_manipulator_x_description')
    ])
    omx_parent = PathJoinSubstitution([
        FindPackageShare('open_manipulator_x_description'),
        '..'
    ])
    plugin_path = TextSubstitution(text='/opt/ros/humble/lib:')
    sep = TextSubstitution(text=':')
    gazebo_share = TextSubstitution(text='/usr/share/gazebo-11')
    gazebo_worlds = TextSubstitution(text='/usr/share/gazebo-11/worlds')
    gazebo_models = TextSubstitution(text='/usr/share/gazebo-11/models')
    return [
        UnsetEnvironmentVariable('ROS_NAMESPACE'),
        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            [plugin_path, EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value='')],
        ),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            [
                omx_share,
                sep,
                omx_parent,
                sep,
                gazebo_models,
                sep,
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            ],
        ),
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            [
                gazebo_share,
                sep,
                gazebo_worlds,
                sep,
                omx_share,
                sep,
                EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value=''),
            ],
        ),
    ]


def generate_launch_description():
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
            'launch_gazebo',
            default_value='true',
            description='Whether to start gzserver (disable for headless tests)'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Pass-through to URDF xacro: enable simulation plugin'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Pass-through to URDF xacro: use generic fake hardware'
        ),
        DeclareLaunchArgument(
            'gazebo_mode',
            default_value='single',
            description='Gazebo mode: single (one server) or dual (two servers)'
        ),
        DeclareLaunchArgument(
            'start_gzclient',
            default_value='true',
            description='Whether to start Gazebo clients'
        ),
        DeclareLaunchArgument(
            'robot1_x',
            default_value='0.0',
            description='X position of robot1'
        ),
        DeclareLaunchArgument(
            'robot1_y',
            default_value='0.5',
            description='Y position of robot1'
        ),
        DeclareLaunchArgument(
            'robot2_x',
            default_value='0.0',
            description='X position of robot2'
        ),
        DeclareLaunchArgument(
            'robot2_y',
            default_value='-0.5',
            description='Y position of robot2'
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    launch_gazebo = LaunchConfiguration('launch_gazebo')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    start_gzclient = LaunchConfiguration('start_gzclient')

    controller_config_robot1 = PathJoinSubstitution([
        FindPackageShare('omx_dual_bringup'),
        'config', 'robot1_gravity_comp.yaml'
    ])

    controller_config_robot2 = PathJoinSubstitution([
        FindPackageShare('omx_dual_bringup'),
        'config', 'robot2_gravity_comp.yaml'
    ])

    # URDF flags now directly controlled by launch arguments
    sim_arg = use_sim
    fake_hw = use_fake_hardware

    urdf_robot1 = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro'
        ]),
        ' use_sim:=', sim_arg,
        ' use_fake_hardware:=', fake_hw,
        ' controller_config:=', controller_config_robot1,
        ' robot_namespace:=robot1',
        ' prefix:=robot1_',
    ])

    urdf_robot2 = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro'
        ]),
        ' use_sim:=', sim_arg,
        ' use_fake_hardware:=', fake_hw,
        ' controller_config:=', controller_config_robot2,
        ' robot_namespace:=robot2',
        ' prefix:=robot2_',
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('omx_dual_bringup'),
        'rviz', 'dual_robots.rviz'
    ])

    from launch_ros.parameter_descriptions import ParameterValue

    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot1',
        parameters=[{
            'robot_description': ParameterValue(urdf_robot1, value_type=str),
            'use_sim_time': True,
        }],
        output='screen'
    )

    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot2',
        parameters=[{
            'robot_description': ParameterValue(urdf_robot2, value_type=str),
            'use_sim_time': True,
        }],
        output='screen'
    )

    # standalone controller_managers for headless testing
    ros2_control_r1 = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        namespace='robot1',
        parameters=[
            {'robot_description': ParameterValue(urdf_robot1, value_type=str), 'use_sim_time': True},
            controller_config_robot1,
        ],
        output='screen',
    )

    ros2_control_r2 = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        namespace='robot2',
        parameters=[
            {'robot_description': ParameterValue(urdf_robot2, value_type=str), 'use_sim_time': True},
            controller_config_robot2,
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
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
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['python3', _live_plot_script,
                 '--controller', 'gravity_comp',
                 '--namespace', '/robot1',
                 '--namespace2', '/robot2'],
            output='screen',
            condition=IfCondition(enable_live_plot),
        )],
    )

    nodes_to_start = [
        *declared_arguments,
        robot1_state_publisher,
        robot2_state_publisher,
        ros2_control_r1,
        ros2_control_r2,
        OpaqueFunction(function=_build_gazebo_nodes),
        rviz_node,
        live_plot,
    ]

    return LaunchDescription(nodes_to_start)
