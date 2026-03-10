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
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
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
from launch.substitutions import EnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue


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
            'enable_live_plot',
            default_value='true',
            description='Start live timeseries plotter (variable_stiffness)'
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
        DeclareLaunchArgument(
            'launch_gazebo',
            default_value='true',
            description='Whether to start gzserver (use false for headless/test)'
        ),
        DeclareLaunchArgument(
            'spawn_delay',
            default_value='5.0',
            description='Delay (s) before spawning the robot in Gazebo'
        ),
        DeclareLaunchArgument(
            'stiffness_loader_delay',
            default_value='5.0',
            description='Delay (s) after controllers are active before starting the stiffness loader'
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    csv_file = LaunchConfiguration('csv_file')
    enable_logger = LaunchConfiguration('enable_logger')
    enable_live_plot = LaunchConfiguration('enable_live_plot')
    world = LaunchConfiguration('world')
    robot_namespace = LaunchConfiguration('robot_namespace')
    gui = LaunchConfiguration('gui')
    launch_gazebo = LaunchConfiguration('launch_gazebo')

    # Controller configuration file (Gazebo-specific: includes both
    # controller_manager type declarations for the Gazebo plugin and
    # controller-specific parameters for the spawner's --param-file).
    controller_config = PathJoinSubstitution([
        FindPackageShare('omx_variable_stiffness_controller'),
        'config', 'gazebo_variable_stiffness.yaml'
    ])

    # check for the gazebo_ros2_control plugin library once at
    # launch‑file evaluation time.  This avoids crashing the controller
    # manager when the package isn’t installed (e.g. a lightweight devcontainer).
    import os
    _PLUGIN_AVAILABLE = os.path.exists(
        '/opt/ros/humble/lib/libgazebo_ros2_control.so'
    )

    # Generate URDF from xacro.  Use simulation mode only when Gazebo is
    # actually launched *and* the plugin is available; otherwise fall back to
    # fake hardware so the controller_manager can start peacefully.  Xacro
    # expects unquoted true/false for boolean args, hence the PythonExpression.
    # Evaluate to a valid Python boolean expression string. Use simple
    # boolean operators so PythonExpression doesn't require an 'else'.
    sim_arg = PythonExpression([
        "('", launch_gazebo, "' == 'true') and ",
        ('True' if _PLUGIN_AVAILABLE else 'False'),
    ])
    fake_hw = PythonExpression([
        "('", launch_gazebo, "' == 'false') or ",
        ('False' if _PLUGIN_AVAILABLE else 'True'),
    ])
    urdf = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro'
        ]),
        ' use_sim:=', sim_arg,
        ' use_fake_hardware:=', fake_hw,
        ' controller_config:=', controller_config,
        ' robot_namespace:=', robot_namespace,
    ])

    # Default empty world
    default_world = PathJoinSubstitution([
        FindPackageShare('open_manipulator_x_bringup'),
        'worlds', 'empty_world.model'
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_namespace,
        parameters=[{
            'robot_description': ParameterValue(urdf, value_type=str),
            'use_sim_time': True,
        }],
        output='screen'
    )

    # Decide whether to launch a standalone controller_manager node.
    # When running under Gazebo with the ros2_control plugin the plugin
    # will create and own its own controller_manager, so starting a second
    # ros2_control_node leads to the pluginlib mismatch we observed
    # (plugin registered under GazeboSystemInterface, not
    # hardware_interface::SystemInterface).  Instead only start a
    # controller_manager if we are *not* using the Gazebo plugin.
    #
    # sim_arg is True whenever the URDF was rendered for simulation, which
    # implies use_sim==true and the plugin library existing.  _PLUGIN_AVAILABLE
    # is evaluated at launch evaluation time.
    start_cmgr = PythonExpression([
        "not (", sim_arg, " and ",
        'True' if _PLUGIN_AVAILABLE else 'False',
        ")"
    ])

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        namespace=robot_namespace,
        parameters=[
            controller_config,
            {
                'robot_description': ParameterValue(urdf, value_type=str),
                # Also pass as prefixed parameter so the controller's on_configure()
                # can retrieve it via get_parameter("robot_description") directly,
                # avoiding any executor-deadlock topic subscription.
                'variable_stiffness_controller.robot_description':
                    ParameterValue(urdf, value_type=str),
                'use_sim_time': True,
            },
        ],
        output='screen',
        condition=IfCondition(start_cmgr),
    )

    # Ensure Gazebo can find ROS plugin libraries when it starts.
    # gzserver is launched as a subprocess and may not inherit the full
    # LD_LIBRARY_PATH from the parent shell, causing libgazebo_msgs__rosidl_*
    # not found errors (exit 255).  Propagate the ROS lib paths explicitly.
    plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        [
            '/opt/ros/humble/lib:',
            EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value='')
        ]
    )
    ld_library_path = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        [
            '/opt/ros/humble/lib/x86_64-linux-gnu:',
            '/opt/ros/humble/lib:',
            EnvironmentVariable('LD_LIBRARY_PATH', default_value='')
        ]
    )

    # Start Gazebo server (optional)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzserver.launch.py'
            ])
        ]),
        condition=IfCondition(launch_gazebo),
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

    # Instead of firing the spawner directly after a fixed delay, wait
    # until the Gazebo `/spawn_entity` service is available. This polls
    # `ros2 service list` and only returns once the service is present,
    # making the spawn robust across slow startup times.
    wait_for_spawn = ExecuteProcess(
        cmd=['bash', '-lc',
             "until ros2 service list 2>/dev/null | grep -q '/spawn_entity'; do sleep 0.5; done"],
        output='screen',
    )

    delayed_spawn = TimerAction(
        period=LaunchConfiguration('spawn_delay'),
        actions=[wait_for_spawn],
        condition=IfCondition(launch_gazebo),
    )

    # Start the actual spawn only after the wait helper finishes
    start_spawn_after_wait = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_spawn,
            on_exit=[spawn_entity],
        ),
        condition=IfCondition(launch_gazebo),
    )

    # ── Controller spawners ─────────────────────────────────────────────
    #
    # Instead of fragile fixed-delay TimerActions, we use event-driven
    # sequencing so the launch is fully automatic regardless of how long
    # Gazebo takes to start:
    #
    #   spawn_entity exits → load JSB → JSB spawner exits → load VS ctrl
    #
    # Each spawner uses --controller-manager-timeout 120 so it patiently
    # waits for the Gazebo-internal controller_manager to appear.
    # ──────────────────────────────────────────────────────────────────

    _CM_TIMEOUT = '120'  # seconds to wait for controller_manager

    # --- Gazebo path: event-driven after spawn_entity ---
    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', ['/', robot_namespace, '/controller_manager'],
            '--param-file', controller_config,
            '--controller-manager-timeout', _CM_TIMEOUT,
        ],
        output='screen',
    )

    load_vs_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'variable_stiffness_controller',
            '--controller-manager', ['/', robot_namespace, '/controller_manager'],
            '--param-file', controller_config,
            '--controller-manager-timeout', _CM_TIMEOUT,
        ],
        output='screen',
    )

    # Chain: spawn_entity finishes → start JSB spawner
    start_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_jsb],
        ),
        condition=IfCondition(launch_gazebo),
    )

    # Chain: JSB spawner finishes → start VS controller spawner
    start_vs_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=load_jsb,
            on_exit=[load_vs_controller],
        ),
    )

    # --- Headless / fake-hardware path ---
    load_jsb_headless = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', ['/', robot_namespace, '/controller_manager'],
            '--param-file', controller_config,
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    load_vs_controller_headless = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'variable_stiffness_controller',
            '--controller-manager', ['/', robot_namespace, '/controller_manager'],
            '--param-file', controller_config,
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    # Headless: short fixed delay then event-chain
    delay_controllers_headless = TimerAction(
        period=3.0,
        actions=[load_jsb_headless],
        condition=UnlessCondition(launch_gazebo),
    )

    start_vs_headless_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=load_jsb_headless,
            on_exit=[load_vs_controller_headless],
        ),
        condition=UnlessCondition(launch_gazebo),
    )

    # (previous activation helper removed)

    # Stiffness profile loader node (optional) — starts after VS controller is up
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

    # Start the stiffness loader after VS controller is active
    start_stiffness_after_vs = RegisterEventHandler(
        OnProcessExit(
            target_action=load_vs_controller,
            on_exit=[
                TimerAction(
                    period=LaunchConfiguration('stiffness_loader_delay'),
                    actions=[stiffness_loader],
                ),
            ],
        ),
    )

    # Data logger node (optional)
    logger_node = Node(
        package='omx_variable_stiffness_controller',
        executable='logger.py',
        namespace=robot_namespace,
        name='csv_data_logger',
        parameters=[{
            'controller_name': 'variable_stiffness_controller',
            'output_dir': '/tmp/variable_stiffness_logs/single_gazebo',
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

    _d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        if os.path.isfile(os.path.join(_d, 'tools', 'live_plot_logs.py')):
            break
        _d = os.path.dirname(_d)
    _live_plot_script = os.path.join(_d, 'tools', 'live_plot_logs.py')
    live_plot = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['python3', _live_plot_script,
                 '--controller', 'variable_stiffness',
                 '--namespace', ['/', robot_namespace, '/variable_stiffness_controller']],
            output='screen',
            condition=IfCondition(enable_live_plot),
        )],
    )

    return LaunchDescription([
        *declared_arguments,
        set_libgl_sw,
        plugin_path,
        ld_library_path,
        # Core nodes
        robot_state_publisher,
        ros2_control_node,
        # Gazebo
        gazebo_server,
        gazebo_client,
        delayed_spawn,
        start_spawn_after_wait,
        # Event-driven controller bringup (Gazebo path)
        start_jsb_after_spawn,
        start_vs_after_jsb,
        # Event-driven controller bringup (headless path)
        delay_controllers_headless,
        start_vs_headless_after_jsb,
        # Post-controller actions
        start_stiffness_after_vs,
        delay_logger,
        live_plot,
        rviz_node,
    ])
