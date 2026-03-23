#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_root = get_package_share_path('omx_variable_stiffness_controller')

    # Environment variables for container stability
    env_actions = [
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('SDL_AUDIODRIVER', 'dummy'),
        SetEnvironmentVariable('GAZEBO_HEADLESS_RENDERING', '1'),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
            '/opt/ros/humble/lib:/opt/ros/humble/lib/gazebo_ros'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',
            str(get_package_share_path('open_manipulator_x_description') / 'models')),
        SetEnvironmentVariable('LD_LIBRARY_PATH',
            f"/opt/ros/humble/lib:{os.environ.get('LD_LIBRARY_PATH', '')}"),
    ]

    # URDF descriptions (xacro)
    urdf_robot1 = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare('open_manipulator_x_description'), 'urdf', 'open_manipulator_x_robot.urdf.xacro']),
        ' use_sim:=true use_fake_hardware:=false robot_namespace:=robot1'
    ])
    urdf_robot2 = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare('open_manipulator_x_description'), 'urdf', 'open_manipulator_x_robot.urdf.xacro']),
        ' use_sim:=true use_fake_hardware:=false robot_namespace:=robot2'
    ])

    robot1_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='robot1', parameters=[{'robot_description': urdf_robot1, 'use_sim_time': True}], output='screen'
    )
    robot2_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='robot2', parameters=[{'robot_description': urdf_robot2, 'use_sim_time': True}], output='screen'
    )

    # Gazebo server & client
    world_path = os.path.join(pkg_root, 'worlds', 'empty.world')
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])
        ]),
        launch_arguments={'world': world_path, 'verbose': 'false'}.items()
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'])
        ]),
        condition=IfCondition('true')   # set to 'false' if you don't want GUI
    )

    # Spawn robots using wait_and_spawn.sh
    wait_and_spawn_script = os.path.join(pkg_root, 'scripts', 'wait_and_spawn.sh')
    spawn_robot1 = ExecuteProcess(
        cmd=['bash', wait_and_spawn_script, 'robot1', '/gazebo/model_states', '120'],
        output='screen'
    )
    spawn_robot2 = ExecuteProcess(
        cmd=['bash', wait_and_spawn_script, 'robot2', '/robot1/joint_states', '120'],
        output='screen'
    )

    # Controller spawners
    load_jsb_r1 = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/robot1/controller_manager',
                   '--controller-manager-timeout', '120'],
        output='screen'
    )
    load_vs_r1 = Node(
        package='controller_manager', executable='spawner',
        arguments=['robot1_variable_stiffness', '--controller-manager', '/robot1/controller_manager',
                   '--param-file', os.path.join(pkg_root, 'config', 'robot1_gazebo_variable_stiffness.yaml'),
                   '--controller-manager-timeout', '120'],
        output='screen'
    )
    load_jsb_r2 = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/robot2/controller_manager',
                   '--controller-manager-timeout', '120'],
        output='screen'
    )
    load_vs_r2 = Node(
        package='controller_manager', executable='spawner',
        arguments=['robot2_variable_stiffness', '--controller-manager', '/robot2/controller_manager',
                   '--param-file', os.path.join(pkg_root, 'config', 'robot2_gazebo_variable_stiffness.yaml'),
                   '--controller-manager-timeout', '120'],
        output='screen'
    )

    # Chain controllers after spawn
    chain_r1 = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot1, on_exit=[load_jsb_r1, load_vs_r1])
    )
    chain_r2 = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot2, on_exit=[load_jsb_r2, load_vs_r2])
    )

    return LaunchDescription([
        LogInfo(msg='[dual_gazebo_vs] Starting with container‑safe environment'),
        *env_actions,
        robot1_state_publisher,
        robot2_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_robot1,
        spawn_robot2,
        chain_r1,
        chain_r2,
    ])
# #!/usr/bin/env python3
# """
# Dual Open Manipulator X with Variable Stiffness Control in Gazebo Simulation.

# This launch file starts two robot instances (robot1, robot2) in Gazebo with

# - Gazebo plugin path and rendering shims for container compatibility
# - Explicit timing for spawn/controller-activation ordering
# - External helper script [spawn_robot] to gate robot spawn on topic/service readiness
# """

# import os

# from ament_index_python.packages import get_package_share_path
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler, SetEnvironmentVariable, TimerAction
# from launch.conditions import IfCondition
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     # Settings
#     disable_gui = False

#     # Paths
#     pkg_root = get_package_share_path('omx_variable_stiffness_controller')
#     use_sim = 'true'

#     # Environment / stability hints
#     env_actions = [
#         SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
#         SetEnvironmentVariable('GAZEBO_HEADLESS_RENDERING', '1'),
#         SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
#             '/workspaces/omx_ros2/ws/build/gazebo_ros2_control:/workspaces/omx_ros2/ws/install/gazebo_ros2_control/lib:/opt/ros/humble/lib'),
#         SetEnvironmentVariable('LD_LIBRARY_PATH',
#             '/workspaces/omx_ros2/ws/build/gazebo_ros2_control:/workspaces/omx_ros2/ws/install/gazebo_ros2_control/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:' + os.environ.get('LD_LIBRARY_PATH','')),
#         SetEnvironmentVariable('GAZEBO_MODEL_PATH',
#             os.path.join(get_package_share_path('open_manipulator_x_description'), 'models')),
#     ]

#     plugin_warning = LogInfo(msg='[dual_gazebo_vs] gazebo_ros2_control plugin path adjusted (container-safe)')

#     urdf_robot1 = Command([
#         FindExecutable(name='xacro'), ' ',
#         PathJoinSubstitution([FindPackageShare('open_manipulator_x_description'), 'urdf', 'open_manipulator_x_robot.urdf.xacro']),
#         ' use_sim:=true',
#         ' use_fake_hardware:=false',
#         ' robot_namespace:=robot1'
#     ])

#     urdf_robot2 = Command([
#         FindExecutable(name='xacro'), ' ',
#         PathJoinSubstitution([FindPackageShare('open_manipulator_x_description'), 'urdf', 'open_manipulator_x_robot.urdf.xacro']),
#         ' use_sim:=true',
#         ' use_fake_hardware:=false',
#         ' robot_namespace:=robot2'
#     ])

#     robot1_state_publisher = Node(
#         package='robot_state_publisher', executable='robot_state_publisher', namespace='robot1',
#         parameters=[{'robot_description': urdf_robot1, 'use_sim_time': True}],
#         output='screen',
#     )

#     robot2_state_publisher = Node(
#         package='robot_state_publisher', executable='robot_state_publisher', namespace='robot2',
#         parameters=[{'robot_description': urdf_robot2, 'use_sim_time': True}],
#         output='screen',
#     )

#     # Gazebo server + client with gazebo_ros launch wrappers to expose proper services
#     world_path = os.path.join(pkg_root, 'worlds', 'empty.world')

#     gazebo_server = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             PathJoinSubstitution([
#                 FindPackageShare('gazebo_ros'),
#                 'launch',
#                 'gzserver.launch.py',
#             ])
#         ]),
#         launch_arguments={
#             'world': world_path,
#             'verbose': 'false',
#         }.items(),
#     )

#     gazebo_client = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             PathJoinSubstitution([
#                 FindPackageShare('gazebo_ros'),
#                 'launch',
#                 'gzclient.launch.py',
#             ])
#         ]),
#         condition=IfCondition('true' if not disable_gui else 'false'),
#     )

#     # Gazebo spawner nodes (external helper script, robust wait conditions)
#     spawn_robot_path = '/workspaces/omx_ros2/ws/src/omx_variable_stiffness_controller/scripts/spawn_robot.py'

#     spawn_robot1 = ExecuteProcess(
#         cmd=['python3', spawn_robot_path,
#              '--robot', 'robot1',
#              '--topic', '/gazebo/model_states',
#              '--timeout', '120',
#              '--urdf_topic', '/robot1/robot_description',
#              '--ns', 'robot1',
#              '--x', '0.0',
#              '--y', '0.3',
#              '--z', '0.0'],
#         output='screen',
#     )

#     spawn_robot2 = ExecuteProcess(
#         cmd=['python3', spawn_robot_path,
#              '--robot', 'robot2',
#              '--wait_controller', 'robot1',
#              '--timeout', '120',
#              '--urdf_topic', '/robot2/robot_description',
#              '--ns', 'robot2',
#              '--x', '0.0',
#              '--y', '-0.3',
#              '--z', '0.0'],
#         output='screen',
#     )

#     # controller manager spawners (same as previous chaining, but timed for reliability)
#     load_jsb_r1 = Node(
#         package='controller_manager', executable='spawner',
#         arguments=['joint_state_broadcaster', '--controller-manager', '/robot1/controller_manager', '--controller-manager-timeout', '120'],
#         output='screen',
#     )
#     load_vs_r1 = Node(
#         package='controller_manager', executable='spawner',
#         arguments=['robot1_variable_stiffness', '--controller-manager', '/robot1/controller_manager', '--param-file', os.path.join(pkg_root, 'config', 'robot1_gazebo_variable_stiffness.yaml'), '--controller-manager-timeout', '120'],
#         output='screen',
#     )
#     load_jsb_r2 = Node(
#         package='controller_manager', executable='spawner',
#         arguments=['joint_state_broadcaster', '--controller-manager', '/robot2/controller_manager', '--controller-manager-timeout', '120'],
#         output='screen',
#     )
#     load_vs_r2 = Node(
#         package='controller_manager', executable='spawner',
#         arguments=['robot2_variable_stiffness', '--controller-manager', '/robot2/controller_manager', '--param-file', os.path.join(pkg_root, 'config', 'robot2_gazebo_variable_stiffness.yaml'), '--controller-manager-timeout', '120'],
#         output='screen',
#     )

#     # sequencing actions using timers for deterministic startup
#     delay_spawn_r1 = TimerAction(period=8.0, actions=[spawn_robot1])
#     delay_jsb_r1 = TimerAction(period=20.0, actions=[load_jsb_r1])
#     delay_vs_r1 = TimerAction(period=35.0, actions=[load_vs_r1])
#     delay_spawn_r2 = TimerAction(period=60.0, actions=[spawn_robot2])
#     delay_jsb_r2 = TimerAction(period=75.0, actions=[load_jsb_r2])
#     delay_vs_r2 = TimerAction(period=90.0, actions=[load_vs_r2])

#     # optional diagnostics in case of failure: publish random calcs? (not needed)

#     return LaunchDescription([
#         plugin_warning,
#         *env_actions,
#         robot1_state_publisher,
#         robot2_state_publisher,
#         gazebo_server,
#         gazebo_client,
#         delay_spawn_r1,
#         delay_jsb_r1,
#         delay_vs_r1,
#         delay_spawn_r2,
#         delay_jsb_r2,
#         delay_vs_r2,
#     ])


# if __name__ == '__main__':
#     generate_launch_description()
