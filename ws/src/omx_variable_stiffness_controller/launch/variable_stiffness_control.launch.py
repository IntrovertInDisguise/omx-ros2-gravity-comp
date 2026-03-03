#!/usr/bin/env python3

"""
Launch file for Variable Stiffness Controller on OpenManipulator-X.

Supports:
- Single robot
- Multi-robot (via namespace)
- Simulation (mock hardware)
- Real hardware
"""

import glob

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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
from launch_ros.parameter_descriptions import ParameterValue


def detect_serial_port():
    """Return first available serial port or fallback."""
    for pattern in ["/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*"]:
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return "/dev/ttyUSB0"


def generate_launch_description():

    default_port = detect_serial_port()

    # -------------------------
    # Launch Arguments
    # -------------------------
    declared_arguments = [

        DeclareLaunchArgument(
            "robot_namespace",
            default_value="omx",
            description="Robot namespace (use different names for dual robots)"
        ),

        DeclareLaunchArgument(
            "port",
            default_value=default_port,
            description="Serial port for hardware"
        ),

        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Use mock hardware"
        ),

        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Launch RViz2"
        ),

        DeclareLaunchArgument(
            "csv_file",
            default_value="",
            description="Optional stiffness CSV file"
        ),

        DeclareLaunchArgument(
            "enable_logger",
            default_value="true",
            description="Enable data logger"
        ),
    ]

    # -------------------------
    # Configurations
    # -------------------------
    robot_namespace = LaunchConfiguration("robot_namespace")
    port = LaunchConfiguration("port")
    sim = LaunchConfiguration("sim")
    start_rviz = LaunchConfiguration("start_rviz")
    csv_file = LaunchConfiguration("csv_file")
    enable_logger = LaunchConfiguration("enable_logger")

    controller_config = PathJoinSubstitution([
        FindPackageShare("omx_variable_stiffness_controller"),
        "config",
        "variable_stiffness_controller.yaml",
    ])

    # -------------------------
    # URDF (Xacro)
    # -------------------------
    urdf = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("open_manipulator_x_description"),
            "urdf",
            "open_manipulator_x_robot.urdf.xacro",
        ]),
        " use_sim:=", sim,
        " use_fake_hardware:=", sim,
        " port_name:=", port,
        " controller_config:=", controller_config,
        " robot_namespace:=", robot_namespace,
    ])

    # -------------------------
    # Robot State Publisher
    # -------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_namespace,
        parameters=[{
            "robot_description": ParameterValue(urdf, value_type=str),
            "use_sim_time": PythonExpression(["'", sim, "' == 'true'"]),
        }],
        output="screen",
    )

    # -------------------------
    # Controller Manager
    # -------------------------
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        namespace=robot_namespace,
        parameters=[
            {"use_sim_time": PythonExpression(["'", sim, "' == 'true'"])},
            controller_config,
        ],
        remappings=[
            # IMPORTANT: connect controller_manager to correct robot_description
            ("~/robot_description", "robot_description"),
        ],
        output="both",
    )

    # -------------------------
    # Controller Spawners
    # -------------------------
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            ["/", robot_namespace, "/controller_manager"],
            "--param-file",
            controller_config,
        ],
        output="screen",
    )

    load_vs_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "variable_stiffness_controller",
            "--controller-manager",
            ["/", robot_namespace, "/controller_manager"],
            "--param-file",
            controller_config,
        ],
        output="screen",
    )

    delay_controllers = TimerAction(
        period=5.0,
        actions=[load_jsb, load_vs_controller],
    )

    # -------------------------
    # Optional Stiffness Loader
    # -------------------------
    stiffness_loader = Node(
        package="omx_variable_stiffness_controller",
        executable="load_stiffness.py",
        namespace=robot_namespace,
        name="stiffness_loader",
        parameters=[{
            "csv_path": csv_file,
            "controller_name": "variable_stiffness_controller",
        }],
        condition=IfCondition(
            PythonExpression(["'", csv_file, "' != ''"])
        ),
        output="screen",
    )

    delay_stiffness_loader = TimerAction(
        period=5.0,
        actions=[stiffness_loader],
    )

    # -------------------------
    # Optional Logger
    # -------------------------
    logger_node = Node(
        package="omx_variable_stiffness_controller",
        executable="logger.py",
        namespace=robot_namespace,
        name="csv_data_logger",
        parameters=[{
            "controller_name": "variable_stiffness_controller",
        }],
        condition=IfCondition(enable_logger),
        output="screen",
    )

    delay_logger = TimerAction(
        period=5.0,
        actions=[logger_node],
    )

    # -------------------------
    # Optional RViz
    # -------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{
            "use_sim_time": PythonExpression(["'", sim, "' == 'true'"]),
        }],
        condition=IfCondition(start_rviz),
        output="screen",
    )

    # -------------------------
    return LaunchDescription([
        *declared_arguments,
        robot_state_publisher,
        controller_manager,
        delay_controllers,
        delay_stiffness_loader,
        delay_logger,
        rviz_node,
    ])