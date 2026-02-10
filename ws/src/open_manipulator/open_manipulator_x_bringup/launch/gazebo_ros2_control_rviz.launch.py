import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz   = LaunchConfiguration("start_rviz")
    world        = LaunchConfiguration("world")
    controllers  = LaunchConfiguration("controllers_file")
    hold_ctrl    = LaunchConfiguration("hold_controller_name")

    bringup_share = FindPackageShare("open_manipulator_x_bringup").find("open_manipulator_x_bringup")
    desc_share    = FindPackageShare("open_manipulator_x_description").find("open_manipulator_x_description")

    # You may need to adjust this xacro path if your repo differs.
    xacro_file = os.path.join(desc_share, "urdf", "open_manipulator_x.urdf.xacro")

    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    gazebo_launch = PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "open_manipulator_x_system", "-x", "0", "-y", "0", "-z", "0.01"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Controller manager (ros2_control_node)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Spawners (delay a bit so controller_manager is up)
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    hold_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[hold_ctrl, "--controller-manager", "/controller_manager"],
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=None,  # we gate it via start_rviz using a small trick below
    )

    # Gate RViz with a TimerAction + start_rviz flag (simple, avoids extra deps)
    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([FindPackageShare("open_manipulator_x_bringup"), "worlds", "empty_world.model"]),
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=os.path.join(bringup_share, "config", "controllers.yaml"),
            description="YAML for ros2_control controllers",
        ),
        DeclareLaunchArgument(
            "hold_controller_name",
            default_value="arm_controller",
            description="Controller to hold joints against gravity (must exist in controllers.yaml)",
        ),

        robot_state_publisher,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={"world": world, "verbose": "false"}.items(),
        ),

        spawn_entity,

        # Start controller manager after the robot_description exists
        TimerAction(period=1.0, actions=[ros2_control_node]),

        # Spawn controllers after controller_manager starts
        TimerAction(period=2.0, actions=[jsb_spawner]),
        TimerAction(period=3.0, actions=[hold_spawner]),

        # Start RViz (always; if you want gating, we can add a proper IfCondition)
        TimerAction(period=0.5, actions=[rviz_launch]),
    ])
