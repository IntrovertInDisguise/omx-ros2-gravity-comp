import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    bringup_share = FindPackageShare("open_manipulator_x_bringup").find("open_manipulator_x_bringup")
    gazebo_launch = os.path.join(bringup_share, "launch", "gazebo.launch.py")

    # RViz config in SOURCE tree (works even if not installed)
    ws_root = "/workspaces/omx_ros2/ws"
    rviz_cfg = os.path.join(
        ws_root,
        "src", "open_manipulator", "open_manipulator_x_bringup", "rviz", "open_manipulator_x.rviz"
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={"use_sim_time": use_sim_time}.items()
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_cfg],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
