from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    cfg = PathJoinSubstitution([
        FindPackageShare("omx_dual_bringup"),
        "config",
        "dual_mock_ros2_control.yaml",
    ])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[cfg],
        output="screen",
    )

    return LaunchDescription([ros2_control_node])
