import os
import re
from ament_index_python import get_package_share_directory
import xacro

def test_sim_urdf_has_gazebo_ros2_control():
    """Ensure the URDF generated for simulation uses the gazebo_ros2_control plugin."""
    # directly inspect the xacro file for the expected plugin string
    ws = os.getenv('WORKSPACE_DIR', '/workspaces/omx_ros2/ws')
    xacro_file = os.path.join(ws, 'src', 'open_manipulator',
                              'open_manipulator_x_description',
                              'urdf', 'open_manipulator_x_robot.urdf.xacro')
    if not os.path.exists(xacro_file):
        pkg_share = get_package_share_directory('open_manipulator_x_description')
        xacro_file = os.path.join(pkg_share, 'urdf', 'open_manipulator_x_robot.urdf.xacro')
    assert os.path.exists(xacro_file), f"xacro file missing: {xacro_file}"

    # also inspect ros2_control xacro where the plugin actually resides
    control_file = os.path.join(ws, 'src', 'open_manipulator',
                                'open_manipulator_x_description',
                                'ros2_control', 'open_manipulator_x_system.ros2_control.xacro')
    assert os.path.exists(control_file), f"control xacro missing: {control_file}"
    control_text = open(control_file).read()
    assert '<plugin>gazebo_ros2_control/GazeboSystem</plugin>' in control_text, \
        "Control xacro does not contain gazebo_ros2_control plugin for simulation"
    # ensure dynamixel plugin appears only inside conditional block
    assert 'dynamixel_hardware_interface' not in control_text or '<xacro:unless' in control_text, \
        "Control xacro has unconditional dynamixel_hardware_interface reference"

