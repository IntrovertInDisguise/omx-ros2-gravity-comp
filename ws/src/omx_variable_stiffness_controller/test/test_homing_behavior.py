import os
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import time

@pytest.mark.rostest
def test_homing_behavior():
    rclpy.init()
    node = rclpy.create_node('test_homing_behavior')
    
    # Listen to joint_states to verify homing
    received_joint_states = []
    def joint_state_cb(msg):
        received_joint_states.append(msg)
    sub = node.create_subscription(JointState, '/omx/joint_states', joint_state_cb, 10)

    # Wait for homing to complete (timeout 20s)
    start = time.time()
    while rclpy.ok() and time.time() - start < 20:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(received_joint_states) > 10:
            break
    
    assert len(received_joint_states) > 0, 'No joint states received during homing.'
    # Optionally, check that joint positions change from initial to homed
    positions = [msg.position for msg in received_joint_states]
    assert any(any(abs(p) > 0.01 for p in pos) for pos in positions), 'Joint positions did not change during homing.'
    node.destroy_node()
    rclpy.shutdown()
