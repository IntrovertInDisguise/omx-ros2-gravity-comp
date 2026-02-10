#!/bin/bash
set -e

echo "=== Setting up OMX ROS2 Workspace ==="

# Source ROS2
source /opt/ros/humble/setup.bash

# Install apt dependencies
echo "Installing apt dependencies..."
apt-get update
apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-interface \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-kdl-parser \
    libserial-dev \
    x11vnc \
    novnc

# Clone dynamixel packages if not present
cd /workspaces/omx_ros2/ws/src

if [ ! -d "dynamixel_sdk" ]; then
    echo "Cloning dynamixel_sdk..."
    git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git dynamixel_sdk
fi

if [ ! -d "dynamixel_hardware_interface" ]; then
    echo "Cloning dynamixel_hardware_interface..."
    git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
fi

# Run rosdep
echo "Running rosdep..."
rosdep update || true
rosdep install --from-paths . --ignore-src -r -y || true

# Build workspace
echo "Building workspace..."
cd /workspaces/omx_ros2/ws
colcon build --symlink-install

echo "=== Setup complete! ==="
echo "Run 'source /workspaces/omx_ros2/ws/install/setup.bash' to use the workspace"
