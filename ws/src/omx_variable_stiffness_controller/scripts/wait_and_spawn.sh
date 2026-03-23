#!/bin/bash
set -e
ROBOT=$1
CONDITION=$2
TIMEOUT=$3

echo "Waiting for $CONDITION (timeout ${TIMEOUT}s)..."
ros2 topic wait --timeout $TIMEOUT $CONDITION

echo "Spawning $ROBOT..."
URDF_FILE="/workspaces/omx_ros2/ws/src/open_manipulator_x_description/urdf/open_manipulator_x_robot.urdf.xacro"
ros2 run gazebo_ros spawn_entity.py -entity $ROBOT -file $URDF_FILE -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0