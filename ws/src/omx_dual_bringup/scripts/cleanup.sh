#!/bin/bash
# Cleanup script to kill all ROS2/Gazebo processes before relaunching
# Usage: ./cleanup.sh

echo "Stopping ROS2/Gazebo processes..."

pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -9 robot_state_publisher 2>/dev/null
pkill -9 rviz2 2>/dev/null
pkill -9 spawner 2>/dev/null
pkill -9 -f "ros2.*controller_manager" 2>/dev/null

# Wait for processes to terminate
sleep 2

# Verify cleanup
if pgrep -f "gzserver|gzclient|rviz2" > /dev/null; then
    echo "WARNING: Some processes still running:"
    pgrep -la "gzserver|gzclient|rviz2"
else
    echo "Cleanup complete. Ready to relaunch."
fi
