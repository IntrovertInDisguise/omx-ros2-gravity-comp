#!/bin/bash
set -e
cd /workspaces/omx_ros2/ws
. install/setup.bash
# start launch in background
ros2 launch omx_variable_stiffness_controller gazebo_variable_stiffness.launch.py gui:=false launch_gazebo:=false spawn_delay:=0.1 controller_delay:=1.0 stiffness_loader_delay:=1.0 enable_logger:=false &> /tmp/vs_launch.log &
LAUNCH_PID=$!
# wait some seconds
sleep 5
# list controllers via service
ros2 service call /omx/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers "{}" || true
# list controllers via CLI
ros2 control list_controllers --controller-manager /omx/controller_manager || true
# kill launch
kill $LAUNCH_PID
wait $LAUNCH_PID || true

# dump launch log
cat /tmp/vs_launch.log
