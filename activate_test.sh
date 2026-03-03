#!/bin/bash
set -e
cd /workspaces/omx_ros2/ws
. install/setup.bash
# start gazebo simulation
ros2 launch omx_variable_stiffness_controller gazebo_variable_stiffness.launch.py gui:=false launch_gazebo:=true spawn_delay:=0.1 controller_delay:=3.0 stiffness_loader_delay:=4.0 enable_logger:=false &> /tmp/real_gazebo4.log &
GZPID=$!
echo "launched gazebo pid $GZPID"
sleep 20
# list controllers
ros2 service call /omx/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}' > /tmp/lst.txt 2>&1 || true
# attempt to activate controller
ros2 service call /omx/controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{start_controllers: ['variable_stiffness_controller'], stop_controllers: [], strictness: 1}" >> /tmp/lst.txt 2>&1 || true
# list again
ros2 service call /omx/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}' >> /tmp/lst.txt 2>&1 || true
# cleanup
kill $GZPID
wait $GZPID || true
# show results
cat /tmp/lst.txt
cat /tmp/real_gazebo4.log
