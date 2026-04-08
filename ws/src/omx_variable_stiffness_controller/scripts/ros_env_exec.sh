#!/bin/bash
set -euo pipefail

# Ensure all ROS2 environment and workspace overlay variables are set
source /opt/ros/humble/setup.bash
source /workspaces/omx_ros2/ws/install/setup.bash

# Preserve plugin and graphics environment too
export LIBGL_ALWAYS_SOFTWARE=1
export SDL_AUDIODRIVER=dummy

exec "$@"
