#!/bin/bash
set -e
ROBOT=$1
CONDITION=$2
TIMEOUT=$3
X=${4:-0.0}
Y=${5:-0.0}
Z=${6:-0.0}
YAW=${7:-0.0}

if [ -z "$ROBOT" ] || [ -z "$CONDITION" ] || [ -z "$TIMEOUT" ]; then
  echo "Usage: $0 <robot-name> <topic-or-service> <timeout> [x y z yaw]" >&2
  exit 1
fi

echo "Waiting for $CONDITION (timeout ${TIMEOUT}s)..."

if [ "$CONDITION" = "/gazebo/model_states" ]; then
  # wait for gazebo simulation to publish model states
  ENDTIME=$((SECONDS + TIMEOUT))
  condition_met=false
  while [ $SECONDS -le $ENDTIME ]; do
    if ros2 service call /get_model_list gazebo_msgs/srv/GetModelList >/dev/null 2>&1; then
      echo "Condition satisfied: /get_model_list is available (Gazebo running)."
      condition_met=true
      break
    fi
    sleep 0.5
  done
  if [ "$condition_met" != true ]; then
    echo "Timed out waiting for /get_model_list (Gazebo not ready)." >&2
    exit 2
  fi
else
  # general topic readiness check (ros2 topic wait may be absent)
  ENDTIME=$((SECONDS + TIMEOUT))
  while [ $SECONDS -le $ENDTIME ]; do
    if ros2 topic echo "$CONDITION" --once >/dev/null 2>&1; then
      echo "Condition satisfied: topic $CONDITION is publishing."
      break
    fi
    sleep 0.5
  done
  if [ $SECONDS -gt $ENDTIME ]; then
    echo "Timed out waiting for topic $CONDITION." >&2
    exit 2
  fi
fi

echo "Spawning $ROBOT at ($X, $Y, $Z)..."

# Determine xacro path from the open_manipulator_x_description package
PACKAGE_PREFIX=$(ros2 pkg prefix open_manipulator_x_description 2>/dev/null || true)
if [ -z "$PACKAGE_PREFIX" ]; then
  echo "Could not find package open_manipulator_x_description" >&2
  exit 1
fi
URDF_XACRO="$PACKAGE_PREFIX/share/open_manipulator_x_description/urdf/open_manipulator_x_robot.urdf.xacro"
if [ ! -f "$URDF_XACRO" ]; then
  echo "Could not find xacro file at $URDF_XACRO" >&2
  exit 1
fi

# Render URDF from xacro with simulated plugin parameters
TEMP_URDF=$(mktemp /tmp/${ROBOT}_open_manipulator_x.XXXXXX.urdf)
trap 'rm -f "$TEMP_URDF"' EXIT
xacro "$URDF_XACRO" \
  use_sim:=true \
  use_fake_hardware:=false \
  robot_namespace:=$ROBOT > "$TEMP_URDF"

# Spawn entity in Gazebo
set +e
ros2 run gazebo_ros spawn_entity.py -entity $ROBOT -file "$TEMP_URDF" -x $X -y $Y -z $Z -R 0 -P 0 -Y $YAW
EXIT_CODE=$?
set -e
if [ $EXIT_CODE -ne 0 ]; then
  echo "Failed to spawn entity $ROBOT (exit $EXIT_CODE)." >&2
  exit $EXIT_CODE
fi
