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

# Ensure ros2 daemon is running
if ! ros2 daemon status >/dev/null 2>&1; then
  ros2 daemon start >/dev/null 2>&1 || {
    echo "Failed to start ros2 daemon" >&2
    exit 2
  }
fi

if [ "$CONDITION" = "/gazebo/model_states" ]; then
  # Wait for Gazebo core services and model_states (event-driven)
  # Wait using /gazebo/model_states directly because /get_model_list can be unstable in some envs.
  ENDTIME=$((SECONDS + TIMEOUT))
  condition_met=false
  while [ $SECONDS -le $ENDTIME ]; do
    if ros2 topic list | grep -x '/gazebo/model_states' >/dev/null 2>&1; then
      condition_met=true
      break
    fi
    sleep 0.25
  done
  if [ "$condition_met" != true ]; then
    echo "Timed out waiting for /gazebo/model_states." >&2
    exit 2
  fi

  # make sure spawn_entity service exists before spawning the robot.
  # Older/newer gazebo_ros variants may expose /spawn_entity or /gazebo/spawn_entity.
  ENDTIME=$((SECONDS + TIMEOUT))
  condition_met=false
  while [ $SECONDS -le $ENDTIME ]; do
    if ros2 service list | grep -E -x '/spawn_entity|/gazebo/spawn_entity' >/dev/null 2>&1; then
      condition_met=true
      break
    fi
    sleep 0.25
  done
  if [ "$condition_met" != true ]; then
    echo "Timed out waiting for spawn_entity service (either /spawn_entity or /gazebo/spawn_entity)." >&2
    exit 2
  fi

  echo "Condition satisfied: /gazebo/model_states is publishing and spawn_entity service exists."
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

# Idempotency check: if the robot already exists in Gazebo, skip spawning.
# This avoids duplicate-insert errors when the world still contains the entity
# (e.g., after a previous run where gzserver died). This is a conservative
# early exit and not a final fix for gzserver crashes.
if ros2 topic echo /gazebo/model_states --once 2>/dev/null | grep -q "name:.*$ROBOT"; then
  echo "$ROBOT already present in Gazebo; skipping spawn."
  exit 0
fi

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
# Optional controller_config override can be passed as the 8th argument.
CONTROLLER_CONFIG=${8:-}
XACRO_ARGS=(use_sim:=true use_fake_hardware:=false robot_namespace:=$ROBOT)
if [ -n "$CONTROLLER_CONFIG" ]; then
  XACRO_ARGS+=(controller_config:=$CONTROLLER_CONFIG)
fi
xacro "$URDF_XACRO" "${XACRO_ARGS[@]}" > "$TEMP_URDF"

set +e
# Pass explicit robot namespace to spawn_entity to avoid any namespace
# leakage between plugin instances in the same gzserver process.
ros2 run gazebo_ros spawn_entity.py -entity $ROBOT -file "$TEMP_URDF" -x $X -y $Y -z $Z -R 0 -P 0 -Y $YAW -robot_namespace $ROBOT
EXIT_CODE=$?
set -e
if [ $EXIT_CODE -ne 0 ]; then
  echo "Failed to spawn entity $ROBOT (exit $EXIT_CODE)." >&2
  exit $EXIT_CODE
fi
