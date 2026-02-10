#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

if [[ -z "${ROS_DISTRO:-}" ]] && [[ -f "/opt/ros/humble/setup.bash" ]]; then
  # Best-effort environment setup when not already sourced.
  set +u
  source "/opt/ros/humble/setup.bash"
  set -u
fi

if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  set +u
  source "${WS_ROOT}/install/setup.bash"
  set -u
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 CLI not found. Source ROS2 and workspace setup first."
  exit 1
fi

function check_controller() {
  local ns="$1"
  local cm="/${ns}/controller_manager"
  local gravity_ctrl="${ns}_gravity_comp"
  echo "== Controllers (${cm}) =="
  ros2 control list_controllers -c "${cm}" || return 1
  local ok_gravity
  ok_gravity=$(ros2 control list_controllers -c "${cm}" | grep -E "${gravity_ctrl}" | grep -E "active" || true)
  if [[ -z "${ok_gravity}" ]]; then
    echo "FAIL: ${gravity_ctrl} is not active on ${cm}"
    return 1
  fi
  echo "PASS: ${gravity_ctrl} active on ${cm}"
}

function check_interfaces() {
  local ns="$1"
  local cm="/${ns}/controller_manager"
  echo "== Hardware interfaces (${cm}) =="
  ros2 control list_hardware_interfaces -c "${cm}" || return 1
  local ok
  ok=$(ros2 control list_hardware_interfaces -c "${cm}" | grep -E "joint[1-4]/effort" | grep -E "available" || true)
  if [[ -z "${ok}" ]]; then
    echo "FAIL: effort command interfaces not available on ${cm}"
    return 1
  fi
  echo "PASS: effort command interfaces available on ${cm}"
}

function check_robot_description() {
  local ns="$1"
  local node="/${ns}/robot_state_publisher"
  echo "== robot_description (${node}) =="
  local desc
  desc=$(ros2 param get "${node}" robot_description 2>/dev/null || true)
  if [[ -z "${desc}" ]] || [[ "${desc}" == *"not set"* ]]; then
    echo "FAIL: robot_description is missing for ${node}"
    return 1
  fi
  echo "PASS: robot_description available for ${node}"
}

function check_effort() {
  local topic="$1"
  local label="$2"
  echo "== Effort (${label}) =="
  python3 - <<'PY' "${topic}"
import re
import subprocess
import sys

topic = sys.argv[1]
try:
  out = subprocess.check_output(
    ["ros2", "topic", "echo", topic, "--once"],
    text=True,
    stderr=subprocess.STDOUT,
  )
except subprocess.CalledProcessError as exc:
  print("FAIL: could not read", topic)
  print(exc.output)
  sys.exit(1)

# Extract the effort list block from ros2 topic echo output.
lines = out.splitlines()
effort_lines = []
in_effort = False
for line in lines:
  if line.strip().startswith("effort:"):
    in_effort = True
    continue
  if in_effort:
    if re.match(r"^\s*\w+:", line):
      break
    effort_lines.append(line)

values = []
for line in effort_lines:
  values += re.findall(r"[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?", line)

if not values:
  print("FAIL: no effort values found in", topic)
  sys.exit(1)

nonzero = any(abs(float(x)) > 1e-4 for x in values)
if not nonzero:
  print("FAIL: all effort values are near zero in", topic)
  sys.exit(1)

print("PASS: nonzero effort in", topic)
PY
}

function check_nodes() {
  echo "== Nodes =="
  local nodes
  nodes=$(ros2 node list)
  if echo "${nodes}" | grep -q "/rviz2"; then
    echo "PASS: rviz2 node is running"
  else
    echo "WARN: rviz2 node not found"
  fi
  if echo "${nodes}" | grep -qi "gazebo"; then
    echo "PASS: gazebo nodes detected"
  else
    echo "WARN: gazebo nodes not found"
  fi
}

failures=0

check_nodes || true
check_robot_description robot1 || failures=$((failures + 1))
check_robot_description robot2 || failures=$((failures + 1))
check_controller robot1 || failures=$((failures + 1))
check_controller robot2 || failures=$((failures + 1))
check_interfaces robot1 || failures=$((failures + 1))
check_interfaces robot2 || failures=$((failures + 1))
check_effort /robot1/joint_states robot1 || failures=$((failures + 1))
check_effort /robot2/joint_states robot2 || failures=$((failures + 1))

if [[ "${failures}" -gt 0 ]]; then
  echo "\nTESTS FAILED: ${failures} issue(s) detected."
  exit 1
fi

echo "\nALL TESTS PASSED."
