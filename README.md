# Dual Open Manipulator X with Gravity Compensation & Variable Cartesian Impedance Control

This package provides launch files and configurations for running **two independent Open Manipulator X robots** with:
- **Gravity Compensation**: Passive gravity compensation for compliant manipulation
- **Variable Cartesian Impedance Control**: Time-varying stiffness/damping profiles for precise force control

The robots can be controlled via:
- **Hardware**: Two robots connected via separate serial ports through U2D2
- **Simulation**: Gazebo simulation with RViz2 visualization
- **Auto-detection**: Automatically detect hardware or fallback to simulation

## Testing Status

| Controller | Mode | Status | Notes |
|------------|------|--------|-------|
| **Gravity Compensation** | Dual Hardware | ✅ **TESTED** | Verified Feb 2026, two physical OM-X robots |
| **Gravity Compensation** | Single Hardware | ✅ **TESTED** | Verified Feb 2026 |
| **Gravity Compensation** | Dual Gazebo | ⚠️ Untested | Builds, launches, not verified |
| **Gravity Compensation** | Single Gazebo | ⚠️ Untested | Builds, launches, not verified |
| **Variable Stiffness** | Dual Hardware | 🔧 **BUILD ONLY** | Code complete, not run on hardware |
| **Variable Stiffness** | Single Hardware | 🔧 **BUILD ONLY** | Code complete, not run on hardware |
| **Variable Stiffness** | Dual Gazebo | 🔧 **BUILD ONLY** | Code complete, not run in sim |
| **Variable Stiffness** | Single Gazebo | 🔧 **BUILD ONLY** | Code complete, not run in sim |

**Legend:**
- ✅ **TESTED**: Verified working on actual hardware/simulation
- ⚠️ Untested: Code exists and builds, but not verified
- 🔧 **BUILD ONLY**: Implementation complete, compiles successfully, awaiting testing

## Overview

This repository is a ROS 2 Humble workspace for OpenMANIPULATOR-X control.

- **Primary bringup package:** `omx_dual_bringup`
- **Gravity compensation controller:** `omx_gravity_comp_controller`
- **Variable stiffness controller:** `omx_variable_stiffness_controller` (Cartesian impedance with trajectory tracking)
- **Robot description + xacro:** `open_manipulator_x_description`
- **Hardware path (Dynamixel):** `dynamixel_sdk` + `dynamixel_hardware_interface`

You can run:
- **Dual robot** (hardware or Gazebo) with namespaces `robot1` / `robot2`
- **Single robot** (hardware or Gazebo) with namespace `omx`

## Features

### Implemented & Tested (Gravity Compensation)
✅ Proper namespace isolation (`robot1` and `robot2`)
✅ Independent YAML configurations for each robot
✅ Gravity compensation controller for each robot
✅ RViz2 visualization for both robots
✅ Automatic hardware detection
✅ **Dual hardware mode verified working** (Feb 2026)

### Implemented - Awaiting Testing (Variable Stiffness)
🔧 Variable Cartesian Impedance Controller with time-varying stiffness profiles
🔧 Hardware safety limits (stiffness ≤65 N/m, damping ≤3 Ns/m for Robotis servos)
🔧 Manipulability metrics publishing (JJ^T eigenvalues, condition number, determinant)
🔧 Runtime waypoint command interface (offset + absolute modes, waypoint queue)
🔧 Support for both hardware and Gazebo simulation

## Directory Structure

```
omx_dual_bringup/
├── config/
│   ├── robot1_gravity_comp.yaml    # Robot 1 controller config
│   └── robot2_gravity_comp.yaml    # Robot 2 controller config
├── launch/
│   ├── auto_dual_gravity_comp.launch.py      # Auto-detect mode
│   ├── dual_gazebo_gravity_comp.launch.py    # Gazebo simulation
│   └── dual_hardware_gravity_comp.launch.py  # Hardware control
└── rviz/
    └── dual_robots.rviz                 # RViz configuration

omx_variable_stiffness_controller/
├── config/
│   ├── variable_stiffness_controller.yaml    # Single robot config
│   ├── robot1_variable_stiffness.yaml        # Robot 1 impedance config
│   ├── robot2_variable_stiffness.yaml        # Robot 2 impedance config
│   └── *.csv                                 # Stiffness profile files
├── launch/
│   ├── variable_stiffness_control.launch.py  # Single robot launch
│   ├── dual_hardware_variable_stiffness.launch.py
│   └── dual_gazebo_variable_stiffness.launch.py
└── scripts/
    ├── load_stiffness.py             # Load CSV profiles at runtime
    └── logger.py                      # Log controller state
```

## Setup Recipes (4 Modes)

Each mode below lists:
- **Build packages/files**: which packages you need to build (and where their build files live)
- **Key dependencies**: what needs to be installed for that mode
- **Dependency locations**: which workspace folders (and/or upstream repos) you need for that mode
- **Launch file**: what to run
- **Commands**: a minimal command list to bring it up

> Note: All commands assume you are in this repo and using ROS 2 Humble.

### Minimal Initial Setup (after cloning)

Run this once after cloning (then pick one of the 4 modes below).

```bash
# From the repo root
cd /workspaces/omx_ros2/ws

# ROS environment
source /opt/ros/humble/setup.bash

# Tools + rosdep (skip if you already have these)
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# rosdep is the easiest way to pull the correct ROS/Ubuntu packages
sudo rosdep init 2>/dev/null || true
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || true
```

Notes:
- If you are using the devcontainer, most dependencies are already installed during container setup.
- Hardware modes need USB/serial access (e.g. `/dev/ttyUSB*` or `/dev/serial/by-id/*`).

### 1) Dual Hardware Setup (2 real robots) ✅ TESTED

> ✅ **Status: VERIFIED** — Tested on two physical OpenMANIPULATOR-X robots, Feb 2026.

- **Build packages/files**
  - `omx_dual_bringup` (build files: `ws/src/omx_dual_bringup/CMakeLists.txt`, `ws/src/omx_dual_bringup/package.xml`)
  - `omx_gravity_comp_controller` (build files: `ws/src/omx_gravity_comp_controller/CMakeLists.txt`, `ws/src/omx_gravity_comp_controller/package.xml`)
  - Configs used: `ws/src/omx_dual_bringup/config/robot1_gravity_comp.yaml`, `ws/src/omx_dual_bringup/config/robot2_gravity_comp.yaml`
- **Key dependencies**
  - Dynamixel + hardware IO: `dynamixel_sdk`, `dynamixel_hardware_interface`
  - ROS 2 control stack: `ros2_control`, `ros2_controllers`, `controller_manager`
  - Description/xacro: `open_manipulator_x_description`, `xacro`, `robot_state_publisher`
  - Serial access from Python tools/scripts: `python3-serial`
- **Dependency locations (what to download/clone for this mode)**
  - From this repo (workspace folders)
    - `ws/src/omx_dual_bringup/`
    - `ws/src/omx_gravity_comp_controller/`
    - `ws/src/open_manipulator/open_manipulator_x_description/`
    - `ws/src/dynamixel_sdk/`
    - `ws/src/dynamixel_hardware_interface/`
  - Upstream sources (if you are assembling a minimal workspace from scratch)
    - OpenMANIPULATOR (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/open_manipulator.git`
    - DynamixelSDK (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git dynamixel_sdk`
    - Dynamixel HW interface (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git`
- **Launch file**
  - `ws/src/omx_dual_bringup/launch/dual_hardware_gravity_comp.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Install OS/ROS dependencies (recommended for fresh machines)
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-serial
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || true

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  dynamixel_sdk dynamixel_sdk_custom_interfaces \
  dynamixel_hardware_interface \
  omx_gravity_comp_controller omx_dual_bringup
source install/setup.bash

# Launch (auto-detects ports, or override)
ros2 launch omx_dual_bringup dual_hardware_gravity_comp.launch.py \
  robot1_port:=/dev/ttyUSB0 robot2_port:=/dev/ttyUSB1 start_rviz:=true

# Sanity checks
ros2 control list_controllers -c /robot1/controller_manager
ros2 control list_controllers -c /robot2/controller_manager
ros2 topic echo /robot1/joint_states
```

**Tuning Notes:**
- The `torque_scale` parameter in `robot*_gravity_comp.yaml` controls compensation strength (default: 200.0)
- If arms feel weak/droopy, increase `torque_scale` (try 250-400)
- If arms feel stiff or overshoot, decrease `torque_scale`
- Each robot can be tuned independently via its config file

### 2) Dual Simulation Setup (2 robots in Gazebo) ⚠️ Untested

> ⚠️ **Status: UNTESTED** — Builds and launches, but not verified in simulation.

- **Build packages/files**
  - `omx_dual_bringup` (build files: `ws/src/omx_dual_bringup/CMakeLists.txt`, `ws/src/omx_dual_bringup/package.xml`)
  - `omx_gravity_comp_controller` (build files: `ws/src/omx_gravity_comp_controller/CMakeLists.txt`, `ws/src/omx_gravity_comp_controller/package.xml`)
  - Configs used: `ws/src/omx_dual_bringup/config/robot1_gravity_comp.yaml`, `ws/src/omx_dual_bringup/config/robot2_gravity_comp.yaml`
- **Key dependencies**
  - Gazebo Classic + ROS integration: `gazebo_ros`, `gazebo_ros2_control`, `gazebo_ros_pkgs`
  - ROS 2 control stack: `controller_manager`, `ros2controlcli`
  - Description/xacro: `open_manipulator_x_description`, `xacro`, `robot_state_publisher`
- **Dependency locations (what to download/clone for this mode)**
  - From this repo (workspace folders)
    - `ws/src/omx_dual_bringup/`
    - `ws/src/omx_gravity_comp_controller/`
    - `ws/src/open_manipulator/open_manipulator_x_description/`
  - Upstream sources (if you are assembling a minimal workspace from scratch)
    - OpenMANIPULATOR (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/open_manipulator.git`
- **Launch file**
  - `ws/src/omx_dual_bringup/launch/dual_gazebo_gravity_comp.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Install OS/ROS dependencies (recommended for fresh machines)
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || true

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  omx_gravity_comp_controller omx_dual_bringup
source install/setup.bash

# Launch (single Gazebo server by default)
ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py gazebo_mode:=single start_rviz:=true

# Optional: full isolation using two Gazebo servers
# ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py gazebo_mode:=dual

# Sanity checks
ros2 control list_controllers -c /robot1/controller_manager
ros2 control list_controllers -c /robot2/controller_manager
```

### 3) Single Simulation Setup (1 robot in Gazebo) ⚠️ Untested

> ⚠️ **Status: UNTESTED** — Builds and launches, but not verified in simulation.

- **Build packages/files**
  - `omx_dual_bringup` (build files: `ws/src/omx_dual_bringup/CMakeLists.txt`, `ws/src/omx_dual_bringup/package.xml`)
  - `omx_gravity_comp_controller` (build files: `ws/src/omx_gravity_comp_controller/CMakeLists.txt`, `ws/src/omx_gravity_comp_controller/package.xml`)
  - Config used: `ws/src/omx_dual_bringup/config/single_robot_gravity_comp.yaml`
- **Key dependencies**
  - Gazebo Classic + ROS integration: `gazebo_ros`, `gazebo_ros2_control`, `gazebo_ros_pkgs`
  - ROS 2 control stack: `controller_manager`, `ros2controlcli`
  - Description/xacro: `open_manipulator_x_description`, `xacro`, `robot_state_publisher`
- **Dependency locations (what to download/clone for this mode)**
  - From this repo (workspace folders)
    - `ws/src/omx_dual_bringup/`
    - `ws/src/omx_gravity_comp_controller/`
    - `ws/src/open_manipulator/open_manipulator_x_description/`
  - Upstream sources (if you are assembling a minimal workspace from scratch)
    - OpenMANIPULATOR (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/open_manipulator.git`
- **Launch file**
  - `ws/src/omx_dual_bringup/launch/single_robot_test.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Install OS/ROS dependencies (recommended for fresh machines)
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || true

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  omx_gravity_comp_controller omx_dual_bringup
source install/setup.bash

# Launch
ros2 launch omx_dual_bringup single_robot_test.launch.py

# Sanity checks
ros2 control list_controllers -c /omx/controller_manager
ros2 topic echo /omx/joint_states
```

### 4) Single Hardware Setup (1 real robot) ✅ TESTED

> ✅ **Status: VERIFIED** — Tested on physical hardware, Feb 2026.

- **Build packages/files**
  - `omx_dual_bringup` (build files: `ws/src/omx_dual_bringup/CMakeLists.txt`, `ws/src/omx_dual_bringup/package.xml`)
  - `omx_gravity_comp_controller` (build files: `ws/src/omx_gravity_comp_controller/CMakeLists.txt`, `ws/src/omx_gravity_comp_controller/package.xml`)
  - Config used: `ws/src/omx_dual_bringup/config/single_robot_hardware_gravity_comp.yaml`
- **Key dependencies**
  - Dynamixel + hardware IO: `dynamixel_sdk`, `dynamixel_hardware_interface`
  - ROS 2 control stack: `ros2_control`, `ros2_controllers`, `controller_manager`
  - Description/xacro: `open_manipulator_x_description`, `xacro`, `robot_state_publisher`
- **Dependency locations (what to download/clone for this mode)**
  - From this repo (workspace folders)
    - `ws/src/omx_dual_bringup/`
    - `ws/src/omx_gravity_comp_controller/`
    - `ws/src/open_manipulator/open_manipulator_x_description/`
    - `ws/src/dynamixel_sdk/`
    - `ws/src/dynamixel_hardware_interface/`
  - Upstream sources (if you are assembling a minimal workspace from scratch)
    - OpenMANIPULATOR (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/open_manipulator.git`
    - DynamixelSDK (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git dynamixel_sdk`
    - Dynamixel HW interface (Humble): `git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git`
- **Launch file**
  - `ws/src/omx_dual_bringup/launch/single_robot_hardware.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Install OS/ROS dependencies (recommended for fresh machines)
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || true

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  dynamixel_sdk dynamixel_sdk_custom_interfaces \
  dynamixel_hardware_interface \
  omx_gravity_comp_controller omx_dual_bringup
source install/setup.bash

# Launch (auto-detects port, or override)
ros2 launch omx_dual_bringup single_robot_hardware.launch.py port:=/dev/ttyUSB0 start_rviz:=false

# Sanity checks
ros2 control list_controllers -c /omx/controller_manager
ros2 topic echo /omx/joint_states
```

### 5) Variable Cartesian Impedance Control (Dual Hardware) 🔧 UNTESTED

> ⚠️ **Status: BUILD ONLY** — This controller compiles and is feature-complete but has NOT been tested on actual hardware. Parameters may need tuning. Use with caution.

The variable stiffness controller provides Cartesian impedance control with time-varying stiffness/damping profiles along trajectories.

- **Build packages/files**
  - `omx_variable_stiffness_controller` (build files: `ws/src/omx_variable_stiffness_controller/CMakeLists.txt`)
  - Configs: `ws/src/omx_variable_stiffness_controller/config/robot*_variable_stiffness.yaml`
- **Key dependencies**
  - KDL for kinematics: `liborocos-kdl-dev`, `ros-humble-kdl-parser`
  - Eigen for matrix operations: `libeigen3-dev`
  - Same hardware dependencies as gravity comp mode
- **Hardware Safety Limits** (enforced in software)
  - Max Cartesian stiffness: **65 N/m**
  - Max Cartesian damping: **3 Ns/m**
- **Launch file**
  - `ws/src/omx_variable_stiffness_controller/launch/dual_hardware_variable_stiffness.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  dynamixel_sdk dynamixel_sdk_custom_interfaces \
  dynamixel_hardware_interface \
  omx_variable_stiffness_controller
source install/setup.bash

# Launch dual arm variable stiffness
ros2 launch omx_variable_stiffness_controller dual_hardware_variable_stiffness.launch.py

# Monitor manipulability metrics
ros2 topic echo /robot1/robot1_variable_stiffness/manipulability
# Output: [λ1, λ2, λ3, condition_number, determinant]

# Load custom stiffness profile at runtime
ros2 run omx_variable_stiffness_controller load_stiffness.py \
  --file config/robot1_stiffness_profile.csv \
  --controller /robot1/robot1_variable_stiffness
```

**Stiffness Profile CSV Format:**
```csv
s,Kx,Ky,Kz,Dx,Dy,Dz
0.0,60.0,60.0,60.0,2.5,2.5,2.5
0.5,35.0,35.0,35.0,1.4,1.4,1.4
1.0,60.0,60.0,60.0,2.5,2.5,2.5
```
Where `s` is trajectory progress (0-1), K is stiffness (N/m), D is damping (Ns/m).

**Runtime Waypoint Commands:**

The controller supports publishing waypoint deviations at runtime via the `~/waypoint_command` topic (geometry_msgs/PoseStamped). The robot will smoothly blend towards the commanded position based on the current impedance settings.

```bash
# Publish an ABSOLUTE target position (world frame)
ros2 topic pub --once /robot1/robot1_variable_stiffness/waypoint_command \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'absolute'}, pose: {position: {x: 0.22, y: 0.0, z: 0.18}}}"

# Publish an OFFSET from current trajectory (relative deviation)
ros2 topic pub --once /robot1/robot1_variable_stiffness/waypoint_command \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'offset'}, pose: {position: {x: 0.02, y: 0.0, z: -0.03}}}"

# Queue multiple waypoints (they'll be traversed in order)
ros2 topic pub --once /robot1/robot1_variable_stiffness/waypoint_command \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'absolute'}, pose: {position: {x: 0.20, y: 0.05, z: 0.15}}}"
ros2 topic pub --once /robot1/robot1_variable_stiffness/waypoint_command \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'offset'}, pose: {position: {x: 0.0, y: -0.10, z: 0.0}}}"

# Check if waypoint tracking is active
ros2 topic echo /robot1/robot1_variable_stiffness/waypoint_active
```

- **frame_id = "offset" or "relative"**: Waypoint is treated as offset from current trajectory target
- **frame_id = "absolute" or empty**: Waypoint is absolute position in world frame
- **Queue behavior**: Multiple waypoints are queued and traversed sequentially
- **Compliance**: Tracking smoothness depends on current stiffness (lower = softer tracking)

### 6) Variable Cartesian Impedance Control (Dual Simulation) 🔧 UNTESTED

> ⚠️ **Status: BUILD ONLY** — This controller compiles but has NOT been tested in Gazebo simulation.

- **Build packages/files**
  - `omx_variable_stiffness_controller` (build files: `ws/src/omx_variable_stiffness_controller/CMakeLists.txt`)
  - Configs: `ws/src/omx_variable_stiffness_controller/config/robot*_variable_stiffness.yaml`
- **Key dependencies**
  - Gazebo Classic + ROS integration: `gazebo_ros`, `gazebo_ros2_control`
  - KDL for kinematics: `liborocos-kdl-dev`, `ros-humble-kdl-parser`
  - Eigen for matrix operations: `libeigen3-dev`
- **Launch file**
  - `ws/src/omx_variable_stiffness_controller/launch/dual_gazebo_variable_stiffness.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Install dependencies
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  liborocos-kdl-dev ros-humble-kdl-parser libeigen3-dev

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  omx_variable_stiffness_controller
source install/setup.bash

# Launch dual arm simulation
ros2 launch omx_variable_stiffness_controller dual_gazebo_variable_stiffness.launch.py

# Sanity checks
ros2 control list_controllers -c /robot1/controller_manager
ros2 topic echo /robot1/robot1_variable_stiffness/manipulability
```

### 7) Variable Cartesian Impedance Control (Single Hardware) 🔧 UNTESTED

> ⚠️ **Status: BUILD ONLY** — This controller compiles but has NOT been tested on actual hardware.

- **Build packages/files**
  - `omx_variable_stiffness_controller` (build files: `ws/src/omx_variable_stiffness_controller/CMakeLists.txt`)
  - Config: `ws/src/omx_variable_stiffness_controller/config/variable_stiffness_controller.yaml`
- **Key dependencies**
  - Dynamixel + hardware IO: `dynamixel_sdk`, `dynamixel_hardware_interface`
  - KDL for kinematics: `liborocos-kdl-dev`, `ros-humble-kdl-parser`
  - Eigen for matrix operations: `libeigen3-dev`
- **Launch file**
  - `ws/src/omx_variable_stiffness_controller/launch/variable_stiffness_control.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Install dependencies
sudo apt update
sudo apt install -y liborocos-kdl-dev ros-humble-kdl-parser libeigen3-dev

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  dynamixel_sdk dynamixel_sdk_custom_interfaces \
  dynamixel_hardware_interface \
  omx_variable_stiffness_controller
source install/setup.bash

# Launch single arm hardware (auto-detects port)
ros2 launch omx_variable_stiffness_controller variable_stiffness_control.launch.py

# Or specify port explicitly
ros2 launch omx_variable_stiffness_controller variable_stiffness_control.launch.py \
  port:=/dev/ttyUSB0

# Sanity checks
ros2 control list_controllers -c /omx/controller_manager
ros2 topic echo /omx/variable_stiffness_controller/manipulability
```

### 8) Variable Cartesian Impedance Control (Single Simulation) 🔧 UNTESTED

> ⚠️ **Status: BUILD ONLY** — This controller compiles but has NOT been tested in Gazebo simulation.

- **Build packages/files**
  - `omx_variable_stiffness_controller` (build files: `ws/src/omx_variable_stiffness_controller/CMakeLists.txt`)
  - Config: `ws/src/omx_variable_stiffness_controller/config/variable_stiffness_controller.yaml`
- **Key dependencies**
  - Gazebo Classic + ROS integration: `gazebo_ros`, `gazebo_ros2_control`
  - KDL for kinematics: `liborocos-kdl-dev`, `ros-humble-kdl-parser`
  - Eigen for matrix operations: `libeigen3-dev`
- **Launch file**
  - `ws/src/omx_variable_stiffness_controller/launch/variable_stiffness_control.launch.py` with `sim:=true`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Install dependencies
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  liborocos-kdl-dev ros-humble-kdl-parser libeigen3-dev

# Build
colcon build --symlink-install --packages-select \
  open_manipulator_x_description \
  omx_variable_stiffness_controller
source install/setup.bash

# Launch single arm simulation
ros2 launch omx_variable_stiffness_controller variable_stiffness_control.launch.py sim:=true

# Sanity checks
ros2 control list_controllers -c /omx/controller_manager
ros2 topic echo /omx/variable_stiffness_controller/manipulability
```

## Quick Start

### 1. Build the Workspace

```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select omx_dual_bringup omx_gravity_comp_controller
source install/setup.bash
```

### 2. Launch with Auto-Detection (Recommended)

The auto-detection launch file will automatically detect if two robots are connected. If not, it will launch Gazebo simulation:

```bash
ros2 launch omx_dual_bringup auto_dual_gravity_comp.launch.py
```

**Launch Arguments:**
- `mode`: `auto` (default), `hw`, `sim`
- `start_rviz`: `true` (default) or `false`
- `robot1_port`: Serial port for robot 1 (optional, auto-detected)
- `robot2_port`: Serial port for robot 2 (optional, auto-detected)
  - `gazebo_mode`: `single` (default) or `dual` (two Gazebo servers)
  - `start_gzclient`: `true` (default) or `false`

**Examples:**
```bash
# Auto-detect (default)
ros2 launch omx_dual_bringup auto_dual_gravity_comp.launch.py

# Force simulation mode
ros2 launch omx_dual_bringup auto_dual_gravity_comp.launch.py mode:=sim

# Force hardware mode with custom ports
ros2 launch omx_dual_bringup auto_dual_gravity_comp.launch.py \
  mode:=hw \
  robot1_port:=/dev/ttyUSB0 \
  robot2_port:=/dev/ttyUSB1

# Without RViz
ros2 launch omx_dual_bringup auto_dual_gravity_comp.launch.py start_rviz:=false
```

### 3. Launch Gazebo Simulation (Manual)

```bash
ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py
```

**Launch Arguments:**
- `gazebo_mode`: `single` (default) or `dual` (two Gazebo servers)
- `start_gzclient`: `true` (default) or `false`

**Launch Arguments:**
- `start_rviz`: `true` (default) or `false`
- `robot1_x`, `robot1_y`: Position of robot 1 (default: 0.0, 0.5)
- `robot2_x`, `robot2_y`: Position of robot 2 (default: 0.0, -0.5)

**Example:**
```bash
# Position robots at custom locations
ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py \
  robot1_x:=0.5 robot1_y:=0.0 \
  robot2_x:=-0.5 robot2_y:=0.0
```

### 4. Launch with Hardware (Manual)

Make sure two Open Manipulator X robots are connected via USB/serial:

```bash
ros2 launch omx_dual_bringup dual_hardware_gravity_comp.launch.py
```

**Launch Arguments:**
- `start_rviz`: `true` (default) or `false`
- `robot1_port`: Serial port for robot 1 (auto-detected)
- `robot2_port`: Serial port for robot 2 (auto-detected)

**Example:**
```bash
# Specify custom serial ports
ros2 launch omx_dual_bringup dual_hardware_gravity_comp.launch.py \
  robot1_port:=/dev/ttyUSB0 \
  robot2_port:=/dev/ttyUSB1
```

## Namespace Isolation

Both robots operate in separate namespaces with proper isolation:

### Robot 1 Topics
- `/robot1/joint_states`
- `/robot1/robot_description`
- `/robot1/controller_manager/*`
- `/robot1/gravity_comp_controller/*`

### Robot 2 Topics
- `/robot2/joint_states`
- `/robot2/robot_description`
- `/robot2/controller_manager/*`
- `/robot2/gravity_comp_controller/*`

## Controller Management

### List Controllers

```bash
# Robot 1
ros2 control list_controllers -c /robot1/controller_manager

# Robot 2
ros2 control list_controllers -c /robot2/controller_manager
```

### Load/Unload Controllers

```bash
# Load gravity compensation controller for robot 1
ros2 control load_controller -c /robot1/controller_manager robot1/gravity_comp_controller

# Start controller
ros2 control set_controller_state -c /robot1/controller_manager robot1/gravity_comp_controller active

# Stop controller
ros2 control set_controller_state -c /robot1/controller_manager robot1/gravity_comp_controller inactive
```

## Viewing Joint States

```bash
# Robot 1 joint states
ros2 topic echo /robot1/joint_states

# Robot 2 joint states
ros2 topic echo /robot2/joint_states
```

## Tests

Run the verification script after launching Gazebo + RViz to confirm controllers and nonzero effort:

```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# In another terminal, with sim already running
./src/omx_dual_bringup/scripts/verify_dual_setup.sh
```

The script checks:
- `gravity_comp_controller` is active for both robots
- Effort command interfaces are available
- `robot_description` is present
- `/robot*/joint_states` has nonzero effort values
- RViz2 and Gazebo nodes are running (best-effort)

## Configuration Details

### Gravity Compensation Parameters

Each robot has independent gravity compensation configuration:

**Gazebo Simulation:**
- Update rate: 500 Hz
- Torque limits: 2.0 - 4.0 Nm (per joint)
- Damping: 0.02 - 0.05 (lower for simulation)

**Hardware:**
- Update rate: 500 Hz
- Current limits: 0.4 - 0.8 A (Dynamixel safety limits)
- Damping: 0.05 - 0.10 (higher for real hardware)

### YAML Configuration Example

```yaml
robot1/controller_manager:
  ros__parameters:
    update_rate: 500
    use_sim_time: true

    robot1/joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    robot1/gravity_comp_controller:
      type: omx_gravity_comp_controller/OmxGravityCompController

robot1/gravity_comp_controller:
  ros__parameters:
    joints: [robot1_joint1, robot1_joint2, robot1_joint3, robot1_joint4]
    root_link: robot1_world
    tip_link: robot1_link5
    torque_scale: 1.0
    robot_description_node: /robot1/controller_manager
```

## Troubleshooting

### Serial Port Detection Issues

If hardware is not detected:

```bash
# List available serial ports
ls -la /dev/ttyUSB* /dev/ttyACM*
ls -la /dev/serial/by-id/

# Check port permissions
sudo usermod -a -G dialout $USER
# May need to log out and back in

# Manually specify ports
ros2 launch omx_dual_bringup dual_hardware_gravity_comp.launch.py \
  robot1_port:=/dev/ttyUSB0 \
  robot2_port:=/dev/ttyUSB1
```

### Controllers Not Loading

```bash
# Check controller manager status
ros2 control list_controllers -c /robot1/controller_manager

# Manually load controllers
ros2 control load_controller -c /robot1/controller_manager robot1/joint_state_broadcaster
ros2 control load_controller -c /robot1/controller_manager robot1/gravity_comp_controller
ros2 control set_controller_state -c /robot1/controller_manager robot1/joint_state_broadcaster active
ros2 control set_controller_state -c /robot1/controller_manager robot1/gravity_comp_controller active
```

### Gazebo Not Starting

Make sure Gazebo is installed:

```bash
sudo apt-get update
sudo apt-get install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

### Gazebo "Address already in use"

If you see `EXCEPTION: Unable to start server[bind: Address already in use]`, a previous `gzserver` is still running:

```bash
pkill -f gzserver || true
pkill -f gzclient || true
```

### Relaunching After Previous Session

If you see errors like `Controller already loaded` or `Entity already exists`, clean up first:

```bash
./src/omx_dual_bringup/scripts/cleanup.sh
```

Then relaunch:
```bash
ros2 launch omx_dual_bringup auto_dual_gravity_comp.launch.py
```

### "Stereo is NOT SUPPORTED" Warning

The RViz2 message `Stereo is NOT SUPPORTED` is **informational only** - RViz2 works fine without stereo display support. This is normal in containers or VMs.

### Multiple Gazebo Windows or Black Screen

If you see multiple Gazebo windows or a blank client:

```bash
# Use a single Gazebo server for both robots (recommended for visualization)
ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py gazebo_mode:=single

# Or disable auto-starting the client and launch it manually
ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py start_gzclient:=false
gzclient
```

### TF Frame Issues

If you see TF warnings about missing frames:

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Visualize TF in RViz
ros2 run rqt_tf_tree rqt_tf_tree
```

## Hardware Setup

### Physical Robot Connection

1. Connect both Open Manipulator X robots via USB to separate ports
2. Power on both robots
3. Verify serial connections: `ls -la /dev/ttyUSB*`
4. Launch with auto-detection or specify ports explicitly

### Port Identification

```bash
# List ports by ID (recommended)
ls -la /dev/serial/by-id/

# Create udev rules for persistent naming (optional)
# Example: /etc/udev/rules.d/99-omx.rules
# SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="XXXXX", SYMLINK+="omx_robot1"
```

## Advanced Usage

### Running with MoveIt2 (Future)

The namespace isolation allows integration with MoveIt2 for motion planning:

```python
# Example: Planning for robot1
from moveit_py import MoveGroupInterface

robot1_group = MoveGroupInterface(node, "robot1/arm")
robot1_group.set_joint_value_target([0.0, 0.5, 0.3, 0.0])
robot1_group.move()
```

### Custom Controller Parameters

Edit the YAML files in `config/` to adjust:
- Damping coefficients
- Torque/current limits
- Update rates
- Safety parameters

## Dependencies

Required ROS2 packages:
- `controller_manager`
- `ros2_control`
- `ros2controlcli`
- `gazebo_ros`
- `gazebo_ros2_control`
- `robot_state_publisher`
- `rviz2`
- `omx_gravity_comp_controller`

## License

Apache-2.0

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review ROS2 control documentation
3. Check controller manager logs: `ros2 control list_controllers -c /robot1/controller_manager`


### TL;DR:

### Build:
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select omx_dual_bringup omx_gravity_comp_controller
source install/setup.bash



source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch omx_dual_bringup single_robot_hardware.launch.py port:=/dev/ttyUSB0

# 1. Set latency timer (do this every time after USB reconnect)
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

# 2. Disable torque on all servos
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/omx_dual_bringup/scripts/disable_torque.py --port /dev/ttyUSB0

# 3. Launch
ros2 launch omx_dual_bringup single_robot_hardware.launch.py port:=/dev/ttyUSB0


source install/setup.bash
ros2 launch omx_dual_bringup single_robot_hardware.launch.py port:=/dev/ttyUSB0







source /opt/ros/humble/setup.bash && source /workspaces/omx_ros2/ws/install/setup.bash && ros2 launch omx_dual_bringup dual_hardware_gravity_comp.launch.py start_rviz:=false 2>&1