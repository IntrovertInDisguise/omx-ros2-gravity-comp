# Dual Open Manipulator X with Gravity Compensation

This package provides launch files and configurations for running **two independent Open Manipulator X robots** with gravity compensation in ROS2. The robots can be controlled via:
- **Hardware**: Two robots connected via separate serial ports
- **Simulation**: Gazebo simulation with RViz2 visualization
- **Auto-detection**: Automatically detect hardware or fallback to simulation

## Features

✅ Proper namespace isolation (`robot1` and `robot2`)
✅ Independent YAML configurations for each robot
✅ Gravity compensation controller for each robot
✅ Support for both hardware and Gazebo simulation
✅ Automatic hardware detection
✅ RViz2 visualization for both robots

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
```

## Setup Recipes (4 Modes)

Each mode below lists:
- **Build packages/files**: which packages you need to build (and where their build files live)
- **Key dependencies**: what needs to be installed for that mode
- **Launch file**: what to run
- **Commands**: a minimal command list to bring it up

> Note: All commands assume you are in this repo and using ROS 2 Humble.

### 1) Dual Hardware Setup (2 real robots)

- **Build packages/files**
  - `omx_dual_bringup` (build files: `ws/src/omx_dual_bringup/CMakeLists.txt`, `ws/src/omx_dual_bringup/package.xml`)
  - `omx_gravity_comp_controller` (build files: `ws/src/omx_gravity_comp_controller/CMakeLists.txt`, `ws/src/omx_gravity_comp_controller/package.xml`)
  - Configs used: `ws/src/omx_dual_bringup/config/robot1_gravity_comp.yaml`, `ws/src/omx_dual_bringup/config/robot2_gravity_comp.yaml`
- **Key dependencies**
  - Dynamixel + hardware IO: `dynamixel_sdk`, `dynamixel_hardware_interface`
  - ROS 2 control stack: `ros2_control`, `ros2_controllers`, `controller_manager`
  - Description/xacro: `open_manipulator_x_description`, `xacro`, `robot_state_publisher`
  - Serial access from Python tools/scripts: `python3-serial`
- **Launch file**
  - `ws/src/omx_dual_bringup/launch/dual_hardware_gravity_comp.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Build
colcon build --symlink-install --packages-select omx_gravity_comp_controller omx_dual_bringup
source install/setup.bash

# Launch (auto-detects ports, or override)
ros2 launch omx_dual_bringup dual_hardware_gravity_comp.launch.py \
  robot1_port:=/dev/ttyUSB0 robot2_port:=/dev/ttyUSB1 start_rviz:=true

# Sanity checks
ros2 control list_controllers -c /robot1/controller_manager
ros2 control list_controllers -c /robot2/controller_manager
ros2 topic echo /robot1/joint_states
```

### 2) Dual Simulation Setup (2 robots in Gazebo)

- **Build packages/files**
  - `omx_dual_bringup` (build files: `ws/src/omx_dual_bringup/CMakeLists.txt`, `ws/src/omx_dual_bringup/package.xml`)
  - `omx_gravity_comp_controller` (build files: `ws/src/omx_gravity_comp_controller/CMakeLists.txt`, `ws/src/omx_gravity_comp_controller/package.xml`)
  - Configs used: `ws/src/omx_dual_bringup/config/robot1_gravity_comp.yaml`, `ws/src/omx_dual_bringup/config/robot2_gravity_comp.yaml`
- **Key dependencies**
  - Gazebo Classic + ROS integration: `gazebo_ros`, `gazebo_ros2_control`, `gazebo_ros_pkgs`
  - ROS 2 control stack: `controller_manager`, `ros2controlcli`
  - Description/xacro: `open_manipulator_x_description`, `xacro`, `robot_state_publisher`
- **Launch file**
  - `ws/src/omx_dual_bringup/launch/dual_gazebo_gravity_comp.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Build
colcon build --symlink-install --packages-select omx_gravity_comp_controller omx_dual_bringup
source install/setup.bash

# Launch (single Gazebo server by default)
ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py gazebo_mode:=single start_rviz:=true

# Optional: full isolation using two Gazebo servers
# ros2 launch omx_dual_bringup dual_gazebo_gravity_comp.launch.py gazebo_mode:=dual

# Sanity checks
ros2 control list_controllers -c /robot1/controller_manager
ros2 control list_controllers -c /robot2/controller_manager
```

### 3) Single Simulation Setup (1 robot in Gazebo)

- **Build packages/files**
  - `omx_dual_bringup` (build files: `ws/src/omx_dual_bringup/CMakeLists.txt`, `ws/src/omx_dual_bringup/package.xml`)
  - `omx_gravity_comp_controller` (build files: `ws/src/omx_gravity_comp_controller/CMakeLists.txt`, `ws/src/omx_gravity_comp_controller/package.xml`)
  - Config used: `ws/src/omx_dual_bringup/config/single_robot_gravity_comp.yaml`
- **Key dependencies**
  - Gazebo Classic + ROS integration: `gazebo_ros`, `gazebo_ros2_control`, `gazebo_ros_pkgs`
  - ROS 2 control stack: `controller_manager`, `ros2controlcli`
  - Description/xacro: `open_manipulator_x_description`, `xacro`, `robot_state_publisher`
- **Launch file**
  - `ws/src/omx_dual_bringup/launch/single_robot_test.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Build
colcon build --symlink-install --packages-select omx_gravity_comp_controller omx_dual_bringup
source install/setup.bash

# Launch
ros2 launch omx_dual_bringup single_robot_test.launch.py

# Sanity checks
ros2 control list_controllers -c /omx/controller_manager
ros2 topic echo /omx/joint_states
```

### 4) Single Hardware Setup (1 real robot)

- **Build packages/files**
  - `omx_dual_bringup` (build files: `ws/src/omx_dual_bringup/CMakeLists.txt`, `ws/src/omx_dual_bringup/package.xml`)
  - `omx_gravity_comp_controller` (build files: `ws/src/omx_gravity_comp_controller/CMakeLists.txt`, `ws/src/omx_gravity_comp_controller/package.xml`)
  - Config used: `ws/src/omx_dual_bringup/config/single_robot_hardware_gravity_comp.yaml`
- **Key dependencies**
  - Dynamixel + hardware IO: `dynamixel_sdk`, `dynamixel_hardware_interface`
  - ROS 2 control stack: `ros2_control`, `ros2_controllers`, `controller_manager`
  - Description/xacro: `open_manipulator_x_description`, `xacro`, `robot_state_publisher`
- **Launch file**
  - `ws/src/omx_dual_bringup/launch/single_robot_hardware.launch.py`
- **Commands**
```bash
cd /workspaces/omx_ros2/ws
source /opt/ros/humble/setup.bash

# Build
colcon build --symlink-install --packages-select omx_gravity_comp_controller omx_dual_bringup
source install/setup.bash

# Launch (auto-detects port, or override)
ros2 launch omx_dual_bringup single_robot_hardware.launch.py port:=/dev/ttyUSB0 start_rviz:=false

# Sanity checks
ros2 control list_controllers -c /omx/controller_manager
ros2 topic echo /omx/joint_states
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