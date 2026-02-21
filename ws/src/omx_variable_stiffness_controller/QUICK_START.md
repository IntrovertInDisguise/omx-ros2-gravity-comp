# Quick Start: Testing Variable Stiffness Controller on Single Robot Hardware

## 30-Second Setup

```bash
# Terminal 1: Launch controller (hardware auto-detected)
source /opt/ros/humble/setup.bash
source /workspaces/omx_ros2/ws/install/setup.bash
ros2 launch omx_variable_stiffness_controller variable_stiffness_control.launch.py sim:=false

# Terminal 2: Monitor in real-time
source /opt/ros/humble/setup.bash
source /workspaces/omx_ros2/ws/install/setup.bash

# Watch actual vs desired EE positions
ros2 topic echo /omx/variable_stiffness_controller/cartesian_pose_actual

# In another tab: Watch torque commands
ros2 topic echo /omx/variable_stiffness_controller/torque_values

# In another tab: Test dynamic waypoint  
ros2 topic pub --once /omx/variable_stiffness_controller/waypoint_command \
  geometry_msgs/msg/PoseStamped "{header: {frame_id: 'absolute'}, pose: {position: {x: 0.20, y: 0.05, z: 0.18}}}"
```

## What You'll See

**Phase 1: Homing (0-5s)**
```
[INFO] [STARTING] Homing from current EE pose...
[INFO] [STATE] Homing complete, starting MOVE_FORWARD
```
- Robot smoothly moves to start position
- Gravity compensation active (joints hold against gravity)

**Phase 2: Movement (5-15s)**
```
[INFO] [STATE] Reached end, waiting
```
- Forward trajectory with variable stiffness
- Gravity compensation maintains position
- Topics publish at 500 Hz

**Phase 3: Return (15+s)**
```
[INFO] [STATE] Starting MOVE_RETURN
```
- Reverse trajectory back to start
- Cycle repeats automatically

## Dynamic Waypoint Test

Publish offset waypoint while moving forward:
```bash
ros2 topic pub --once /omx/variable_stiffness_controller/waypoint_command \
  geometry_msgs/msg/PoseStamped "{header: {frame_id: 'offset'}, pose: {position: {x: 0.05, y: -0.02, z: 0.0}}}"
```

Expected: Smooth blend toward new position (2 second interpolation)

## Safety Checks

| Check | Expected | Command |
|-------|----------|---------|
| Gravity working | Robot doesn't fall when stopped | `rostopic echo /omx/.../torque_values` |
| Waypoints queue | Multiple waypoints process in order | Publish 5 waypoints in 0.5s |
| Stiffness profile | Varies with trajectory progress | `rostopic echo /omx/.../stiffness_state` |
| Manipulability | No singularity warnings in large window | `ros2 topic echo /omx/.../manipulability_metrics` |

## Troubleshooting

**Motor doesn't move:**
- Check port: `ls /dev/ttyUSB*`
- Verify torque_scale in config (try 100-300)

**Jerky motion:**
- Increase `waypoint_blend_duration_` to 3.0 seconds
- Reduce `max_rise_rate_` to 30-50 N/m/s

**IK validation errors:**
- Disable with `use_joint_space_trajectory: false` in config
- Or adjust waypoints further from singularities

**No gravity compensation:**
- Check `robot_state_publisher` is running: `ros2 node list`
- Verify URDF loads correctly: `ros2 param get /omx/robot_state_publisher robot_description | head -20`

## Performance Metrics

- **Control frequency**: 500 Hz (2 ms cycle)
- **IK solver time**: ~5-10 ms (only at startup for validation)
- **Waypoint callback latency**: <100 µs
- **Actual cycle overhead**: ~1.3 ms (well within budget)

See `IMPLEMENTATION_GUIDE.md` for complete documentation.
