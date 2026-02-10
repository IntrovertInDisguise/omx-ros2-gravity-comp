#!/usr/bin/env python3
#
# Auto-detecting Dual Open Manipulator X Launch with Gravity Compensation
# Automatically detects if hardware is connected or uses Gazebo simulation
#

import os
import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _detect_dual_ports(port1_arg: str, port2_arg: str):
    """
    Detect two serial ports for dual robot setup.
    Returns tuple of (port1, port2, both_present)
    """
    ports = []
    
    # If ports are explicitly provided, use them
    if port1_arg and port2_arg:
        return port1_arg, port2_arg, (os.path.exists(port1_arg) and os.path.exists(port2_arg))
    
    # Check /dev/serial/by-id/ first (most reliable)
    byid = sorted(glob.glob("/dev/serial/by-id/*"))
    ports.extend(byid)
    
    # Add USB/ACM devices
    for pattern in ["/dev/ttyUSB*", "/dev/ttyACM*"]:
        hits = sorted(glob.glob(pattern))
        for hit in hits:
            if hit not in ports:
                ports.append(hit)
    
    # Filter to only existing ports
    ports = [p for p in ports if os.path.exists(p)]
    
    port1 = ports[0] if len(ports) > 0 else ""
    port2 = ports[1] if len(ports) > 1 else ""
    both_present = len(ports) >= 2
    
    return port1, port2, both_present


def _choose_launch_mode(context, *args, **kwargs):
    """
    Decide whether to launch hardware or simulation based on mode and port detection.
    """
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    port1_arg = LaunchConfiguration("robot1_port").perform(context).strip()
    port2_arg = LaunchConfiguration("robot2_port").perform(context).strip()
    start_rviz = LaunchConfiguration("start_rviz").perform(context).strip()
    
    port1, port2, hw_present = _detect_dual_ports(port1_arg, port2_arg)
    
    # Determine launch mode
    if mode == "hw" or mode == "hardware":
        use_hw = True
    elif mode == "sim" or mode == "simulation":
        use_hw = False
    else:  # auto mode
        use_hw = hw_present
    
    # Get package share directory
    pkg_share = FindPackageShare("omx_dual_bringup").find("omx_dual_bringup")
    
    # Choose appropriate launch file
    if use_hw:
        hw_launch_path = os.path.join(pkg_share, "launch", "dual_hardware_gravity_comp.launch.py")
        chosen = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hw_launch_path),
            launch_arguments={
                'robot1_port': port1,
                'robot2_port': port2,
                'start_rviz': start_rviz,
            }.items()
        )
        mode_str = "HARDWARE"
    else:
        sim_launch_path = os.path.join(pkg_share, "launch", "dual_gazebo_gravity_comp.launch.py")
        chosen = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_path),
            launch_arguments={
                'start_rviz': start_rviz,
            }.items()
        )
        mode_str = "SIMULATION (Gazebo + RViz2)"
    
    # Create informative log message
    msg = (
        f"[auto_dual_launch] mode='{mode}' | "
        f"detected_ports=['{port1}', '{port2}'] | "
        f"hw_present={hw_present} | "
        f"chosen={mode_str}"
    )
    
    return [LogInfo(msg=msg), chosen]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="auto",
            description="Launch mode: 'auto' (detect hardware), 'hw'/'hardware' (force hardware), 'sim' (force simulation with Gazebo)"
        ),
        DeclareLaunchArgument(
            "robot1_port",
            default_value="",
            description="Optional serial port override for robot1"
        ),
        DeclareLaunchArgument(
            "robot2_port",
            default_value="",
            description="Optional serial port override for robot2"
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Whether to start RViz2"
        ),
        OpaqueFunction(function=_choose_launch_mode),
    ])
