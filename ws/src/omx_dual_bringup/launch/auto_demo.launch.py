import os
import glob

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _detect_hw_port(preferred: str) -> str:
    if preferred:
        return preferred

    byid = sorted(glob.glob("/dev/serial/by-id/*"))
    if byid:
        return byid[0]

    for pat in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]

    return ""


def _choose(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    port_arg = LaunchConfiguration("port").perform(context).strip()
    hw_port = _detect_hw_port(port_arg)
    hw_present = bool(hw_port) and os.path.exists(hw_port)

    if mode == "hw":
        use_hw = True
    elif mode == "sim":
        use_hw = False
    else:
        use_hw = hw_present

    bringup_share = FindPackageShare("open_manipulator_x_bringup").find("open_manipulator_x_bringup")
    hw_launch_path = os.path.join(bringup_share, "launch", "hardware.launch.py")
    sim_launch_path = os.path.join(bringup_share, "launch", "gazebo_rviz.launch.py")

    if use_hw:
        chosen = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hw_launch_path),
            launch_arguments={"port": hw_port}.items() if hw_port else {}
        )
    else:
        chosen = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_path)
        )

    msg = (
        f"[auto_demo] mode='{mode}' | detected_port='{hw_port}' | "
        f"hw_present={hw_present} | chosen={'HARDWARE' if use_hw else 'SIM'}"
    )
    return [LogInfo(msg=msg), chosen]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="auto", description="auto|sim|hw"),
        DeclareLaunchArgument("port", default_value="", description="Optional serial port override"),
        OpaqueFunction(function=_choose),
    ])
