#!/usr/bin/env python3
"""
Live timeseries plotter — works for both controller variants.

Subscribes directly to ROS2 topics — no log files are ever read or written.
Uses the same publication-quality matplotlib style as plot_logs.py.

Two controller modes:

  variable_stiffness  Subscribe to all detailed controller state topics
                      (cartesian pose, EE velocity, stiffness/damping, etc.)
                      Published by OmxVariableStiffnessController.

  gravity_comp        Subscribe to /joint_states (from joint_state_broadcaster)
                      and plot joint positions, velocities and compensation
                      efforts.  Published by joint_state_broadcaster alongside
                      OmxGravityCompController.

Usage:
    # Variable-stiffness single robot
    python3 live_plot_logs.py --controller variable_stiffness \\
        --namespace /omx/variable_stiffness_controller

    # Variable-stiffness dual robot
    python3 live_plot_logs.py --controller variable_stiffness \\
        --namespace /robot1/robot1_variable_stiffness \\
        --namespace2 /robot2/robot2_variable_stiffness

    # Gravity-comp single robot
    python3 live_plot_logs.py --controller gravity_comp --namespace /omx

    # Gravity-comp dual robot
    python3 live_plot_logs.py --controller gravity_comp \\
        --namespace /robot1 --namespace2 /robot2

    # Tune rolling window and refresh rate
    python3 live_plot_logs.py --window 60 --interval 0.5

    # Via ROS2 launch file (args forwarded automatically)
    ros2 launch tools/launch/live_plot.launch.py
"""

from __future__ import annotations

import argparse
import collections
import os
import sys
import threading
import time
from typing import Deque, Dict, List, Optional

# ---------------------------------------------------------------------------
# matplotlib — must be configured before any other import uses it.
# ---------------------------------------------------------------------------
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# ROS2 — required.
# ---------------------------------------------------------------------------
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Pose, Point, Vector3, WrenchStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, Bool
except ImportError as exc:
    print(f"Error: rclpy not available — is the ROS2 workspace sourced?\n  {exc}")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Import publication-quality helpers directly from plot_logs.py so that the
# style (rcParams, colours, grids, legends) is 100% identical.
# ---------------------------------------------------------------------------
_tools_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _tools_dir)
try:
    import plot_logs as pl
    from plot_logs import (
        apply_pub_style,
        _enable_minor_grid,
        _style_legend,
        _add_time_xlabel,
        _HC_PALETTE,
        ROBOT_COLORS,
        TIMESERIES_GROUPS,
        ALL_LABELS,
    )
except ImportError as exc:
    print(f"Error: could not import plot_logs.py: {exc}")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Controller mode constants
# ---------------------------------------------------------------------------
MODE_VS = "variable_stiffness"
MODE_GC = "gravity_comp"

# ---------------------------------------------------------------------------
# Gravity-comp column groups (joint_state_broadcaster topics)
# ---------------------------------------------------------------------------
GC_POS_COLS = {
    "jpos1": "Joint 1 Position (rad)",
    "jpos2": "Joint 2 Position (rad)",
    "jpos3": "Joint 3 Position (rad)",
    "jpos4": "Joint 4 Position (rad)",
}
GC_VEL_COLS = {
    "jvel1": "Joint 1 Velocity (rad/s)",
    "jvel2": "Joint 2 Velocity (rad/s)",
    "jvel3": "Joint 3 Velocity (rad/s)",
    "jvel4": "Joint 4 Velocity (rad/s)",
}
GC_EFF_COLS = {
    "jeff1": "Joint 1 Grav-Comp Effort (Nm)",
    "jeff2": "Joint 2 Grav-Comp Effort (Nm)",
    "jeff3": "Joint 3 Grav-Comp Effort (Nm)",
    "jeff4": "Joint 4 Grav-Comp Effort (Nm)",
}
GC_TIMESERIES_GROUPS = [
    ("Joint Positions", GC_POS_COLS),
    ("Joint Velocities", GC_VEL_COLS),
    ("Gravity Compensation Efforts", GC_EFF_COLS),
]
GC_ALL_LABELS: Dict[str, str] = {}
for _d in [GC_POS_COLS, GC_VEL_COLS, GC_EFF_COLS]:
    GC_ALL_LABELS.update(_d)

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
DEFAULT_NS_VS  = "variable_stiffness_controller"
DEFAULT_NS_GC  = "omx"
DEFAULT_WINDOW_S   = 30.0
DEFAULT_INTERVAL_S = 0.5
MAX_SAMPLES        = 20_000


# ---------------------------------------------------------------------------
# Thread-safe rolling buffer
# ---------------------------------------------------------------------------
class LiveBuffer:
    """Accumulates ROS2 callbacks into per-column deques (thread-safe)."""

    VS_COLUMNS: List[str] = [
        "time_s",
        "actual_x", "actual_y", "actual_z",
        "desired_x", "desired_y", "desired_z",
        "ee_x", "ee_y", "ee_z",
        "ee_roll", "ee_pitch", "ee_yaw",
        "ee_vx", "ee_vy", "ee_vz", "ee_wx", "ee_wy", "ee_wz",
        "jv1", "jv2", "jv3", "jv4",
        "tau1", "tau2", "tau3", "tau4",
        "Ktx", "Kty", "Ktz", "Krx", "Kry", "Krz",
        "Dtx", "Dty", "Dtz", "Drx", "Dry", "Drz",
        "manip_yoshikawa", "manip_sigma_min", "manip_sigma_max",
        "manip_condition_number",
        "contact_fx", "contact_fy", "contact_fz",
        "contact_tx", "contact_ty", "contact_tz",
        "contact_valid", "waypoint_active",
    ]

    GC_COLUMNS: List[str] = [
        "time_s",
        "jpos1", "jpos2", "jpos3", "jpos4",
        "jvel1", "jvel2", "jvel3", "jvel4",
        "jeff1", "jeff2", "jeff3", "jeff4",
    ]

    def __init__(self, controller_mode: str, maxlen: int = MAX_SAMPLES) -> None:
        self._columns = (
            self.VS_COLUMNS if controller_mode == MODE_VS else self.GC_COLUMNS
        )
        self._lock: threading.Lock = threading.Lock()
        self._deques: Dict[str, Deque[float]] = {
            c: collections.deque(maxlen=maxlen) for c in self._columns
        }
        self._t0: Optional[float] = None

    def push(self, row: dict) -> None:
        t_now = time.monotonic()
        with self._lock:
            if self._t0 is None:
                self._t0 = t_now
            self._deques["time_s"].append(t_now - self._t0)
            for col in self._columns:
                if col == "time_s":
                    continue
                self._deques[col].append(row.get(col, float("nan")))

    def snapshot(self) -> Dict[str, list]:
        with self._lock:
            return {c: list(q) for c, q in self._deques.items()}

    def __len__(self) -> int:
        with self._lock:
            return len(self._deques["time_s"])


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------
class LivePlotNode(Node):
    """
    Subscribes to the appropriate topics for the selected controller mode.
    Snapshots at 50 Hz via timer into LiveBuffers.
    No files are read or written.
    """

    def __init__(
        self,
        buffers: Dict[int, LiveBuffer],
        namespaces: Dict[int, str],
        controller_mode: str,
    ) -> None:
        super().__init__("live_plot_logs")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._buffers = buffers
        self._state: Dict[int, dict] = {rn: {} for rn in namespaces}
        self._controller_mode = controller_mode

        for rn, ns in namespaces.items():
            base = ns.rstrip("/")
            r = rn  # capture loop variable

            if controller_mode == MODE_GC:
                # gravity_comp: subscribe to joint_states from joint_state_broadcaster
                self.create_subscription(
                    JointState, f"{base}/joint_states",
                    lambda m, _r=r: self._joint_states_cb(m, _r), qos)
                self.get_logger().info(
                    f"[Robot {rn}] subscribed to {base}/joint_states (gravity_comp)")
            else:
                # variable_stiffness: subscribe to all detailed controller topics
                self.create_subscription(
                    Pose, f"{base}/cartesian_pose_actual",
                    lambda m, _r=r: self._actual_pose_cb(m, _r), qos)
                self.create_subscription(
                    Pose, f"{base}/cartesian_pose_desired",
                    lambda m, _r=r: self._desired_pose_cb(m, _r), qos)
                self.create_subscription(
                    Point, f"{base}/end_effector_position",
                    lambda m, _r=r: self._ee_pos_cb(m, _r), qos)
                self.create_subscription(
                    Vector3, f"{base}/end_effector_orientation",
                    lambda m, _r=r: self._ee_orient_cb(m, _r), qos)
                self.create_subscription(
                    Float64MultiArray, f"{base}/end_effector_velocities",
                    lambda m, _r=r: self._ee_vel_cb(m, _r), qos)
                self.create_subscription(
                    Float64MultiArray, f"{base}/joint_velocities",
                    lambda m, _r=r: self._joint_vel_cb(m, _r), qos)
                self.create_subscription(
                    Float64MultiArray, f"{base}/torque_values",
                    lambda m, _r=r: self._torque_cb(m, _r), qos)
                self.create_subscription(
                    Float64MultiArray, f"{base}/stiffness_state",
                    lambda m, _r=r: self._stiffness_cb(m, _r), qos)
                self.create_subscription(
                    Float64MultiArray, f"{base}/manipulability_metrics",
                    lambda m, _r=r: self._manip_cb(m, _r), qos)
                self.create_subscription(
                    WrenchStamped, f"{base}/contact_wrench",
                    lambda m, _r=r: self._contact_wrench_cb(m, _r), qos)
                self.create_subscription(
                    Bool, f"{base}/contact_valid",
                    lambda m, _r=r: self._contact_valid_cb(m, _r), qos)
                self.create_subscription(
                    Bool, f"{base}/waypoint_active",
                    lambda m, _r=r: self._waypoint_active_cb(m, _r), qos)
                self.get_logger().info(
                    f"[Robot {rn}] subscribed to {base}/* (variable_stiffness)")

        self.create_timer(0.02, self._snapshot_timer)

    # --- gravity comp callback ---

    def _joint_states_cb(self, msg: JointState, r: int) -> None:
        n = min(4, len(msg.position))
        for i in range(n):
            self._state[r][f"jpos{i+1}"] = msg.position[i] if i < len(msg.position) else float("nan")
            self._state[r][f"jvel{i+1}"] = msg.velocity[i] if i < len(msg.velocity) else float("nan")
            self._state[r][f"jeff{i+1}"] = msg.effort[i]   if i < len(msg.effort)   else float("nan")

    # --- variable stiffness callbacks ---

    def _actual_pose_cb(self, msg: Pose, r: int) -> None:
        p = msg.position
        self._state[r].update(actual_x=p.x, actual_y=p.y, actual_z=p.z)

    def _desired_pose_cb(self, msg: Pose, r: int) -> None:
        p = msg.position
        self._state[r].update(desired_x=p.x, desired_y=p.y, desired_z=p.z)

    def _ee_pos_cb(self, msg: Point, r: int) -> None:
        self._state[r].update(ee_x=msg.x, ee_y=msg.y, ee_z=msg.z)

    def _ee_orient_cb(self, msg: Vector3, r: int) -> None:
        self._state[r].update(ee_roll=msg.x, ee_pitch=msg.y, ee_yaw=msg.z)

    def _ee_vel_cb(self, msg: Float64MultiArray, r: int) -> None:
        d = msg.data
        keys = ["ee_vx", "ee_vy", "ee_vz", "ee_wx", "ee_wy", "ee_wz"]
        self._state[r].update({k: d[i] for i, k in enumerate(keys) if i < len(d)})

    def _joint_vel_cb(self, msg: Float64MultiArray, r: int) -> None:
        d = msg.data
        self._state[r].update({f"jv{i+1}": d[i] for i in range(min(4, len(d)))})

    def _torque_cb(self, msg: Float64MultiArray, r: int) -> None:
        d = msg.data
        self._state[r].update({f"tau{i+1}": d[i] for i in range(min(4, len(d)))})

    def _stiffness_cb(self, msg: Float64MultiArray, r: int) -> None:
        d = msg.data
        keys = ["Ktx", "Kty", "Ktz", "Krx", "Kry", "Krz",
                "Dtx", "Dty", "Dtz", "Drx", "Dry", "Drz"]
        self._state[r].update({k: d[i] for i, k in enumerate(keys) if i < len(d)})

    def _manip_cb(self, msg: Float64MultiArray, r: int) -> None:
        d = msg.data
        keys = ["manip_condition_number", "manip_det", "manip_yoshikawa",
                "manip_sigma_min", "manip_sigma_max"]
        self._state[r].update({k: d[i] for i, k in enumerate(keys) if i < len(d)})

    def _contact_wrench_cb(self, msg: WrenchStamped, r: int) -> None:
        f, t = msg.wrench.force, msg.wrench.torque
        self._state[r].update(
            contact_fx=f.x, contact_fy=f.y, contact_fz=f.z,
            contact_tx=t.x, contact_ty=t.y, contact_tz=t.z,
        )

    def _contact_valid_cb(self, msg: Bool, r: int) -> None:
        self._state[r]["contact_valid"] = float(msg.data)

    def _waypoint_active_cb(self, msg: Bool, r: int) -> None:
        self._state[r]["waypoint_active"] = float(msg.data)

    def _snapshot_timer(self) -> None:
        for rn, state in self._state.items():
            if state:
                self._buffers[rn].push(dict(state))


# ---------------------------------------------------------------------------
# Live figure manager
# ---------------------------------------------------------------------------
class LiveFigureManager:
    """
    Single figure with one subplot per timeseries group.  All columns in a
    group share one axes (overlaid lines, coloured per-robot × per-column).
    Fits on screen regardless of mode.
    """

    def __init__(
        self,
        robot_nums: List[int],
        window_s: float,
        robot_count: int,
        controller_mode: str,
    ) -> None:
        self._robot_nums    = robot_nums
        self._window_s      = window_s
        self._robot_count   = robot_count
        self._controller_mode = controller_mode

        # Select the right group/label dictionaries
        if controller_mode == MODE_GC:
            self._timeseries_groups = GC_TIMESERIES_GROUPS
            self._all_labels        = GC_ALL_LABELS
        else:
            self._timeseries_groups = TIMESERIES_GROUPS
            self._all_labels        = ALL_LABELS

        apply_pub_style()  # identical rcParams to plot_logs.py

        self._fig = None
        self._axes: List = []
        # lines[group_idx][(rn, col)] = Line2D
        self._lines: Dict[int, Dict[tuple, object]] = {}
        self._built = False

    # ---- one-time construction ------------------------------------------
    def _build(self) -> None:
        n_groups = len(self._timeseries_groups)
        mode_label = (
            "Gravity Compensation" if self._controller_mode == MODE_GC
            else "Variable Stiffness"
        )
        robot_label = "Dual-Robot" if self._robot_count > 1 else "Single-Robot"

        fig, axes = plt.subplots(
            n_groups, 1,
            figsize=(18, max(2.4 * n_groups, 6)),
            sharex=True, squeeze=False,
        )
        self._fig = fig
        self._axes = axes.flatten()

        fig.suptitle(
            f"{robot_label} {mode_label} — Live",
            fontsize=18, fontweight="bold", y=0.99,
        )

        # Compact font sizes for the combined view
        tick_size = max(8, 14 - n_groups // 4)

        for g_idx, (group_title, col_map) in enumerate(self._timeseries_groups):
            ax = self._axes[g_idx]
            ax.set_ylabel(group_title, fontsize=max(8, 13 - n_groups // 5),
                          fontweight="bold")
            ax.tick_params(labelsize=tick_size)
            _enable_minor_grid(ax)
            self._lines[g_idx] = {}

            col_names = list(col_map.keys())
            col_labels = list(col_map.values())

            for rn in self._robot_nums:
                robot_prefix = "" if self._robot_count == 1 else f"R{rn} "
                for c_idx, col in enumerate(col_names):
                    color = _HC_PALETTE[c_idx % len(_HC_PALETTE)]
                    # For dual-robot: use solid for R1, dashed for R2
                    ls = "-" if rn == 1 else "--"
                    lw = 1.8 if self._robot_count == 1 else 1.4
                    short_label = col_labels[c_idx].split("(")[0].strip()
                    lbl = f"{robot_prefix}{short_label}"
                    (line,) = ax.plot([], [], color=color, linewidth=lw,
                                     linestyle=ls, label=lbl)
                    self._lines[g_idx][(rn, col)] = line

            ax.legend(fontsize=max(6, 9 - n_groups // 5), loc="upper left",
                      ncol=max(1, len(col_map)),
                      framealpha=0.7, borderpad=0.3, handlelength=1.5)

        _add_time_xlabel(self._axes[-1])
        self._axes[-1].tick_params(labelsize=tick_size)
        fig.tight_layout(rect=[0, 0.01, 1.0, 0.96])
        fig.subplots_adjust(hspace=0.35)
        self._built = True

    # ---- per-cycle refresh ----------------------------------------------
    def update(self, snapshots: Dict[int, dict]) -> None:
        if not self._built:
            if all(len(s.get("time_s", [])) > 1 for s in snapshots.values()):
                self._build()
            else:
                return

        for g_idx, (_group_title, col_map) in enumerate(self._timeseries_groups):
            ax = self._axes[g_idx]
            for col in col_map:
                for rn in self._robot_nums:
                    snap   = snapshots.get(rn, {})
                    ts_all = snap.get("time_s", [])
                    ys_all = snap.get(col, [])
                    if not ts_all or not ys_all:
                        continue
                    n = min(len(ts_all), len(ys_all))
                    ts_all = ts_all[:n]
                    ys_all = ys_all[:n]
                    t_end   = ts_all[-1]
                    t_start = t_end - self._window_s
                    offset = 0
                    for offset in range(n):
                        if ts_all[offset] >= t_start:
                            break
                    ts = ts_all[offset:]
                    ys = ys_all[offset:]
                    line = self._lines[g_idx].get((rn, col))
                    if line is not None:
                        line.set_data(ts, ys)
            ax.relim()
            ax.autoscale_view()

        self._fig.canvas.draw_idle()
        plt.pause(0.001)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_cli() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Live timeseries plotter — subscribes to ROS2 topics, "
                    "no log files read or written.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "--controller", "-c",
        choices=[MODE_VS, MODE_GC],
        default=MODE_VS,
        help=f"Controller mode: '{MODE_VS}' or '{MODE_GC}' (default: {MODE_VS})",
    )
    p.add_argument(
        "--namespace", "-n",
        default=None,
        help=(
            f"Topic namespace for robot 1. "
            f"VS default: {DEFAULT_NS_VS}  |  GC default: {DEFAULT_NS_GC}"
        ),
    )
    p.add_argument(
        "--namespace2", default=None,
        help="Topic namespace for robot 2 (enables dual-robot mode)",
    )
    p.add_argument(
        "--window", type=float, default=DEFAULT_WINDOW_S,
        help=f"Rolling time window in seconds (default: {DEFAULT_WINDOW_S})",
    )
    p.add_argument(
        "--interval", type=float, default=DEFAULT_INTERVAL_S,
        help=f"Plot refresh interval in seconds (default: {DEFAULT_INTERVAL_S})",
    )
    return p


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> None:
    args = build_cli().parse_args()

    controller_mode = args.controller

    # Apply mode-aware namespace defaults
    if args.namespace is None:
        args.namespace = DEFAULT_NS_VS if controller_mode == MODE_VS else DEFAULT_NS_GC

    namespaces: Dict[int, str] = {1: args.namespace}
    if args.namespace2 and args.namespace2.strip():
        namespaces[2] = args.namespace2

    robot_nums  = list(namespaces.keys())
    robot_count = len(robot_nums)

    buffers: Dict[int, LiveBuffer] = {
        rn: LiveBuffer(controller_mode) for rn in robot_nums
    }

    rclpy.init()
    node = LivePlotNode(buffers, namespaces, controller_mode)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig_mgr = LiveFigureManager(robot_nums, args.window, robot_count, controller_mode)
    plt.ion()

    ns_str = ", ".join(f"robot{rn}={ns}" for rn, ns in namespaces.items())
    print(f"Live plotter started — controller={controller_mode} — {ns_str}")
    print(f"Rolling window: {args.window}s  |  Refresh: {args.interval}s")
    print("Press Ctrl-C to exit.\n")

    try:
        while True:
            snapshots = {rn: buf.snapshot() for rn, buf in buffers.items()}
            fig_mgr.update(snapshots)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nLive plotter shutting down.")
        plt.close("all")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
