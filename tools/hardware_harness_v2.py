#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point, Pose, PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class HardwareHarnessAdaptive(Node):
    def __init__(self) -> None:
        super().__init__("hw_harness_adaptive")

        self.pub1 = self.create_publisher(
            PoseStamped, "/robot1/robot1_variable_stiffness/waypoint_command", 10
        )
        self.pub2 = self.create_publisher(
            PoseStamped, "/robot2/robot2_variable_stiffness/waypoint_command", 10
        )

        qos_fast = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(JointState, "/robot1/joint_states", self.cb_js1, qos_fast)
        self.create_subscription(JointState, "/robot2/joint_states", self.cb_js2, qos_fast)
        self.create_subscription(
            Point,
            "/robot1/robot1_variable_stiffness/end_effector_position",
            self.cb_ee1,
            qos_fast,
        )
        self.create_subscription(
            Point,
            "/robot2/robot2_variable_stiffness/end_effector_position",
            self.cb_ee2,
            qos_fast,
        )
        self.create_subscription(
            Pose,
            "/robot1/robot1_variable_stiffness/cartesian_pose_desired",
            self.cb_des1,
            qos_fast,
        )
        self.create_subscription(
            Pose,
            "/robot2/robot2_variable_stiffness/cartesian_pose_desired",
            self.cb_des2,
            qos_fast,
        )
        self.create_subscription(
            WrenchStamped,
            "/robot1/robot1_variable_stiffness/contact_wrench",
            self.cb_contact1,
            qos_fast,
        )
        self.create_subscription(
            WrenchStamped,
            "/robot2/robot2_variable_stiffness/contact_wrench",
            self.cb_contact2,
            qos_fast,
        )
        self.create_subscription(
            Bool,
            "/robot1/robot1_variable_stiffness/contact_valid",
            self.cb_contact_valid1,
            qos_fast,
        )
        self.create_subscription(
            Bool,
            "/robot2/robot2_variable_stiffness/contact_valid",
            self.cb_contact_valid2,
            qos_fast,
        )
        self.create_subscription(
            Bool,
            "/robot1/robot1_variable_stiffness/waypoint_active",
            self.cb_wp1,
            qos_fast,
        )
        self.create_subscription(
            Bool,
            "/robot2/robot2_variable_stiffness/waypoint_active",
            self.cb_wp2,
            qos_fast,
        )

        self.js1: Optional[JointState] = None
        self.js2: Optional[JointState] = None
        self.ee1: Optional[Point] = None
        self.ee2: Optional[Point] = None
        self.des1: Optional[Pose] = None
        self.des2: Optional[Pose] = None
        self.contact_fx_mag_1 = float("nan")
        self.contact_fx_mag_2 = float("nan")
        self.contact_valid_1 = False
        self.contact_valid_2 = False
        self.wp1_active: Optional[bool] = None
        self.wp2_active: Optional[bool] = None

        self.current_phase = "init"
        self.current_offset_x1 = float("nan")
        self.current_offset_x2 = float("nan")
        self.current_press_offset_x1 = 0.0
        self.current_press_offset_x2 = 0.0
        self.current_contact_mode = "none"
        self.precontact_baseline_fx_1 = float("nan")
        self.precontact_baseline_fx_2 = float("nan")

        self.idle_vel_limit = 1.0
        self.move_vel_limit = 1.5
        self.hold_vel_limit = 0.8
        self.command_repeats = 2
        self.command_dt = 0.05
        self.command_sample_delay = 0.45

        self.precontact_start_x_offset = 0.0
        self.precontact_end_x_offset = -0.0100
        self.hold_x_offset = -0.0100
        self.press_end_x_offset = -0.0140
        self.press_step = 0.0015

        self.contact_force_enter = 0.70
        self.contact_force_delta_enter = 0.10
        self.contact_force_threshold_cap = 0.85
        self.force_balance_tolerance = 0.35
        self.max_side_extra_press = 0.0060
        self.side_press_step = 0.0010
        self.force_diff_abort = 4.0
        self.max_force_mag_abort = 8.0
        self.max_precontact_iterations = 10

        self.log_root = os.environ.get("OMX_LOG_DIR", "/tmp/variable_stiffness_logs")
        self.run_ts = time.strftime("%Y%m%d_%H%M%S")
        os.makedirs(self.log_root, exist_ok=True)

        self.csv_path = os.path.join(self.log_root, f"hardware_harness_adaptive_{self.run_ts}.csv")
        self.sync_csv_path = os.path.join(self.log_root, f"hardware_harness_sync_steps_{self.run_ts}.csv")

        self.log_columns = [
            "timestamp",
            "phase",
            "offset_x1",
            "offset_x2",
            "press_offset_x1",
            "press_offset_x2",
            "contact_mode",
            "robot1_max_vel",
            "robot2_max_vel",
            "ee_x_1",
            "ee_x_2",
            "ee_x_diff",
            "desired_x_1",
            "desired_x_2",
            "desired_x_diff",
            "contact_fx_mag_1",
            "contact_fx_mag_2",
            "contact_fx_diff",
            "baseline_fx_1",
            "baseline_fx_2",
            "contact_threshold_1",
            "contact_threshold_2",
            "contact_valid_1",
            "contact_valid_2",
        ]
        self.sync_columns = [
            "timestamp",
            "phase",
            "command_x_offset",
            "offset_x1",
            "offset_x2",
            "press_offset_x1",
            "press_offset_x2",
            "contact_mode",
            "robot1_max_vel",
            "robot2_max_vel",
            "ee_x_1",
            "ee_x_2",
            "ee_x_diff",
            "desired_x_1",
            "desired_x_2",
            "desired_x_diff",
            "contact_fx_mag_1",
            "contact_fx_mag_2",
            "contact_fx_diff",
            "baseline_fx_1",
            "baseline_fx_2",
            "contact_threshold_1",
            "contact_threshold_2",
            "contact_valid_1",
            "contact_valid_2",
        ]

        self.write_csv_header(self.csv_path, self.log_columns)
        self.write_csv_header(self.sync_csv_path, self.sync_columns)
        self.create_timer(0.05, self.log_snapshot_row)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_js1(self, msg: JointState) -> None:
        self.js1 = msg

    def cb_js2(self, msg: JointState) -> None:
        self.js2 = msg

    def cb_ee1(self, msg: Point) -> None:
        self.ee1 = msg

    def cb_ee2(self, msg: Point) -> None:
        self.ee2 = msg

    def cb_des1(self, msg: Pose) -> None:
        self.des1 = msg

    def cb_des2(self, msg: Pose) -> None:
        self.des2 = msg

    def cb_contact1(self, msg: WrenchStamped) -> None:
        self.contact_fx_mag_1 = abs(msg.wrench.force.x)

    def cb_contact2(self, msg: WrenchStamped) -> None:
        self.contact_fx_mag_2 = abs(msg.wrench.force.x)

    def cb_contact_valid1(self, msg: Bool) -> None:
        self.contact_valid_1 = bool(msg.data)

    def cb_contact_valid2(self, msg: Bool) -> None:
        self.contact_valid_2 = bool(msg.data)

    def cb_wp1(self, msg: Bool) -> None:
        self.wp1_active = bool(msg.data)

    def cb_wp2(self, msg: Bool) -> None:
        self.wp2_active = bool(msg.data)

    def spin_for(self, duration: float, step: float = 0.05) -> None:
        end_t = self.now_s() + duration
        while self.now_s() < end_t:
            rclpy.spin_once(self, timeout_sec=step)

    def finite(self, x: float) -> bool:
        return not math.isnan(x) and not math.isinf(x)

    def clip(self, x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def make_offset_pose(self, x: float, y: float, z: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "offset"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    def publish_offsets(
        self,
        x1: float,
        y1: float,
        z1: float,
        x2: float,
        y2: float,
        z2: float,
        repeats: Optional[int] = None,
        dt: Optional[float] = None,
    ) -> None:
        repeat_count = self.command_repeats if repeats is None else repeats
        step_dt = self.command_dt if dt is None else dt

        self.current_offset_x1 = x1
        self.current_offset_x2 = x2
        self.current_press_offset_x1 = x1
        self.current_press_offset_x2 = x2

        msg1 = self.make_offset_pose(x1, y1, z1)
        msg2 = self.make_offset_pose(x2, y2, z2)

        for _ in range(repeat_count):
            stamp = self.get_clock().now().to_msg()
            msg1.header.stamp = stamp
            msg2.header.stamp = stamp
            self.pub1.publish(msg1)
            self.pub2.publish(msg2)
            rclpy.spin_once(self, timeout_sec=step_dt)

    def wait_for_subscribers(self, timeout: float = 3.0) -> bool:
        t0 = self.now_s()
        while self.now_s() - t0 < timeout:
            if self.pub1.get_subscription_count() > 0 and self.pub2.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def wait_joint_states(self, timeout: float = 5.0) -> bool:
        t0 = self.now_s()
        while self.now_s() - t0 < timeout:
            if self.js1 is not None and self.js2 is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def wait_for_poses(self, timeout: float = 3.0) -> bool:
        t0 = self.now_s()
        while self.now_s() - t0 < timeout:
            if self.ee1 is not None and self.ee2 is not None and self.des1 is not None and self.des2 is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def max_abs_velocity(self, js: Optional[JointState]) -> Optional[float]:
        if js is None or js.velocity is None or len(js.velocity) == 0:
            return None
        return max(abs(v) for v in js.velocity)

    def check_limits(self, js: Optional[JointState], limit: float) -> bool:
        vmax = self.max_abs_velocity(js)
        return vmax is not None and vmax < limit

    def capture_precontact_baseline(self) -> None:
        self.precontact_baseline_fx_1 = self.contact_fx_mag_1 if self.finite(self.contact_fx_mag_1) else 0.0
        self.precontact_baseline_fx_2 = self.contact_fx_mag_2 if self.finite(self.contact_fx_mag_2) else 0.0

    def contact_threshold(self, robot_id: int) -> float:
        baseline = self.precontact_baseline_fx_1 if robot_id == 1 else self.precontact_baseline_fx_2
        if not self.finite(baseline):
            baseline = 0.0
        return min(
            self.contact_force_threshold_cap,
            max(self.contact_force_enter, baseline + self.contact_force_delta_enter),
        )

    def contact_detected(self, robot_id: int) -> bool:
        if robot_id == 1:
            return (
                self.contact_valid_1
                and self.finite(self.contact_fx_mag_1)
                and self.contact_fx_mag_1 >= self.contact_threshold(1)
            )
        return (
            self.contact_valid_2
            and self.finite(self.contact_fx_mag_2)
            and self.contact_fx_mag_2 >= self.contact_threshold(2)
        )

    def wait_for_forward_phase(self, timeout: float = 10.0) -> bool:
        t0 = self.now_s()
        while self.now_s() - t0 < timeout:
            if self.des1 is None or self.des2 is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            start1 = self.des1.position.x
            start2 = self.des2.position.x
            self.spin_for(0.30)
            if self.des1 is None or self.des2 is None:
                continue
            delta1 = self.des1.position.x - start1
            delta2 = self.des2.position.x - start2
            if delta1 < -1e-4 and delta2 < -1e-4:
                return True
        return False

    def compute_press_target(self, x_offset1: float, x_offset2: float) -> Optional[Dict[str, float]]:
        if self.des1 is None or self.des2 is None:
            return None

        return {
            "offset_x1": x_offset1,
            "offset_y1": 0.0,
            "offset_z1": 0.0,
            "offset_x2": x_offset2,
            "offset_y2": 0.0,
            "offset_z2": 0.0,
        }

    def snapshot(self) -> Dict[str, Any]:
        robot1_max_vel = self.max_abs_velocity(self.js1)
        robot2_max_vel = self.max_abs_velocity(self.js2)
        ee_x_1 = self.ee1.x if self.ee1 is not None else float("nan")
        ee_x_2 = self.ee2.x if self.ee2 is not None else float("nan")
        desired_x_1 = self.des1.position.x if self.des1 is not None else float("nan")
        desired_x_2 = self.des2.position.x if self.des2 is not None else float("nan")

        ee_x_diff = ee_x_1 - ee_x_2 if self.finite(ee_x_1) and self.finite(ee_x_2) else float("nan")
        desired_x_diff = desired_x_1 - desired_x_2 if self.finite(desired_x_1) and self.finite(desired_x_2) else float("nan")
        contact_fx_diff = (
            self.contact_fx_mag_1 - self.contact_fx_mag_2
            if self.finite(self.contact_fx_mag_1) and self.finite(self.contact_fx_mag_2)
            else float("nan")
        )

        return {
            "timestamp": self.now_s(),
            "phase": self.current_phase,
            "offset_x1": self.current_offset_x1,
            "offset_x2": self.current_offset_x2,
            "press_offset_x1": self.current_press_offset_x1,
            "press_offset_x2": self.current_press_offset_x2,
            "contact_mode": self.current_contact_mode,
            "robot1_max_vel": robot1_max_vel,
            "robot2_max_vel": robot2_max_vel,
            "ee_x_1": ee_x_1,
            "ee_x_2": ee_x_2,
            "ee_x_diff": ee_x_diff,
            "desired_x_1": desired_x_1,
            "desired_x_2": desired_x_2,
            "desired_x_diff": desired_x_diff,
            "contact_fx_mag_1": self.contact_fx_mag_1,
            "contact_fx_mag_2": self.contact_fx_mag_2,
            "contact_fx_diff": contact_fx_diff,
            "baseline_fx_1": self.precontact_baseline_fx_1,
            "baseline_fx_2": self.precontact_baseline_fx_2,
            "contact_threshold_1": self.contact_threshold(1),
            "contact_threshold_2": self.contact_threshold(2),
            "contact_valid_1": float(self.contact_valid_1),
            "contact_valid_2": float(self.contact_valid_2),
        }

    def write_csv_header(self, path: str, columns: List[str]) -> None:
        with open(path, "w", newline="") as f:
            csv.writer(f).writerow(columns)

    def append_csv_row(self, path: str, columns: List[str], row: Dict[str, Any]) -> None:
        with open(path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
            writer.writerow(row)

    def log_snapshot_row(self) -> None:
        self.append_csv_row(self.csv_path, self.log_columns, self.snapshot())

    def log_sync_step(
        self,
        command_x_offset: float,
    ) -> None:
        row = self.snapshot()
        row["command_x_offset"] = command_x_offset
        self.append_csv_row(self.sync_csv_path, self.sync_columns, row)

    def apply_press_target(self, target: Dict[str, float], contact_mode: str) -> None:
        self.current_contact_mode = contact_mode
        self.publish_offsets(
            x1=target["offset_x1"],
            y1=target["offset_y1"],
            z1=target["offset_z1"],
            x2=target["offset_x2"],
            y2=target["offset_y2"],
            z2=target["offset_z2"],
        )

    def safe_abort(self) -> None:
        self.current_phase = "abort"
        self.get_logger().error("ABORT: sending zero offset to return control to nominal trajectory")
        self.publish_offsets(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, repeats=3, dt=0.05)
        self.spin_for(0.5)

    def stage1_liveness(self) -> Tuple[bool, str, Dict[str, Any]]:
        pubs_ok = self.wait_for_subscribers(timeout=3.0)
        js_ok = self.wait_joint_states(timeout=5.0)
        pose_ok = self.wait_for_poses(timeout=3.0)
        ok = pubs_ok and js_ok and pose_ok
        info = {
            "pub1_subs": self.pub1.get_subscription_count(),
            "pub2_subs": self.pub2.get_subscription_count(),
            "js1_seen": self.js1 is not None,
            "js2_seen": self.js2 is not None,
            "ee1_seen": self.ee1 is not None,
            "ee2_seen": self.ee2 is not None,
            "des1_seen": self.des1 is not None,
            "des2_seen": self.des2 is not None,
            "wp1_status_seen": self.wp1_active is not None,
            "wp2_status_seen": self.wp2_active is not None,
            "contact_valid_topics_seen": self.contact_valid_1 or self.contact_valid_2,
        }
        if ok:
            message = "publishers, joint states, and Cartesian pose topics present"
        elif not pubs_ok:
            message = "waypoint subscribers missing"
        elif not js_ok:
            message = "joint_states missing"
        else:
            message = "Cartesian pose topics missing"
        return ok, message, info

    def stage2_idle(self) -> Tuple[bool, str, Dict[str, Any]]:
        self.current_phase = "idle"
        self.spin_for(0.5)
        info = self.snapshot()
        ok = self.check_limits(self.js1, self.idle_vel_limit) and self.check_limits(self.js2, self.idle_vel_limit)
        return ok, "low velocity idle", info

    def stage3_sync_move(self) -> Tuple[bool, str, Dict[str, Any]]:
        self.current_phase = "precontact"
        stage_info: List[Dict[str, Any]] = []

        if not self.wait_for_forward_phase(timeout=10.0):
            return False, "controllers did not enter forward phase", {"samples": stage_info}

        self.spin_for(0.10)
        self.capture_precontact_baseline()

        offset1 = self.precontact_start_x_offset
        offset2 = self.precontact_start_x_offset
        contact1 = False
        contact2 = False
        for step_index in range(self.max_precontact_iterations):
            if not contact1:
                offset1 = max(offset1 - self.press_step, self.precontact_end_x_offset)
            if not contact2:
                offset2 = max(offset2 - self.press_step, self.precontact_end_x_offset)

            target = self.compute_press_target(offset1, offset2)
            if target is None:
                return False, "desired poses unavailable", {"samples": stage_info}
            contact_mode = "robot1_only" if contact1 and not contact2 else "robot2_only" if contact2 and not contact1 else "none"
            self.apply_press_target(target, contact_mode)
            self.spin_for(self.command_sample_delay)

            contact1 = self.contact_detected(1)
            contact2 = self.contact_detected(2)

            snap = self.snapshot()
            snap["command_x_offset"] = min(offset1, offset2)
            stage_info.append(snap)
            self.log_sync_step(min(offset1, offset2))

            if contact1 and contact2:
                return True, "bilateral contact established", {"samples": stage_info}

            if not self.check_limits(self.js1, self.move_vel_limit) or not self.check_limits(self.js2, self.move_vel_limit):
                return False, "velocity spike during precontact approach", {"samples": stage_info}

        return False, "bilateral contact not established", {"samples": stage_info}

    def stage4_hold(self) -> Tuple[bool, str, Dict[str, Any]]:
        self.current_phase = "hold"
        target = self.compute_press_target(self.current_press_offset_x1, self.current_press_offset_x2)
        if target is None:
            return False, "desired poses unavailable", self.snapshot()

        self.apply_press_target(target, "both")
        self.spin_for(0.8)
        info = self.snapshot()
        ok = self.check_limits(self.js1, self.hold_vel_limit) and self.check_limits(self.js2, self.hold_vel_limit)
        return ok, "hold stable", info

    def stage5_adaptive_press(self) -> Tuple[bool, str, Dict[str, Any]]:
        self.current_phase = "compress"
        stage_info: List[Dict[str, Any]] = []

        offset1 = self.current_press_offset_x1
        offset2 = self.current_press_offset_x2
        while min(offset1, offset2) >= self.press_end_x_offset - 1e-9:
            offset1 -= self.press_step
            offset2 -= self.press_step

            contact1 = self.contact_detected(1)
            contact2 = self.contact_detected(2)
            if contact1 and contact2 and self.finite(self.contact_fx_mag_1) and self.finite(self.contact_fx_mag_2):
                if self.contact_fx_mag_1 + self.force_balance_tolerance < self.contact_fx_mag_2:
                    offset1 = max(offset1 - self.side_press_step, self.press_end_x_offset - self.max_side_extra_press)
                    contact_mode = "push_robot1_more"
                elif self.contact_fx_mag_2 + self.force_balance_tolerance < self.contact_fx_mag_1:
                    offset2 = max(offset2 - self.side_press_step, self.press_end_x_offset - self.max_side_extra_press)
                    contact_mode = "push_robot2_more"
                else:
                    contact_mode = "balanced"
            elif contact1 and not contact2:
                offset1 = self.current_press_offset_x1
                offset2 = max(offset2 - self.side_press_step, self.press_end_x_offset - self.max_side_extra_press)
                contact_mode = "push_robot2_to_contact"
            elif contact2 and not contact1:
                offset2 = self.current_press_offset_x2
                offset1 = max(offset1 - self.side_press_step, self.press_end_x_offset - self.max_side_extra_press)
                contact_mode = "push_robot1_to_contact"
            else:
                contact_mode = "seeking_contact"

            target = self.compute_press_target(offset1, offset2)
            if target is None:
                return False, "desired poses unavailable", {"samples": stage_info}

            self.apply_press_target(target, contact_mode)
            self.spin_for(self.command_sample_delay)

            snap = self.snapshot()
            snap["command_x_offset"] = min(offset1, offset2)
            stage_info.append(snap)
            self.log_sync_step(min(offset1, offset2))

            self.current_press_offset_x1 = offset1
            self.current_press_offset_x2 = offset2

            if not self.check_limits(self.js1, self.move_vel_limit) or not self.check_limits(self.js2, self.move_vel_limit):
                return False, "velocity spike during compression", {"samples": stage_info}

            f1 = snap["contact_fx_mag_1"]
            f2 = snap["contact_fx_mag_2"]
            fd = snap["contact_fx_diff"]
            both_contact = bool(snap["contact_valid_1"]) and bool(snap["contact_valid_2"])
            if both_contact and self.finite(fd) and abs(fd) > self.force_diff_abort:
                return False, "force asymmetry too high", {"samples": stage_info}
            if both_contact and self.finite(f1) and self.finite(f2) and max(f1, f2) > self.max_force_mag_abort:
                return False, "contact force too high", {"samples": stage_info}

        return True, "adaptive bilateral compression completed", {"samples": stage_info}

    def run(self) -> List[Dict[str, Any]]:
        stages = [
            self.stage1_liveness,
            self.stage2_idle,
            self.stage3_sync_move,
            self.stage4_hold,
            self.stage5_adaptive_press,
        ]

        results: List[Dict[str, Any]] = []

        for idx, stage_fn in enumerate(stages, start=1):
            ok, msg, info = stage_fn()
            record = {
                "stage": idx,
                "pass": ok,
                "message": msg,
                "info": info,
            }
            results.append(record)
            self.get_logger().info(f"Stage {idx}: {msg} -> {ok}")

            if not ok:
                self.safe_abort()
                return results

        self.current_phase = "done"
        self.get_logger().info("ALL STAGES PASSED")
        return results


def main() -> None:
    rclpy.init()
    node = HardwareHarnessAdaptive()
    try:
        out = node.run()
        print(json.dumps(out, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
