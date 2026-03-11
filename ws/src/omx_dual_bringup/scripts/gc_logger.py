#!/usr/bin/env python3
"""
Data logger for Gravity Compensation mode.

Subscribes to /joint_states (published by joint_state_broadcaster alongside
OmxGravityCompController) and logs joint positions, velocities and compensation
efforts to a CSV file at a configurable rate.

Parameters
----------
output_dir   : str   — directory for the CSV file (created if needed)
log_rate_hz  : float — snapshot rate (default 100 Hz)
"""

import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class GravityCompLogger(Node):
    def __init__(self):
        super().__init__('gc_data_logger')

        self.declare_parameter('output_dir', '/tmp/gravity_comp_logs')
        self.declare_parameter('log_rate_hz', 100.0)

        self.output_dir = self.get_parameter('output_dir').value
        self.log_rate_hz = float(self.get_parameter('log_rate_hz').value)

        os.makedirs(self.output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = os.path.join(
            self.output_dir, f'gravity_comp_{timestamp}.csv'
        )

        self.min_period = 1.0 / self.log_rate_hz if self.log_rate_hz > 0 else 0.0
        self.last_log_time = 0.0

        self.latest_msg = None

        ns = self.get_namespace()
        if ns == '/':
            ns = ''
        topic = f'{ns}/joint_states'

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, topic, self._joint_states_cb, qos)

        headers = [
            'timestamp',
            'jpos1', 'jpos2', 'jpos3', 'jpos4',
            'jvel1', 'jvel2', 'jvel3', 'jvel4',
            'jeff1', 'jeff2', 'jeff3', 'jeff4',
        ]
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=headers)
        self.csv_writer.writeheader()

        if self.log_rate_hz > 0:
            self.timer = self.create_timer(self.min_period, self._log_data)

        self.get_logger().info(f'GC logger → {self.csv_filename}')

    def _joint_states_cb(self, msg: JointState):
        self.latest_msg = msg

    def _log_data(self):
        if self.latest_msg is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time < self.min_period:
            return
        self.last_log_time = now

        msg = self.latest_msg
        n = min(4, len(msg.position))
        row = {'timestamp': now}
        for i in range(n):
            row[f'jpos{i+1}'] = msg.position[i] if i < len(msg.position) else ''
            row[f'jvel{i+1}'] = msg.velocity[i] if i < len(msg.velocity) else ''
            row[f'jeff{i+1}'] = msg.effort[i]   if i < len(msg.effort)   else ''

        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def destroy_node(self):
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f'Closed log: {self.csv_filename}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GravityCompLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
