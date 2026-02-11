#!/usr/bin/env python3
"""
Data Logger for Variable Stiffness Controller.

This node subscribes to controller outputs and logs them to CSV files
for post-experiment analysis.

Logged data includes:
- Actual and desired Cartesian poses
- End effector position and orientation
- End effector velocities
- Joint velocities
- Commanded torques
- Stiffness/damping state
- Jacobian values
"""

import csv
import os
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import Float64MultiArray


class DataLogger(Node):
    def __init__(self):
        super().__init__('csv_data_logger')

        # Declare parameters
        self.declare_parameter('controller_name', 'variable_stiffness_controller')
        self.declare_parameter('output_dir', '/tmp/variable_stiffness_logs')
        self.declare_parameter('log_rate_hz', 100.0)  # Max logging rate
        self.declare_parameter('log_jacobian', False)  # Jacobian can be large

        self.controller_name = self.get_parameter('controller_name').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.log_rate_hz = self.get_parameter('log_rate_hz').get_parameter_value().double_value
        self.log_jacobian = self.get_parameter('log_jacobian').get_parameter_value().bool_value

        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)

        # Generate timestamped filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_filename = os.path.join(
            self.output_dir,
            f'variable_stiffness_log_{timestamp}.csv'
        )

        # Rate limiting
        self.min_period = 1.0 / self.log_rate_hz if self.log_rate_hz > 0 else 0.0
        self.last_log_time = 0.0

        # Data buffers (latest values from each topic)
        self.actual_pose: Optional[Pose] = None
        self.desired_pose: Optional[Pose] = None
        self.ee_position: Optional[Point] = None
        self.ee_orientation: Optional[Vector3] = None
        self.ee_velocity: Optional[list] = None
        self.joint_velocity: Optional[list] = None
        self.torques: Optional[list] = None
        self.stiffness_state: Optional[list] = None
        self.jacobian: Optional[list] = None

        # Get namespace
        ns = self.get_namespace()
        if ns == '/':
            ns = ''
        base_topic = f'{ns}/{self.controller_name}'

        # QoS for subscriptions
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriptions
        self.create_subscription(
            Pose, f'{base_topic}/cartesian_pose_actual',
            self.actual_pose_cb, qos)
        self.create_subscription(
            Pose, f'{base_topic}/cartesian_pose_desired',
            self.desired_pose_cb, qos)
        self.create_subscription(
            Point, f'{base_topic}/end_effector_position',
            self.ee_pos_cb, qos)
        self.create_subscription(
            Vector3, f'{base_topic}/end_effector_orientation',
            self.ee_orient_cb, qos)
        self.create_subscription(
            Float64MultiArray, f'{base_topic}/end_effector_velocities',
            self.ee_vel_cb, qos)
        self.create_subscription(
            Float64MultiArray, f'{base_topic}/joint_velocities',
            self.joint_vel_cb, qos)
        self.create_subscription(
            Float64MultiArray, f'{base_topic}/torque_values',
            self.torque_cb, qos)
        self.create_subscription(
            Float64MultiArray, f'{base_topic}/stiffness_state',
            self.stiffness_cb, qos)

        if self.log_jacobian:
            self.create_subscription(
                Float64MultiArray, f'{base_topic}/jacobian_values',
                self.jacobian_cb, qos)

        # Open CSV file and write header
        self.csv_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = self.init_csv()

        # Timer for periodic logging
        if self.log_rate_hz > 0:
            self.timer = self.create_timer(self.min_period, self.log_data)

        self.get_logger().info(f'Logging to: {self.log_filename}')

    def init_csv(self):
        """Initialize CSV writer with header row."""
        headers = [
            'timestamp',
            # Actual pose
            'actual_x', 'actual_y', 'actual_z',
            'actual_qx', 'actual_qy', 'actual_qz', 'actual_qw',
            # Desired pose
            'desired_x', 'desired_y', 'desired_z',
            'desired_qx', 'desired_qy', 'desired_qz', 'desired_qw',
            # EE position/orientation
            'ee_x', 'ee_y', 'ee_z',
            'ee_roll', 'ee_pitch', 'ee_yaw',
            # EE velocities (6D)
            'ee_vx', 'ee_vy', 'ee_vz', 'ee_wx', 'ee_wy', 'ee_wz',
            # Joint velocities (4 joints for OMX)
            'jv1', 'jv2', 'jv3', 'jv4',
            # Torques (4 joints)
            'tau1', 'tau2', 'tau3', 'tau4',
            # Stiffness state (Ktx, Kty, Ktz, Krx, Kry, Krz, Dtx, Dty, Dtz, Drx, Dry, Drz)
            'Ktx', 'Kty', 'Ktz', 'Krx', 'Kry', 'Krz',
            'Dtx', 'Dty', 'Dtz', 'Drx', 'Dry', 'Drz',
        ]

        if self.log_jacobian:
            # Add Jacobian columns (6x4 = 24 elements for OMX)
            for i in range(6):
                for j in range(4):
                    headers.append(f'J{i}{j}')

        writer = csv.DictWriter(self.csv_file, fieldnames=headers)
        writer.writeheader()
        return writer

    def actual_pose_cb(self, msg: Pose):
        self.actual_pose = msg

    def desired_pose_cb(self, msg: Pose):
        self.desired_pose = msg

    def ee_pos_cb(self, msg: Point):
        self.ee_position = msg

    def ee_orient_cb(self, msg: Vector3):
        self.ee_orientation = msg

    def ee_vel_cb(self, msg: Float64MultiArray):
        self.ee_velocity = list(msg.data)

    def joint_vel_cb(self, msg: Float64MultiArray):
        self.joint_velocity = list(msg.data)

    def torque_cb(self, msg: Float64MultiArray):
        self.torques = list(msg.data)

    def stiffness_cb(self, msg: Float64MultiArray):
        self.stiffness_state = list(msg.data)

    def jacobian_cb(self, msg: Float64MultiArray):
        self.jacobian = list(msg.data)

    def log_data(self):
        """Write current data to CSV file."""
        now = self.get_clock().now().nanoseconds / 1e9

        # Rate limiting
        if now - self.last_log_time < self.min_period:
            return
        self.last_log_time = now

        row = {'timestamp': now}

        # Actual pose
        if self.actual_pose:
            row.update({
                'actual_x': self.actual_pose.position.x,
                'actual_y': self.actual_pose.position.y,
                'actual_z': self.actual_pose.position.z,
                'actual_qx': self.actual_pose.orientation.x,
                'actual_qy': self.actual_pose.orientation.y,
                'actual_qz': self.actual_pose.orientation.z,
                'actual_qw': self.actual_pose.orientation.w,
            })

        # Desired pose
        if self.desired_pose:
            row.update({
                'desired_x': self.desired_pose.position.x,
                'desired_y': self.desired_pose.position.y,
                'desired_z': self.desired_pose.position.z,
                'desired_qx': self.desired_pose.orientation.x,
                'desired_qy': self.desired_pose.orientation.y,
                'desired_qz': self.desired_pose.orientation.z,
                'desired_qw': self.desired_pose.orientation.w,
            })

        # EE position
        if self.ee_position:
            row.update({
                'ee_x': self.ee_position.x,
                'ee_y': self.ee_position.y,
                'ee_z': self.ee_position.z,
            })

        # EE orientation
        if self.ee_orientation:
            row.update({
                'ee_roll': self.ee_orientation.x,
                'ee_pitch': self.ee_orientation.y,
                'ee_yaw': self.ee_orientation.z,
            })

        # EE velocities
        if self.ee_velocity and len(self.ee_velocity) >= 6:
            row.update({
                'ee_vx': self.ee_velocity[0],
                'ee_vy': self.ee_velocity[1],
                'ee_vz': self.ee_velocity[2],
                'ee_wx': self.ee_velocity[3],
                'ee_wy': self.ee_velocity[4],
                'ee_wz': self.ee_velocity[5],
            })

        # Joint velocities
        if self.joint_velocity:
            for i, v in enumerate(self.joint_velocity[:4]):
                row[f'jv{i+1}'] = v

        # Torques
        if self.torques:
            for i, t in enumerate(self.torques[:4]):
                row[f'tau{i+1}'] = t

        # Stiffness state
        if self.stiffness_state and len(self.stiffness_state) >= 12:
            row.update({
                'Ktx': self.stiffness_state[0],
                'Kty': self.stiffness_state[1],
                'Ktz': self.stiffness_state[2],
                'Krx': self.stiffness_state[3],
                'Kry': self.stiffness_state[4],
                'Krz': self.stiffness_state[5],
                'Dtx': self.stiffness_state[6],
                'Dty': self.stiffness_state[7],
                'Dtz': self.stiffness_state[8],
                'Drx': self.stiffness_state[9],
                'Dry': self.stiffness_state[10],
                'Drz': self.stiffness_state[11],
            })

        # Jacobian
        if self.log_jacobian and self.jacobian:
            for idx, val in enumerate(self.jacobian[:24]):
                i, j = idx // 4, idx % 4
                row[f'J{i}{j}'] = val

        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def destroy_node(self):
        """Close CSV file on shutdown."""
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f'Closed log file: {self.log_filename}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
