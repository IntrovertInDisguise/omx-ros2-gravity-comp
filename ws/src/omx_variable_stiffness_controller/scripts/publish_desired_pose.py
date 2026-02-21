#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import time

class PosePublisher(Node):
    def __init__(self):
        super().__init__('vs_desired_pose_pub')
        self.pub = self.create_publisher(Pose, '/omx/variable_stiffness_controller/cartesian_pose_desired', 10)
        self.t0 = time.time()
        self.timer = self.create_timer(0.02, self.timer_cb)  # 50 Hz

        # default start/end (match package config)
        self.start = (0.25, 0.0, 0.15)
        self.end = (0.20, 0.0, 0.15)
        self.period = 10.0

    def cos_interpolate(self, t, T):
        s = min(max(t / T, 0.0), 1.0)
        return 0.5 * (1.0 - math.cos(s * math.pi))

    def timer_cb(self):
        now = time.time() - self.t0
        # ping-pong between start->end->start
        phase = now % (2.0 * self.period)
        if phase <= self.period:
            s = self.cos_interpolate(phase, self.period)
            target = (
                self.start[0] * (1.0 - s) + self.end[0] * s,
                self.start[1] * (1.0 - s) + self.end[1] * s,
                self.start[2] * (1.0 - s) + self.end[2] * s,
            )
        else:
            phase2 = phase - self.period
            s = self.cos_interpolate(phase2, self.period)
            target = (
                self.end[0] * (1.0 - s) + self.start[0] * s,
                self.end[1] * (1.0 - s) + self.start[1] * s,
                self.end[2] * (1.0 - s) + self.start[2] * s,
            )

        msg = Pose()
        msg.position.x = float(target[0])
        msg.position.y = float(target[1])
        msg.position.z = float(target[2])
        # keep orientation identity
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
