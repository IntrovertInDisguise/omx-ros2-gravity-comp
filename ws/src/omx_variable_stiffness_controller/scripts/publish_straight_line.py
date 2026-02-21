#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time
import math

class StraightLinePublisher(Node):
    def __init__(self):
        super().__init__('vs_straight_line_pub')
        self.pub = self.create_publisher(Pose, '/omx/variable_stiffness_controller/cartesian_pose_desired', 10)
        self.start = (0.25, 0.0, 0.15)
        self.end = (0.20, 0.0, 0.15)
        self.duration = 10.0
        self.t0 = time.time()
        self.timer = self.create_timer(0.02, self.timer_cb)
        self.done = False

    def cos_interpolate(self, t, T):
        s = min(max(t / T, 0.0), 1.0)
        return 0.5 * (1.0 - math.cos(s * math.pi))

    def timer_cb(self):
        if self.done:
            return
        elapsed = time.time() - self.t0
        if elapsed > self.duration:
            s = 1.0
            self.done = True
        else:
            s = self.cos_interpolate(elapsed, self.duration)

        x = self.start[0] * (1.0 - s) + self.end[0] * s
        y = self.start[1] * (1.0 - s) + self.end[1] * s
        z = self.start[2] * (1.0 - s) + self.end[2] * s

        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        self.pub.publish(msg)

        if self.done:
            self.get_logger().info('Straight-line publish complete')

def main(args=None):
    rclpy.init(args=args)
    node = StraightLinePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
