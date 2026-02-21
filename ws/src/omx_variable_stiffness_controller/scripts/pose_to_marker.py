#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

class PoseToMarker(Node):
    def __init__(self):
        super().__init__('pose_to_marker')
        self.sub_actual = self.create_subscription(Pose,
            '/omx/variable_stiffness_controller/cartesian_pose_actual', self.cb_actual, 10)
        self.sub_desired = self.create_subscription(Pose,
            '/omx/variable_stiffness_controller/cartesian_pose_desired', self.cb_desired, 10)
        self.pub_marker = self.create_publisher(Marker, '/omx/variable_stiffness_controller/cartesian_pose_actual_marker', 10)
        self.pub_marker_des = self.create_publisher(Marker, '/omx/variable_stiffness_controller/cartesian_pose_desired_marker', 10)

    def make_marker(self, pose, ns, color):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = pose
        m.scale.x = 0.06
        m.scale.y = 0.02
        m.scale.z = 0.02
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1.0
        return m

    def cb_actual(self, msg):
        m = self.make_marker(msg, 'actual', (0.0, 0.8, 0.0))
        self.pub_marker.publish(m)

    def cb_desired(self, msg):
        m = self.make_marker(msg, 'desired', (0.8, 0.0, 0.0))
        self.pub_marker_des.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
