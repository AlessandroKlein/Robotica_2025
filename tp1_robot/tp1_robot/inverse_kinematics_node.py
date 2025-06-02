#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.declare_parameter('wheel_separation', 0.1)
        self.declare_parameter('wheel_radius', 0.035)

        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        self.left_pub = self.create_publisher(Float64, 'left_wheel_cmd', 10)
        self.right_pub = self.create_publisher(Float64, 'right_wheel_cmd', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)

    def twist_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        v_left = (v - (w * self.wheel_separation / 2)) / self.wheel_radius
        v_right = (v + (w * self.wheel_separation / 2)) / self.wheel_radius

        self.left_pub.publish(Float64(data=v_left))
        self.right_pub.publish(Float64(data=v_right))

def main():
    rclpy.init()
    node = InverseKinematicsNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()