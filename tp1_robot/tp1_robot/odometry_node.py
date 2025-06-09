#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import transforms3d  # Necesario: pip install transforms3d

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.declare_parameter('wheel_separation', 0.1)
        self.declare_parameter('wheel_radius', 0.035)

        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Suscripción a comandos de velocidad de las ruedas
        self.create_subscription(Float64, 'left_wheel_cmd', self.left_vel_callback, 10)
        self.create_subscription(Float64, 'right_wheel_cmd', self.right_vel_callback, 10)

        # Publicador de odometría
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

    def left_vel_callback(self, msg):
        self.left_velocity = msg.data

    def right_vel_callback(self, msg):
        self.right_velocity = msg.data

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        v = self.wheel_radius * (self.right_velocity + self.left_velocity) / 2
        w = (self.wheel_radius * (self.right_velocity - self.left_velocity)) / self.wheel_separation

        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = w * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        self.publish_odometry(v, w)

    def publish_odometry(self, linear_velocity, angular_velocity):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Posición
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        quat = transforms3d.euler.euler2quat(0, 0, self.theta)
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        odom.pose.pose.orientation.w = quat[0]

        # Velocidad
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = OdometryNode()
    rate = node.create_rate(50)  # 50 Hz
    try:
        while rclpy.ok():
            node.update_odometry()
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()