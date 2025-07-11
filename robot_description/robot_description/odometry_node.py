#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Parámetros del robot
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.2)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        # Variables de estado
        self.left_wheel_prev_pos = 0.0
        self.right_wheel_prev_pos = 0.0
        self.prev_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Subscripción a JointState
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Publicador de Odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Transformación odom -> base_link
        self.timer = self.create_timer(0.05, self.publish_odometry)

    def joint_state_callback(self, msg):
        try:
            right_wheel_idx = msg.name.index('right_wheel_joint')
            left_wheel_idx = msg.name.index('left_wheel_joint')

            right_pos = msg.position[right_wheel_idx]
            left_pos = msg.position[left_wheel_idx]

            current_time = msg.header.stamp
            dt = (current_time.nanosec - self.prev_time.nanosec) / 1e9

            # Diferencia de posición de ruedas
            d_left = left_pos - self.left_wheel_prev_pos
            d_right = right_pos - self.right_wheel_prev_pos

            # Distancia recorrida por cada rueda
            d_left *= self.wheel_radius
            d_right *= self.wheel_radius

            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / self.wheel_separation

            # Actualizar posición
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)
            self.theta += d_theta

            # Actualizar valores previos
            self.left_wheel_prev_pos = left_pos
            self.right_wheel_prev_pos = right_pos
            self.prev_time = current_time

        except ValueError:
            self.get_logger().warn("No se encontraron juntas de ruedas en JointState")

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()