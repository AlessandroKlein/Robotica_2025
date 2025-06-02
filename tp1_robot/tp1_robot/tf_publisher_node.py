#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class TFPublisherNode(Node):
    def __init__(self):
        super().__init__('tf_publisher_node')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Suscripción a odometría
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        # Posición
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        # Orientación
        transform.transform.rotation = msg.pose.pose.orientation

        # Publicar transformación
        self.tf_broadcaster.sendTransform(transform)

def main():
    rclpy.init()
    node = TFPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()