#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat
import numpy as np

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('hz', 30.0)
        self.radius = self.get_parameter('radius').value
        hz = self.get_parameter('hz').value
        self.broadcaster = TransformBroadcaster(self)
        period = 1.0 / hz
        self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        # tiempo t en radianes, 1 vuelta cada 4 s ⇒ velocidad angular = π/2 rad/s
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        t = (sec + nsec*1e-9) * (np.pi/2.0)

        x = self.radius * np.cos(t)
        y = self.radius * np.sin(t)
        # tangente: orientación ortogonal a radio
        yaw = np.arctan2(np.cos(t), -np.sin(t))

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'world'
        tf.child_frame_id = 'robotC'
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)
        tf.transform.translation.z = 0.0
        q = euler2quat(0.0, 0.0, float(yaw))
        tf.transform.rotation.x = q[1]
        tf.transform.rotation.y = q[2]
        tf.transform.rotation.z = q[3]
        tf.transform.rotation.w = q[0]

        # Envío del transform
        self.broadcaster.sendTransform(tf)
        # Loguear la información en terminal
        self.get_logger().info(
            f"[robotC ← world] traslación = ({x:.2f}, {y:.2f}, 0.00), yaw = {np.rad2deg(yaw):.1f}°"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()