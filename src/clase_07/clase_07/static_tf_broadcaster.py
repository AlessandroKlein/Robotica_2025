import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat
import numpy as np

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        # Timer para retransmitir periódicamente (latcheo)
        self.create_timer(0.5, self.broadcast_transforms)

    def broadcast_transforms(self):
        now = self.get_clock().now().to_msg()

        # 1) robotA en world
        self.send_tf(now,
            parent_frame='world',
            child_frame='robotA',
            t=(2.0, 1.0, 0.0),
            yaw=np.deg2rad(30.0)
        )
        # 2) robotB en robotA
        self.send_tf(now,
            parent_frame='robotA',
            child_frame='robotB',
            t=(0.5, np.sqrt(3)/2.0, 0.0),
            yaw=np.deg2rad(60.0)
        )
        # 3) obstacle1 en world
        self.send_tf(now,
            parent_frame='world',
            child_frame='obstacle1',
            t=(0.0, 3.0, 0.0),
            yaw=0.0
        )
        # 4) obstacle2 en robotA
        self.send_tf(now,
            parent_frame='robotA',
            child_frame='obstacle2',
            t=(np.sqrt(3), -1.0, 0.0),
            yaw=0.0
        )

    def send_tf(self, stamp, parent_frame, child_frame, t, yaw):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = parent_frame
        tf.child_frame_id = child_frame
        tf.transform.translation.x = t[0]
        tf.transform.translation.y = t[1]
        tf.transform.translation.z = t[2]
        # roll = pitch = 0
        q = euler2quat(0.0, 0.0, yaw)  # devuelve (w, x, y, z)
        tf.transform.rotation.x = q[1]
        tf.transform.rotation.y = q[2]
        tf.transform.rotation.z = q[3]
        tf.transform.rotation.w = q[0]
        # Envío del transform
        self.broadcaster.sendTransform(tf)
        # Loguear la información en terminal
        self.get_logger().info(
            f"[{child_frame} ← {parent_frame}] traslación = ({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}), yaw = {np.rad2deg(yaw):.1f}°"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StaticTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()