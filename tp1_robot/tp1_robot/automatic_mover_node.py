# tp1_robot/automatic_mover_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class AutomaticMover(Node):
    def __init__(self):
        super().__init__('automatic_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def move_straight(self, duration=5.0, linear_speed=0.2):
        msg = Twist()
        msg.linear.x = linear_speed
        self._publish_for_duration(msg, duration)

    def turn(self, duration=3.0, angular_speed=0.5):
        msg = Twist()
        msg.angular.z = angular_speed
        self._publish_for_duration(msg, duration)

    def stop(self):
        msg = Twist()
        self.publisher_.publish(msg)

    def _publish_for_duration(self, msg, duration):
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < duration:
            self.publisher_.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

def main():
    rclpy.init()
    mover = AutomaticMover()
    try:
        mover.move_straight(duration=3.0)
        mover.turn(duration=2.0)
        mover.move_straight(duration=3.0)
        mover.stop()
    except KeyboardInterrupt:
        pass
    finally:
        mover.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()