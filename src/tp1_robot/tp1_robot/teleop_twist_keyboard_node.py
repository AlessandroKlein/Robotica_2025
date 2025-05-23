import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class DiffBotController(Node):
    def __init__(self):
        super().__init__('diffbot_controller')
        self.wheel_separation = 0.08  # metros
        self.wheel_radius = 0.035    # metros

        self.left_pub = self.create_publisher(Float64, 'left_wheel_cmd', 10)
        self.right_pub = self.create_publisher(Float64, 'right_wheel_cmd', 10)

        self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)

    def twist_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        v_left = (v - (w * self.wheel_separation / 2)) / self.wheel_radius
        v_right = (v + (w * self.wheel_separation / 2)) / self.wheel_radius

        self.left_pub.publish(Float64(data=v_left))
        self.right_pub.publish(Float64(data=v_right))

def main():
    rclpy.init()
    controller = DiffBotController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()