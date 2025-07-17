import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.publisher_left = self.create_publisher(Float64MultiArray, '/velocity_controller_left/commands', 10)
        self.publisher_right = self.create_publisher(Float64MultiArray, '/velocity_controller_right/commands', 10)

        # Declare and get parameters for wheel radius and wheel separation
        self.declare_parameter('wheel_radius', 0.07) # Default value
        self.declare_parameter('wheel_separation', 0.135) # Default value

        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.b = self.get_parameter('wheel_separation').get_parameter_value().double_value

        self.get_logger().info(f'CmdVelListener initialized with wheel_radius: {self.r} and wheel_separation: {self.b}')

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Inverse Kinematics
        # phi_dot_R = (vx + (b/2)*wz) / r
        # phi_dot_L = (vx - (b/2)*wz) / r
        
        # From image_bd139e.png (Modelo cinemático inverso):
        # phi_dot_R = 1/r * (x_dot + b/2 * theta_dot)
        # phi_dot_L = 1/r * (x_dot - b/2 * theta_dot)
        
        phi_dot_R = (1 / self.r) * (linear_x + (self.b / 2) * angular_z)
        phi_dot_L = (1 / self.r) * (linear_x - (self.b / 2) * angular_z)

        # Create Float64MultiArray messages
        msg_left = Float64MultiArray()
        msg_left.data = [phi_dot_L]
        self.publisher_left.publish(msg_left)

        msg_right = Float64MultiArray()
        msg_right.data = [phi_dot_R]
        self.publisher_right.publish(msg_right)

        self.get_logger().debug(f'Received cmd_vel: linear_x={linear_x}, angular_z={angular_z}')
        self.get_logger().debug(f'Published wheel speeds: left={phi_dot_L}, right={phi_dot_R}')

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_listener = CmdVelListener()
    rclpy.spin(cmd_vel_listener)
    cmd_vel_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()