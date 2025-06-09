import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys
import select
import termios
import tty

class DiffBotController(Node):
    def __init__(self):
        super().__init__('diffbot_controller')
        self.declare_parameter('wheel_separation', 0.08)
        self.declare_parameter('wheel_radius', 0.035)

        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

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

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

msg = """
Reading from the keyboard!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i : forward
, : backward
j : left
l : right
k : stop
CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'm': (-1, 1),
    ',': (-1, 0),
    '.': (-1, -1),
}

settings = termios.tcgetattr(sys.stdin)

def main():
    rclpy.init()
    controller = DiffBotController()
    teleop = TeleopNode()

    print(msg)
    try:
        while True:
            key = getKey()
            if key in moveBindings:
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                teleop.send_velocity(x * 0.5, th * 0.5)
            elif key == '\x03':
                break
            rclpy.spin_once(controller, timeout_sec=0.1)
    except Exception as e:
        print(e)
    finally:
        teleop.send_velocity(0.0, 0.0)
        rclpy.shutdown()

if __name__ == '__main__':
    main()