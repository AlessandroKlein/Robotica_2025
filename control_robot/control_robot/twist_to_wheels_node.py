import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# No necesitamos Float64MultiArray si solo publicamos un valor por tópico
# from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64 # Importamos Float64

class TwistToWheelsNode(Node):
    def __init__(self):
        super().__init__('twist_to_wheels_node')
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('track_width', 0.3)
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.track_width = self.get_parameter('track_width').get_parameter_value().double_value
        self.get_logger().info(f'Parámetros del robot: Radio de rueda={self.wheel_radius} m, Ancho de vía={self.track_width} m')
        
        # Suscriptor al tópico /cmd_vel
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 1)
        
        # Publicadores para las velocidades de las ruedas (esperan Float64)
        self.left_wheel_pub = self.create_publisher(Float64, 'left_wheel_cmd', 1)
        self.wheel_pub = self.create_publisher(Float64, 'right_wheel_cmd', 1) # Corrected typo here
        
        self.get_logger().info('Nodo TwistToWheelsNode iniciado. Suscribiendo a /cmd_vel y publicando en /left_wheel_cmd, /right_wheel_cmd.')

    def twist_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Calcular las velocidades angulares de cada rueda
        left_wheel_angular_vel = (linear_vel - angular_vel * (self.track_width / 2.0)) / self.wheel_radius
        right_wheel_angular_vel = (linear_vel + angular_vel * (self.track_width / 2.0)) / self.wheel_radius

        # Crear mensajes Float64 y asignar directamente el valor
        left_cmd_msg = Float64()
        left_cmd_msg.data = left_wheel_angular_vel # Asignar el float directamente al campo 'data'

        right_cmd_msg = Float64()
        right_cmd_msg.data = right_wheel_angular_vel # Asignar el float directamente al campo 'data'

        self.get_logger().info(f'Recibido Twist: v={linear_vel}, w={angular_vel}. Publicando L={left_wheel_angular_vel:.2f}, R={right_wheel_angular_vel:.2f}')
        
        # Publicar los mensajes Float64
        self.left_wheel_pub.publish(left_cmd_msg)
        self.right_wheel_pub.publish(right_cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    twist_to_wheels_node = TwistToWheelsNode()
    rclpy.spin(twist_to_wheels_node)
    twist_to_wheels_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
