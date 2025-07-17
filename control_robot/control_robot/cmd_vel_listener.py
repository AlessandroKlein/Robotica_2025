import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

# =========================================================
#   Nodo: CmdVelListener
#   Descripción:
#   - Escucha mensajes de velocidad (Twist) en /cmd_vel
#   - Calcula las velocidades angulares de las ruedas usando cinemática inversa
#   - Publica comandos de velocidad a los controladores de las ruedas izquierda y derecha
# =========================================================

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')
        # Suscripción al tópico /cmd_vel (mensajes Twist)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # Evita advertencia de variable no usada

        # Publicadores para los controladores de velocidad de cada rueda
        self.publisher_left = self.create_publisher(Float64MultiArray, '/velocity_controller_left/commands', 10)
        self.publisher_right = self.create_publisher(Float64MultiArray, '/velocity_controller_right/commands', 10)

        # Declarar y obtener parámetros para el radio y la separación de las ruedas
        self.declare_parameter('wheel_radius', 0.07) # Valor por defecto
        self.declare_parameter('wheel_separation', 0.135) # Valor por defecto

        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.b = self.get_parameter('wheel_separation').get_parameter_value().double_value

        self.get_logger().info(f'CmdVelListener inicializado con radio de rueda: {self.r} y separación entre ruedas: {self.b}')

    def cmd_vel_callback(self, msg):
        # Extraer velocidades lineal y angular del mensaje Twist
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Cinemática inversa para robot diferencial:
        # phi_dot_R = (vx + (b/2)*wz) / r
        # phi_dot_L = (vx - (b/2)*wz) / r
        # Donde:
        #   vx: velocidad lineal (m/s)
        #   wz: velocidad angular (rad/s)
        #   b: separación entre ruedas (m)
        #   r: radio de rueda (m)
        phi_dot_R = (1 / self.r) * (linear_x + (self.b / 2) * angular_z)
        phi_dot_L = (1 / self.r) * (linear_x - (self.b / 2) * angular_z)

        # Crear mensajes Float64MultiArray para cada rueda
        msg_left = Float64MultiArray()
        msg_left.data = [phi_dot_L]
        self.publisher_left.publish(msg_left)

        msg_right = Float64MultiArray()
        msg_right.data = [phi_dot_R]
        self.publisher_right.publish(msg_right)

        # Mensajes de depuración (debug)
        self.get_logger().debug(f'Recibido cmd_vel: lineal_x={linear_x}, angular_z={angular_z}')
        self.get_logger().debug(f'Velocidades publicadas a las ruedas: izquierda={phi_dot_L}, derecha={phi_dot_R}')

# =========================================================
#   Función principal: inicializa el nodo y mantiene el spin
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    cmd_vel_listener = CmdVelListener()
    rclpy.spin(cmd_vel_listener)
    cmd_vel_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()