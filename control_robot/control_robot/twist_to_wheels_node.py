import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64 # Para publicar comandos de velocidad angular individuales

class TwistToWheelsNode(Node):
    """
    Nodo ROS 2 que convierte mensajes geometry_msgs/Twist (velocidad lineal y angular)
    en comandos de velocidad angular para las ruedas izquierda y derecha de un robot diferencial.
    """

    def __init__(self):
        super().__init__('twist_to_wheels_node')

        # Declarar y obtener parámetros del robot
        # Estos parámetros deberían coincidir con los de tu URDF para una simulación precisa.
        # Puedes ajustarlos aquí o cargarlos desde un archivo YAML si lo prefieres.
        self.declare_parameter('wheel_radius', 0.05)  # Radio de la rueda en metros
        self.declare_parameter('track_width', 0.3)   # Distancia entre las ruedas (ancho de vía) en metros

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.track_width = self.get_parameter('track_width').get_parameter_value().double_value

        self.get_logger().info(f'Parámetros del robot: Radio de rueda={self.wheel_radius} m, Ancho de vía={self.track_width} m')

        # Suscriptor al topic cmd_vel
        # Recibirá mensajes de tipo Twist con la velocidad deseada para el robot.
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10 # Calidad de servicio (QoS) para el suscriptor
        )
        self.subscription  # Evita la advertencia de variable no utilizada

        # Publicadores para los comandos de velocidad de cada rueda
        # Publicarán mensajes de tipo Float64 con la velocidad angular deseada para cada rueda.
        self.left_wheel_pub = self.create_publisher(Float64, 'left_wheel_cmd', 10)
        self.right_wheel_pub = self.create_publisher(Float64, 'right_wheel_cmd', 10)

        self.get_logger().info('Nodo TwistToWheelsNode iniciado. Suscribiendo a /cmd_vel y publicando en /left_wheel_cmd, /right_wheel_cmd.')

    def twist_callback(self, msg):
        """
        Callback que se ejecuta cada vez que se recibe un mensaje Twist en /cmd_vel.
        Calcula las velocidades angulares de las ruedas y las publica.
        """
        linear_vel = msg.linear.x    # Velocidad lineal deseada (m/s)
        angular_vel = msg.angular.z  # Velocidad angular deseada (rad/s)

        # Cálculo de la cinemática inversa para un robot diferencial
        # w_R = (v + w * L / 2) / R
        # w_L = (v - w * L / 2) / R

        # Velocidad angular de la rueda derecha (rad/s)
        right_wheel_angular_vel = (linear_vel + angular_vel * (self.track_width / 2.0)) / self.wheel_radius
        # Velocidad angular de la rueda izquierda (rad/s)
        left_wheel_angular_vel = (linear_vel - angular_vel * (self.track_width / 2.0)) / self.wheel_radius

        # Crear mensajes Float64 para publicar
        left_cmd_msg = Float64()
        left_cmd_msg.data = left_wheel_angular_vel

        right_cmd_msg = Float64()
        right_cmd_msg.data = right_wheel_angular_vel

        # Publicar las velocidades calculadas
        self.left_wheel_pub.publish(left_cmd_msg)
        self.right_wheel_pub.publish(right_cmd_msg)

        # Opcional: Loguear las velocidades para depuración
        # self.get_logger().info(f'Recibido Twist: v={linear_vel:.2f}, w={angular_vel:.2f} -> Rueda Izq: {left_wheel_angular_vel:.2f} rad/s, Rueda Der: {right_wheel_angular_vel:.2f} rad/s')

def main(args=None):
    rclpy.init(args=args) # Inicializa la biblioteca rclpy
    twist_to_wheels_node = TwistToWheelsNode() # Crea una instancia de nuestro nodo
    rclpy.spin(twist_to_wheels_node) # Mantiene el nodo activo y procesando callbacks
    twist_to_wheels_node.destroy_node() # Limpia el nodo al finalizar
    rclpy.shutdown() # Apaga la biblioteca rclpy

if __name__ == '__main__':
    main()
