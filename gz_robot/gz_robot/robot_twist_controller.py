import rclpy # Importa la biblioteca cliente de ROS 2 para Python
from rclpy.node import Node # Importa la clase base para crear nodos en ROS 2
from geometry_msgs.msg import Twist # Importa el tipo de mensaje Twist para los comandos de velocidad
from std_msgs.msg import Float64 # Importa el tipo de mensaje Float64 para publicar velocidades de rueda
import math # Importa el módulo math para operaciones matemáticas como pi

class RobotTwistController(Node):
    def __init__(self):
        super().__init__('robot_twist_controller') # Llama al constructor de la clase base Node y nombra el nodo
        
        # Declaración y obtención de parámetros del robot
        # Estos parámetros se suelen definir en un archivo YAML o se cargan desde el description del robot
        self.declare_parameter('wheel_radius', 0.05) # Declara el parámetro 'wheel_radius' con un valor por defecto
        self.declare_parameter('wheel_separation', 0.2) # Declara el parámetro 'wheel_separation' con un valor por defecto
        
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value # Obtiene el valor del radio de la rueda
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value # Obtiene el valor de la separación entre ruedas
        
        self.get_logger().info(f"Radio de rueda: {self.wheel_radius}") # Muestra un mensaje informativo con el radio de la rueda
        self.get_logger().info(f"Separación entre ruedas: {self.wheel_separation}") # Muestra un mensaje informativo con la separación entre ruedas

        # Suscriptor al topic de comandos de velocidad (cmd_vel)
        self.subscription = self.create_subscription(
            Twist, # Tipo de mensaje a suscribirse
            'cmd_vel', # Nombre del topic
            self.cmd_vel_callback, # Función que se llamará cada vez que llegue un mensaje
            10 # Profundidad de la cola (cantidad de mensajes a almacenar si no se procesan a tiempo)
        )
        self.subscription # Evita una advertencia de variable no utilizada

        # Publicadores de velocidad para cada rueda
        self.left_wheel_pub = self.create_publisher(Float64, 'left_wheel_cmd', 10) # Crea un publicador para la rueda izquierda
        self.right_wheel_pub = self.create_publisher(Float64, 'right_wheel_cmd', 10) # Crea un publicador para la rueda derecha
        
        self.get_logger().info('Nodo de control de Twist iniciado y esperando comandos en /cmd_vel...') # Mensaje de inicio del nodo

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x # Obtiene la velocidad lineal en el eje X del mensaje Twist
        angular_z = msg.angular.z # Obtiene la velocidad angular en el eje Z del mensaje Twist
        
        self.get_logger().info(f'Recibido: lineal_x={linear_x:.2f}, angular_z={angular_z:.2f}') # Muestra el comando recibido

        # Cálculo de la cinemática inversa
        # v_d = (2*v + w*L) / (2*R) -> Velocidad angular de la rueda derecha
        # v_i = (2*v - w*L) / (2*R) -> Velocidad angular de la rueda izquierda
        
        # Donde:
        # v = velocidad lineal (linear_x)
        # w = velocidad angular (angular_z)
        # L = separación entre ruedas (wheel_separation)
        # R = radio de la rueda (wheel_radius)

        if self.wheel_radius == 0: # Evita división por cero
            self.get_logger().error("El radio de la rueda no puede ser cero. Por favor, configure el parámetro 'wheel_radius'.")
            return

        # Calcula la velocidad angular de la rueda derecha (en rad/s)
        right_wheel_velocity = (linear_x + angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        # Calcula la velocidad angular de la rueda izquierda (en rad/s)
        left_wheel_velocity = (linear_x - angular_z * self.wheel_separation / 2.0) / self.wheel_radius

        # Crea los mensajes Float64 para las velocidades de las ruedas
        left_wheel_cmd_msg = Float64() # Instancia un mensaje Float64
        left_wheel_cmd_msg.data = left_wheel_velocity # Asigna la velocidad calculada a la propiedad 'data'
        
        right_wheel_cmd_msg = Float64() # Instancia un mensaje Float64
        right_wheel_cmd_msg.data = right_wheel_velocity # Asigna la velocidad calculada a la propiedad 'data'

        # Publica las velocidades de las ruedas
        self.left_wheel_pub.publish(left_wheel_cmd_msg) # Publica el mensaje para la rueda izquierda
        self.right_wheel_pub.publish(right_wheel_cmd_msg) # Publica el mensaje para la rueda derecha
        
        self.get_logger().info(f'Publicado: Rueda Izquierda={left_wheel_velocity:.2f} rad/s, Rueda Derecha={right_wheel_velocity:.2f} rad/s') # Muestra las velocidades publicadas


def main(args=None):
    rclpy.init(args=args) # Inicializa la comunicación con ROS 2
    node = RobotTwistController() # Crea una instancia del nodo
    rclpy.spin(node) # Mantiene el nodo en ejecución, esperando callbacks
    node.destroy_node() # Destruye el nodo limpiamente al finalizar
    rclpy.shutdown() # Cierra la comunicación con ROS 2

if __name__ == '__main__':
    main() # Punto de entrada principal para ejecutar el script