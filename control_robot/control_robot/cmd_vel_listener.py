import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')
        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('wheel_separation', 0.135)

        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.b = self.get_parameter('wheel_separation').get_parameter_value().double_value

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publicadores para los comandos de velocidad de las ruedas izquierda y derecha
        self.publisher_left = self.create_publisher(
            Float64MultiArray, '/velocity_controller_left/commands', 10)
        self.publisher_right = self.create_publisher(
            Float64MultiArray, '/velocity_controller_right/commands', 10)

        self.get_logger().info(
            f"Inicializado con r={self.r:.3f} m y b={self.b:.3f} m")

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x    # Velocidad lineal deseada (m/s)
        angular_z = msg.angular.z  # Velocidad angular deseada (rad/s)

        # Cinemática diferencial inversa
        # Calculamos las velocidades lineales tangenciales de cada rueda.
        # Aquí, asumimos que 'v_l' es para la rueda izquierda y 'v_r' para la rueda derecha.
        v_l = linear_x - (angular_z * self.b / 2.0)
        v_r = linear_x + (angular_z * self.b / 2.0)

        # Convertimos las velocidades lineales tangenciales a velocidades angulares
        # de las ruedas (rad/s) dividiendo por el radio de la rueda.
        phi_l = v_l / self.r
        phi_r = v_r / self.r

        # --- CORRECCIÓN FINAL DE DIRECCIÓN ---
        # Si el robot se mueve en sentido contrario al deseado para el avance/retroceso,
        # pero los giros son correctos (relativamente entre ruedas),
        # simplemente invertimos el signo de AMBAS velocidades angulares de las ruedas.
        phi_l = -phi_l
        phi_r = -phi_r

        # --- AJUSTES PARA DIRECCIÓN Y GIRO ---

        # Creamos los mensajes Float64MultiArray ANTES de asignarles los datos.
        msg_right = Float64MultiArray()
        msg_left = Float64MultiArray()

        # Opciones de ajuste: (Prueba UNA a la vez y observa el comportamiento)
        # -------------------------------------------------------------------

        # OPCIÓN A: Si el robot va hacia atrás cuando debería ir hacia adelante,
        #           y los giros también están invertidos (izquierda es derecha, derecha es izquierda).
        #           Esto implica que ambos motores están cableados al revés, o que el "frente" de tu modelo
        #           es opuesto al eje X de ROS.
        # msg_right.data = [-phi_r]
        # msg_left.data = [-phi_l]

        # OPCIÓN B: Si el robot avanza correctamente, pero al girar las ruedas giran en sentidos opuestos
        #           o la dirección de giro está invertida (por ejemplo, cmd_vel.angular.z positivo lo hace girar a la derecha).
        #           Esto sugiere que los comandos de los motores izquierdo y derecho están intercambiados
        #           respecto a la definición de las ruedas en tu modelo de Gazebo.
        #           Esta es la corrección más común cuando el avance es correcto pero los giros no.
        # msg_right.data = [phi_l] # Envía la velocidad calculada de la rueda IZQUIERDA al controlador DERECHO
        # msg_left.data = [phi_r]  # Envía la velocidad calculada de la rueda DERECHA al controlador IZQUIERDO
        
        # OPCIÓN C: Si el robot avanza y gira correctamente, pero una rueda específica gira en sentido opuesto
        #           *solo durante un giro* (por ejemplo, la rueda izquierda gira hacia atrás cuando debería ir hacia adelante en un giro suave).
        #           Esto es menos común si todo el giro está mal, pero puede pasar si
        #           el controlador o motor de una rueda está invertido respecto a la otra.
        #           Prueba invertir solo una, por ejemplo:
        msg_right.data = [phi_r]
        msg_left.data = [-phi_l] # Invierte solo el comando de la rueda izquierda
        # O
        # msg_right.data = [-phi_r] # Invierte solo el comando de la rueda derecha
        # msg_left.data = [phi_l]

        # Original/Por defecto (si no hay problemas en las pruebas anteriores):
        # msg_right.data = [phi_r]
        # msg_left.data = [phi_l]
        
        # -------------------------------------------------------------------

        self.publisher_right.publish(msg_right)
        self.publisher_left.publish(msg_left)

        self.get_logger().debug(
            f"cmd_vel: x={linear_x:.2f}, z={angular_z:.2f} → φL_calc={phi_l:.2f}, φR_calc={phi_r:.2f} (Enviado L:{msg_left.data[0]:.2f}, R:{msg_right.data[0]:.2f})")

# =========================================================
#   Función principal: inicializa el nodo y mantiene el spin
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()