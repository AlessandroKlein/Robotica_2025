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
        # Cada vez que llega un mensaje Twist a /cmd_vel, se llama a self.cmd_vel_callback
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10 # Calidad de servicio (QoS): tamaño del búfer de mensajes
        )
        self.subscription  # Evita advertencia de variable no usada

        # Publicadores para los controladores de velocidad de cada rueda
        # Estos son los tópicos donde se enviarán las velocidades calculadas
        self.publisher_left = self.create_publisher(Float64MultiArray, '/velocity_controller_left/commands', 10)
        self.publisher_right = self.create_publisher(Float64MultiArray, '/velocity_controller_right/commands', 10)

        # Declarar y obtener parámetros para el radio y la separación de las ruedas
        # Estos valores se pueden configurar al lanzar el nodo, si no, usan los por defecto.
        self.declare_parameter('wheel_radius', 0.035) # Valor por defecto del radio de la rueda (metros)
        self.declare_parameter('wheel_separation', 0.135) # Valor por defecto de la separación entre ruedas (metros)

        # Asignar los valores de los parámetros a las variables de instancia
        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.b = self.get_parameter('wheel_separation').get_parameter_value().double_value

        # Mensaje informativo al iniciar el nodo
        self.get_logger().info(f'CmdVelListener inicializado con radio de rueda: {self.r} y separación entre ruedas: {self.b}')

    def cmd_vel_callback(self, msg):
        """
        Esta función se llama automáticamente cada vez que se recibe un mensaje
        en el tópico /cmd_vel.
        """
        # =====================================================================
        # APARTADO DE RECEPCIÓN Y SEPARACIÓN DE LOS VALORES X, Y, Z DEL COMANDO
        # =====================================================================
        # Extraemos las componentes de velocidad del mensaje Twist.
        # Para un robot diferencial, solo son relevantes las velocidades linear.x y angular.z.

        # Velocidad lineal en el eje X (avance/retroceso del robot)
        linear_x = msg.linear.x

        # Velocidad lineal en el eje Y (movimiento lateral, **no usada** en cinemática diferencial)
        # Aunque se recibe, no se utiliza directamente en los cálculos de las ruedas
        # para un robot diferencial estándar.
        linear_y = msg.linear.y # Se captura, pero no se usa en la fórmula actual.

        # Velocidad angular en el eje Z (giro sobre el propio eje del robot)
        angular_z = msg.angular.z

        # Para la cinemática inversa, las velocidades que usaremos son las recibidas directamente:
        current_linear_x = linear_x
        current_angular_z = angular_z
        # =====================================================================

        # Cinemática inversa para robot diferencial:
        # Calcula las velocidades angulares de las ruedas (phi_dot)
        # Estas fórmulas son válidas para cualquier combinación de linear_x y angular_z,
        # incluyendo el caso donde linear_x es 0.0.

        # Fórmulas de cinemática inversa:
        # phi_dot_R = (vx + (b/2)*wz) / r
        # phi_dot_L = (vx - (b/2)*wz) / r
        # Donde:
        #   vx: velocidad lineal en X (current_linear_x)
        #   wz: velocidad angular en Z (current_angular_z)
        #   b: separación entre ruedas (self.b)
        #   r: radio de rueda (self.r)
        
        # Estos cálculos deben estar fuera de cualquier condicional
        # para asegurar que phi_dot_L y phi_dot_R siempre tengan un valor.
        if current_linear_x != 0:
            
            phi_dot_R = (1 / self.r) * (current_linear_x + (self.b / 2) * current_angular_z)
            phi_dot_L = (1 / self.r) * (current_linear_x - (self.b / 2) * current_angular_z)

            # Si habías añadido 'phi_dot_R = -phi_dot_R' por alguna razón de dirección,
            # asegúrate de que sea realmente necesario para tu setup físico.
            # En la mayoría de los casos, la cinemática inversa estándar no requiere esto.
            # Si tu robot gira al revés, probablemente es un problema de configuración de joint_states
            # o de cómo los controladores esperan las velocidades.
            phi_dot_R = -phi_dot_R # Descomentar solo si es estrictamente necesario para tu robot físico.

        else:
            phi_dot_R = (1 / self.r) * (current_linear_x + (self.b / 2) * current_angular_z)
            phi_dot_L = (1 / self.r) * (current_linear_x - (self.b / 2) * current_angular_z)
            phi_dot_R = -phi_dot_R
            phi_dot_L = -phi_dot_L

        # Crear mensajes Float64MultiArray para cada rueda
        # Estos mensajes son los que se publican a los controladores de velocidad.
        msg_left = Float64MultiArray()
        msg_left.data = [phi_dot_L] # El mensaje Float64MultiArray espera una lista de flotantes
        self.publisher_left.publish(msg_left)

        msg_right = Float64MultiArray()
        msg_right.data = [phi_dot_R] # El mensaje Float64MultiArray espera una lista de flotantes
        self.publisher_right.publish(msg_right)

        # Mensajes de depuración (debug) para monitorear las velocidades
        self.get_logger().debug(f'Recibido cmd_vel: lineal_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}')
        self.get_logger().debug(f'Aplicando velocidades para cinemática: lineal_x={current_linear_x}, angular_z={current_angular_z}')
        self.get_logger().debug(f'Velocidades publicadas a las ruedas: izquierda={phi_dot_L}, derecha={phi_dot_R}')

# =========================================================
#   Función principal: inicializa el nodo y mantiene el spin
# =========================================================
def main(args=None):
    rclpy.init(args=args) # Inicializa la comunicación con ROS 2
    cmd_vel_listener = CmdVelListener() # Crea una instancia de nuestro nodo
    rclpy.spin(cmd_vel_listener) # Mantiene el nodo activo y escuchando mensajes indefinidamente
    
    # Cuando el nodo se detiene (ej. con Ctrl+C), se ejecutan estas líneas
    cmd_vel_listener.destroy_node() # Libera los recursos del nodo
    rclpy.shutdown() # Cierra el contexto de ROS 2

if __name__ == '__main__':
    main()