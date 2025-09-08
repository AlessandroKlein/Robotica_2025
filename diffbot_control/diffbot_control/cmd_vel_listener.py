import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray # Para publicar las velocidades de las ruedas

# =========================================================
#   Nodo: CmdVelListener
#   Descripción:
#   - Se suscribe al tópico /cmd_vel_unstamped para comandos de velocidad (Twist).
#   - Convierte los comandos de velocidad lineal (x) y angular (z)
#     en velocidades individuales para las ruedas izquierda y derecha
#     usando el modelo cinemático inverso de un robot diferencial.
#   - Publica estas velocidades a los controladores de las ruedas.
# =========================================================

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')

        # Declarar y obtener parámetros para dimensiones del robot
        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('wheel_separation', 0.135)
        # ELIMINA LA SIGUIENTE LÍNEA:
        # self.declare_parameter('use_sim_time', True)

        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.b = self.get_parameter('wheel_separation').get_parameter_value().double_value

        # AHORA OBTEN EL PARÁMETRO use_sim_time DESPUÉS DE DECLARARLO EN EL LAUNCH FILE
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f'CmdVelListener inicializado con r={self.r}, b={self.b}, use_sim_time={self.use_sim_time}')


        # Suscripción al tópico de comandos de velocidad.
        # Es importante que el nombre del tópico sea consistente con lo que publican
        # otros nodos (como el nuevo go_to_pose_controller)
        self.subscription = self.create_subscription(
            Twist,
            '/diff_drive_controller/cmd_vel_unstamped', # Tópico que espera el controlador de Gazebo
            self.cmd_vel_callback,
            10
        )
        self.subscription # Evita advertencia de variable no usada

        # Publicadores para las velocidades de las ruedas individuales
        # Estos son los tópicos que el ros2_control_boilerplate espera
        self.left_wheel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller_left/commands', 10)
        self.right_wheel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller_right/commands', 10)

    def cmd_vel_callback(self, msg):
        """
        Callback que se ejecuta cuando se recibe un mensaje Twist en /cmd_vel_unstamped.
        Convierte la velocidad lineal y angular en velocidades de ruedas individuales.
        """
        # Extraer velocidades del mensaje Twist
        v = msg.linear.x  # Velocidad lineal hacia adelante (m/s)
        w = msg.angular.z # Velocidad angular (rad/s)

        # Modelo cinemático inverso para robot diferencial:
        # v_left = (v - w * b/2) / r
        # v_right = (v + w * b/2) / r
        # donde:
        #   v = velocidad lineal del robot
        #   w = velocidad angular del robot
        #   b = separación entre ruedas
        #   r = radio de las ruedas

        v_left = (v - w * self.b / 2.0) / self.r
        v_right = (v + w * self.b / 2.0) / self.r

        # Crear mensajes Float64MultiArray para cada rueda
        left_msg = Float64MultiArray()
        left_msg.data = [v_left]

        right_msg = Float64MultiArray()
        right_msg.data = [v_right]

        # Publicar las velocidades
        self.left_wheel_pub.publish(left_msg)
        self.right_wheel_pub.publish(right_msg)

        # Log para depuración (opcional, puedes comentar si es demasiado verboso)
        self.get_logger().debug(
            f'Cmd recibido: v={v:.3f} m/s, w={w:.3f} rad/s -> '
            f'v_left={v_left:.3f} rad/s, v_right={v_right:.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_listener = CmdVelListener()
    rclpy.spin(cmd_vel_listener)
    cmd_vel_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()