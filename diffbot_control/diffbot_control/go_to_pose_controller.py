import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
from rclpy.time import Time
import numpy as np

# =========================================================
#   Nodo: GoToPoseController
#   Descripción:
#   - Se suscribe a la odometría del robot para conocer su posición actual.
#   - Se suscribe a /cmd_vel, interpretando los valores de linear.x, linear.y y angular.z
#     como un objetivo de desplazamiento (x, y) y un cambio de orientación (theta)
#     relativos a la posición actual cuando se recibe el comando.
#   - Calcula y publica los comandos de velocidad (Twist) para mover el robot
#     a la posición y orientación objetivo.
#   - Detiene el robot una vez que se alcanza la tolerancia definida.
# =========================================================

class GoToPoseController(Node):
    def __init__(self):
        super().__init__('go_to_pose_controller')

        # Declarar y obtener parámetros
        # ELIMINA LA SIGUIENTE LÍNEA:
        # self.declare_parameter('use_sim_time', True)
        self.declare_parameter('linear_tolerance', 0.1)
        self.declare_parameter('angular_tolerance', 0.1)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 0.5)

        # AHORA OBTEN EL PARÁMETRO use_sim_time DESPUÉS DE DECLARARLO EN EL LAUNCH FILE
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.linear_tolerance = self.get_parameter('linear_tolerance').get_parameter_value().double_value
        self.angular_tolerance = self.get_parameter('angular_tolerance').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        self.get_logger().info(f'Controlador GoToPose inicializado.')
        self.get_logger().info(f'Usando tiempo de simulación: {self.use_sim_time}') # Puedes agregar esto para verificar

        self.get_logger().info(f'Controlador GoToPose inicializado.')
        self.get_logger().info(f'Tolerancia lineal: {self.linear_tolerance} m')
        self.get_logger().info(f'Tolerancia angular: {self.angular_tolerance} rad')
        self.get_logger().info(f'Velocidad lineal máxima: {self.max_linear_speed} m/s')
        self.get_logger().info(f'Velocidad angular máxima: {self.max_angular_speed} rad/s')

        # Suscripción a la odometría
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Suscripción al tópico /cmd_vel para establecer el objetivo
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            1
        )

        # Publicador de comandos de velocidad al sistema de control de ruedas
        self.robot_cmd_vel_publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)

        # Variables de estado del objetivo
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        self.is_moving_to_target = False

        # Timer para ejecutar el bucle de control a una frecuencia regular
        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extraer orientación del cuaternión
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_theta = self.euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )
        self.odom_received = True

    def cmd_vel_callback(self, msg):
        if not self.odom_received:
            self.get_logger().warn('Odometría no recibida aún. No se puede establecer el objetivo.')
            return

        # Interpretar el comando Twist como un desplazamiento relativo desde la posición actual
        delta_x_rel = msg.linear.x
        delta_y_rel = msg.linear.y
        delta_theta_rel = msg.angular.z

        # Solo transformar coordenadas si hay movimiento lineal
        if abs(delta_x_rel) > 0.001 or abs(delta_y_rel) > 0.001:
            # Transformar el desplazamiento relativo a coordenadas globales
            # Esto asume que linear.x es hacia adelante y linear.y es hacia un lado en el marco del robot
            self.target_x = self.current_x + delta_x_rel * math.cos(self.current_theta) - delta_y_rel * math.sin(self.current_theta)
            self.target_y = self.current_y + delta_x_rel * math.sin(self.current_theta) + delta_y_rel * math.cos(self.current_theta)
        else:
            # Si no hay movimiento lineal, mantener la posición actual
            self.target_x = self.current_x
            self.target_y = self.current_y
            
        self.target_theta = self.current_theta + delta_theta_rel
        self.target_theta = math.atan2(math.sin(self.target_theta), math.cos(self.target_theta)) # Normalizar

        self.is_moving_to_target = True
        self.get_logger().info(f'¡Nuevo objetivo recibido! Ir a: ({self.target_x:.2f}, {self.target_y:.2f}, {self.target_theta:.2f} rad)')


    def control_loop(self):
        if not self.odom_received or not self.is_moving_to_target:
            return

        cmd_vel_msg = Twist()

        # Calcular la distancia y el ángulo al objetivo
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)

        angular_error = angle_to_target - self.current_theta
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error)) # Normalizar

        # Control PID simplificado (proporcional)
        linear_speed = 0.0
        angular_speed = 0.0

        if distance_to_target > self.linear_tolerance:
            # Primero, orientarse hacia el objetivo
            if abs(angular_error) > self.angular_tolerance:
                angular_speed = min(self.max_angular_speed, max(-self.max_angular_speed, 1.0 * angular_error))
                linear_speed = 0.0 # No avanzar hasta que la orientación sea aceptable
            else:
                # Luego, avanzar hacia el objetivo
                linear_speed = min(self.max_linear_speed, max(-self.max_linear_speed, 1.0 * distance_to_target))
                angular_speed = min(self.max_angular_speed, max(-self.max_angular_speed, 0.5 * angular_error)) # Ajuste fino de orientación
        else:
            # Si ya se alcanzó la posición, ajustar solo la orientación final
            if abs(self.target_theta - self.current_theta) > self.angular_tolerance:
                angular_speed = min(self.max_angular_speed, max(-self.max_angular_speed, 1.0 * (self.target_theta - self.current_theta)))
            else:
                # Objetivo alcanzado
                self.is_moving_to_target = False
                self.get_logger().info('¡Objetivo de pose alcanzado! Robot detenido.')

        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.angular.z = angular_speed
        
        self.robot_cmd_vel_publisher.publish(cmd_vel_msg)

        if not self.is_moving_to_target:
            # Si ya no se está moviendo al objetivo, publicar velocidad cero
            stopped_twist = Twist()
            self.robot_cmd_vel_publisher.publish(stopped_twist)


    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (roll)
        pitch is rotation around y in radians (pitch)
        yaw is rotation around z in radians (yaw)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def main(args=None):
    rclpy.init(args=args)
    go_to_pose_controller = GoToPoseController()
    rclpy.spin(go_to_pose_controller)
    go_to_pose_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()