import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdometryNode(Node):
    """
    Nodo ROS 2 que calcula la odometría de un robot diferencial
    basado en las lecturas de posición de las juntas de las ruedas.
    """

    def __init__(self):
        super().__init__('odometry_node')

        # 1. Declarar y obtener parámetros del robot
        # Estos parámetros son esenciales para el modelo cinemático directo.
        # Asegúrate de que coincidan con las dimensiones físicas de tu robot.
        self.declare_parameter('wheel_radius', 0.05)  # Radio de la rueda en metros
        self.declare_parameter('track_width', 0.3)   # Distancia entre las ruedas (ancho de vía) en metros
        self.declare_parameter('left_wheel_joint_name', 'left_wheel_joint')  # Nombre de la junta de la rueda izquierda
        self.declare_parameter('right_wheel_joint_name', 'right_wheel_joint') # Nombre de la junta de la rueda derecha
        self.declare_parameter('odom_frame_id', 'odom') # Frame ID para la odometría
        self.declare_parameter('base_link_frame_id', 'base_link') # Frame ID para el base_link del robot

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.track_width = self.get_parameter('track_width').get_parameter_value().double_value
        self.left_wheel_joint_name = self.get_parameter('left_wheel_joint_name').get_parameter_value().string_value
        self.right_wheel_joint_name = self.get_parameter('right_wheel_joint_name').get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_link_frame_id = self.get_parameter('base_link_frame_id').get_parameter_value().string_value


        self.get_logger().info(f'Parámetros de Odometría: Radio={self.wheel_radius}m, Ancho={self.track_width}m, Izq={self.left_wheel_joint_name}, Der={self.right_wheel_joint_name}')

        # 2. Inicializar el estado de la odometría
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_left_wheel_pos = None
        self.last_right_wheel_pos = None

        # 3. Suscriptor al topic /joint_states
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10 # Calidad de servicio (QoS)
        )
        self.joint_state_sub # Evita la advertencia de variable no utilizada

        # 4. Publicador para el topic /odom
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # 5. Publicador de transformaciones TF
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Nodo OdometryNode iniciado. Suscribiendo a /joint_states y publicando en /odom y TF.')

    def joint_state_callback(self, msg):
        """
        Callback que se ejecuta cada vez que se recibe un mensaje JointState.
        Calcula la odometría y la publica.
        """
        current_time = self.get_clock().now()

        # Encontrar las posiciones de las ruedas en el mensaje JointState
        try:
            left_wheel_index = msg.name.index(self.left_wheel_joint_name)
            right_wheel_index = msg.name.index(self.right_wheel_joint_name)
            current_left_wheel_pos = msg.position[left_wheel_index]
            current_right_wheel_pos = msg.position[right_wheel_index]
        except ValueError:
            self.get_logger().warn(f"No se encontraron las juntas '{self.left_wheel_joint_name}' o '{self.right_wheel_joint_name}' en el mensaje JointState.")
            return

        # Inicializar posiciones en la primera lectura
        if self.last_left_wheel_pos is None or self.last_right_wheel_pos is None:
            self.last_left_wheel_pos = current_left_wheel_pos
            self.last_right_wheel_pos = current_right_wheel_pos
            self.last_time = current_time
            return

        # Calcular el tiempo transcurrido
        dt = (current_time - self.last_time).nanoseconds / 1e9 # Convertir nanosegundos a segundos
        if dt == 0: # Evitar división por cero si el tiempo no ha avanzado
            return

        # Calcular el cambio de posición angular de cada rueda
        delta_left_wheel_pos = current_left_wheel_pos - self.last_left_wheel_pos
        delta_right_wheel_pos = current_right_wheel_pos - self.last_right_wheel_pos

        # Calcular el desplazamiento lineal de cada rueda
        delta_s_left = delta_left_wheel_pos * self.wheel_radius
        delta_s_right = delta_right_wheel_pos * self.wheel_radius

        # Calcular el desplazamiento lineal y angular del robot (modelo cinemático directo)
        delta_s_center = (delta_s_left + delta_s_right) / 2.0
        delta_theta = (delta_s_right - delta_s_left) / self.track_width

        # Actualizar la pose del robot (x, y, theta)
        # Usamos la aproximación del punto medio para una mejor precisión
        self.x += delta_s_center * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta = (self.theta + delta_theta + math.pi) % (2 * math.pi) - math.pi # Normalizar theta a [-pi, pi]

        # Calcular velocidades para el mensaje Odometry
        vx = delta_s_center / dt
        wz = delta_theta / dt

        # 6. Publicar el mensaje Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_link_frame_id

        # Posición
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0 # Robot 2D, z=0

        # Orientación (convertir yaw a cuaternión)
        quaternion = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        # Velocidad
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = wz

        self.odom_pub.publish(odom_msg)

        # 7. Publicar la transformación TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_link_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(t)

        # Actualizar las últimas posiciones y tiempo
        self.last_left_wheel_pos = current_left_wheel_pos
        self.last_right_wheel_pos = current_right_wheel_pos
        self.last_time = current_time

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convierte ángulos de Euler (roll, pitch, yaw) a un cuaternión.
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
