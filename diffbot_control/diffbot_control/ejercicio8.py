#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')

        # Declarar parámetros (se pueden cargar desde robot_description vía launch)
        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('wheel_separation', 0.135)

        # Leer parámetros
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value

        # Suscriptor a cmd_vel
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publicadores a los topics solicitados
        self.left_pub  = self.create_publisher(Float64MultiArray, '/velocity_controller_l/commands', 10)
        self.right_pub = self.create_publisher(Float64MultiArray, '/velocity_controller_r/commands', 10)

        self.get_logger().info(f"Parámetros: radio={self.wheel_radius} m, separación={self.wheel_separation} m")

    def cmd_vel_callback(self, msg):
        """
        Convierte comandos de velocidad (Twist) a velocidades angulares de ruedas.
        """
        # Extraer velocidades del mensaje Twist
        linear_x = msg.linear.x   # m/s
        angular_z = msg.angular.z # rad/s

        # Cinemática inversa del robot diferencial
        # phi_dot_R = (1/r) * (x_dot + (b/2) * theta_dot)
        # phi_dot_L = (1/r) * (x_dot - (b/2) * theta_dot)
        phi_dot_right = (1.0 / self.wheel_radius) * (linear_x + (self.wheel_separation / 2.0) * angular_z)
        phi_dot_left  = (1.0 / self.wheel_radius) * (linear_x - (self.wheel_separation / 2.0) * angular_z)

        # Crear mensajes Float64MultiArray
        left_msg = Float64MultiArray()
        left_msg.data = [phi_dot_left]

        right_msg = Float64MultiArray()
        right_msg.data = [phi_dot_right]

        # Publicar comandos
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        # Log para debug
        self.get_logger().debug(f"cmd_vel: linear={linear_x:.3f}, angular={angular_z:.3f}")
        self.get_logger().debug(f"wheel_speeds: left={phi_dot_left:.3f}, right={phi_dot_right:.3f}")

def main(args=None):
    # 1. Inicialización
    rclpy.init(args=args)

    # 2. Creación de nodo
    nodo = DiffDriveController()

    try:
        # 3. Procesamiento de mensajes y callback
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        # 4. Finalización
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()