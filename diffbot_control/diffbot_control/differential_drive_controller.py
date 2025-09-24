#!/usr/bin/env python3
"""
Controlador de Tracción Diferencial para DiffBot

Este nodo implementa un controlador que convierte comandos de velocidad Twist
en velocidades angulares específicas para las ruedas izquierda y derecha del robot.
Utiliza el modelo cinemático inverso del robot diferencial.

Autor: Sistema de Control DiffBot
Fecha: 2025
Versión: 1.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class DiffDriveController(Node):
    """
    Controlador de tracción diferencial que convierte comandos Twist a velocidades de ruedas.
    
    Este nodo se suscribe al topic /cmd_vel y publica comandos de velocidad angular
    a los controladores de las ruedas izquierda y derecha del robot diferencial.
    
    Parámetros:
        wheel_radius (float): Radio de las ruedas en metros (default: 0.035)
        wheel_separation (float): Separación entre ruedas en metros (default: 0.135)
    
    Topics:
        Suscripciones:
            /cmd_vel (geometry_msgs/Twist): Comandos de velocidad del robot
        
        Publicaciones:
            /velocity_controller_l/commands (std_msgs/Float64MultiArray): Comandos para rueda izquierda
            /velocity_controller_r/commands (std_msgs/Float64MultiArray): Comandos para rueda derecha
    """
    def __init__(self):
        super().__init__('diff_drive_controller')

        # Declarar parámetros (se pueden cargar desde robot_description vía launch)
        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('wheel_separation', 0.135)
        
        # Parámetros de calibración de odometría
        self.declare_parameter('c_L', 1.0)  # Coeficiente de corrección rueda izquierda
        self.declare_parameter('c_R', 1.0)  # Coeficiente de corrección rueda derecha

        # Leer parámetros
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.c_L = self.get_parameter('c_L').get_parameter_value().double_value
        self.c_R = self.get_parameter('c_R').get_parameter_value().double_value

        # Suscriptor a cmd_vel
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publicadores a los topics solicitados
        self.left_pub  = self.create_publisher(Float64MultiArray, '/velocity_controller_l/commands', 10)
        self.right_pub = self.create_publisher(Float64MultiArray, '/velocity_controller_r/commands', 10)

        self.get_logger().info(f"Parámetros: radio={self.wheel_radius} m, separación={self.wheel_separation} m")
        self.get_logger().info(f"Calibración: c_L={self.c_L:.6f}, c_R={self.c_R:.6f}")

    def cmd_vel_callback(self, msg):
        """
        Callback que procesa comandos de velocidad y los convierte a velocidades angulares.
        
        Implementa la cinemática inversa del robot diferencial:
        - phi_dot_R = (1/r) * (v + (L/2) * omega)
        - phi_dot_L = (1/r) * (v - (L/2) * omega)
        
        Donde:
            v: velocidad lineal del robot (m/s)
            omega: velocidad angular del robot (rad/s)
            r: radio de las ruedas (m)
            L: separación entre ruedas (m)
            phi_dot_R/L: velocidades angulares de las ruedas (rad/s)
        
        Args:
            msg (geometry_msgs.msg.Twist): Comando de velocidad recibido
        """
        # Extraer velocidades del mensaje Twist
        linear_x = msg.linear.x   # m/s
        angular_z = msg.angular.z # rad/s

        # Cinemática inversa del robot diferencial con corrección de calibración
        # phi_dot_R = (1/r) * (x_dot + (b/2) * theta_dot) * c_R
        # phi_dot_L = (1/r) * (x_dot - (b/2) * theta_dot) * c_L
        phi_dot_right = (1.0 / self.wheel_radius) * (linear_x + (self.wheel_separation / 2.0) * angular_z) * self.c_R
        phi_dot_left  = (1.0 / self.wheel_radius) * (linear_x - (self.wheel_separation / 2.0) * angular_z) * self.c_L

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
    """
    Función principal del nodo controlador de tracción diferencial.
    
    Inicializa el nodo ROS2, crea una instancia del controlador y mantiene
    el nodo activo hasta recibir una señal de interrupción.
    
    Args:
        args: Argumentos de línea de comandos (opcional)
    """
    # Inicialización del sistema ROS2
    rclpy.init(args=args)

    # Creación del nodo controlador
    nodo = DiffDriveController()

    try:
        # Mantener el nodo activo procesando callbacks
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        # Manejo de interrupción por teclado (Ctrl+C)
        pass
    finally:
        # Limpieza y finalización
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()