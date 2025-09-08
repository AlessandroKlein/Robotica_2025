#!/usr/bin/env python3
"""
Ejercicio 8: Nodo que recibe comandos geometry_msgs/Twist via topic cmd_vel,
calcula velocidades angulares usando cinemática inversa y publica a 
left_wheel_cmd y right_wheel_cmd topics.

Autor: Generado para TP-1
Fecha: 2025
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class CmdVelListener(Node):
    """
    Nodo que convierte comandos Twist a velocidades angulares de ruedas
    usando el modelo cinemático inverso del robot diferencial.
    """
    
    def __init__(self):
        super().__init__('cmd_vel_listener')
        
        # Declarar parámetros del robot (obtenidos del URDF)
        self.declare_parameter('wheel_radius', 0.035)  # Radio de rueda en metros
        self.declare_parameter('wheel_separation', 0.100)  # Separación entre ruedas en metros
        self.declare_parameter('invert_polarity', False)  # Invertir polaridad de motores
        self.declare_parameter('swap_wheels', False)  # Intercambiar ruedas izquierda/derecha
        
        # Obtener valores de parámetros
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.invert_polarity = self.get_parameter('invert_polarity').get_parameter_value().bool_value
        self.swap_wheels = self.get_parameter('swap_wheels').get_parameter_value().bool_value
        
        # Log de inicialización
        self.get_logger().info(f'CmdVelListener inicializado:')
        self.get_logger().info(f'  - Radio de rueda: {self.wheel_radius} m')
        self.get_logger().info(f'  - Separación de ruedas: {self.wheel_separation} m')
        self.get_logger().info(f'  - Invertir polaridad: {self.invert_polarity}')
        self.get_logger().info(f'  - Intercambiar ruedas: {self.swap_wheels}')
        
        # Suscriptor al topic cmd_vel (según consigna)
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers a los topics de comandos de ruedas (según consigna)
        self.left_wheel_pub = self.create_publisher(
            Float64MultiArray,
            'left_wheel_cmd',
            10
        )
        
        self.right_wheel_pub = self.create_publisher(
            Float64MultiArray,
            'right_wheel_cmd',
            10
        )
        
        self.get_logger().info('Nodo cmd_vel_listener listo. Esperando comandos en /cmd_vel...')
    
    def cmd_vel_callback(self, msg):
        """
        Callback que procesa comandos Twist y calcula velocidades angulares de ruedas.
        
        Args:
            msg (Twist): Comando de velocidad lineal y angular
        """
        # Extraer velocidades del mensaje Twist
        linear_x = msg.linear.x  # Velocidad lineal hacia adelante (m/s)
        angular_z = msg.angular.z  # Velocidad angular (rad/s)
        
        # Cinemática inversa para robot diferencial
        # v_left = (2*v - w*L) / (2*R)
        # v_right = (2*v + w*L) / (2*R)
        # donde: v = velocidad lineal, w = velocidad angular, L = separación, R = radio
        
        omega_left = (2.0 * linear_x - angular_z * self.wheel_separation) / (2.0 * self.wheel_radius)
        omega_right = (2.0 * linear_x + angular_z * self.wheel_separation) / (2.0 * self.wheel_radius)
        
        # Aplicar configuraciones de parámetros
        if self.invert_polarity:
            omega_left = -omega_left
            omega_right = -omega_right
            
        if self.swap_wheels:
            omega_left, omega_right = omega_right, omega_left
        
        # Crear mensajes Float64MultiArray
        left_msg = Float64MultiArray()
        left_msg.data = [omega_left]
        
        right_msg = Float64MultiArray()
        right_msg.data = [omega_right]
        
        # Publicar comandos
        self.left_wheel_pub.publish(left_msg)
        self.right_wheel_pub.publish(right_msg)
        
        # Log de debug
        self.get_logger().debug(
            f'Cmd: v={linear_x:.3f} m/s, w={angular_z:.3f} rad/s -> '
            f'ωL={omega_left:.3f} rad/s, ωR={omega_right:.3f} rad/s'
        )

def main(args=None):
    """
    Función principal del nodo.
    
    Args:
        args: Argumentos de línea de comandos
    """
    # Inicializar ROS 2
    rclpy.init(args=args)
    
    # Crear instancia del nodo
    node = CmdVelListener()
    
    try:
        # Mantener el nodo ejecutándose
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo interrumpido por el usuario')
    finally:
        # Limpiar recursos
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()