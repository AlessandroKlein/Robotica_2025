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

class DiffbotControl(Node):
    """
    Nodo que convierte comandos Twist a velocidades angulares de ruedas
    usando el modelo cinemático inverso del robot diferencial.
    """
    
    def __init__(self):
        super().__init__('diffbot_control_node')
        
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
    
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x      # m/s
        omega = msg.angular.z # rad/s

        # Modelo cinemático inverso → velocidades angulares de ruedas (rad/s)
        w_r = (v + (omega * self.wheel_separation / 2.0)) / self.wheel_radius
        w_l = (v - (omega * self.wheel_separation / 2.0)) / self.wheel_radius

        # Publicar
        left_msg = Float64MultiArray()
        right_msg = Float64MultiArray()
        left_msg.data = [w_l]
        right_msg.data = [w_r]

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        self.get_logger().info(f"cmd_vel: v={v:.2f} m/s, ω={omega:.2f} rad/s → wl={w_l:.2f} rad/s, wr={w_r:.2f} rad/s")
        


def main(args=None):
    # 1. Inicialización
    rclpy.init(args=args)
    # 2. Creación de nodo
    nodo = DiffbotControl()
    try:
        # 3. Procesamiento de mensajes y callback
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        # 4. Finalización
        rclpy.shutdown()

if __name__ == '__main__':
    main()