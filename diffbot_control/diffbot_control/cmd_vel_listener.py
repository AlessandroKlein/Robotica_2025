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
        
        # Parámetro de separación de ruedas
        self.declare_parameter('wheel_separation', 0.135)
        # Parámetro de radio de rueda
        self.declare_parameter('wheel_radius', 0.07/2)

        # Obtener los parámetros
        self.wheel_sep = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        
        # Creación de suscriptor
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.sub_callback, 10)
        
        # Crear los dos publisher a los topics de cada rueda
        self.pub_lwheel = self.create_publisher(Float64MultiArray,
            'left_wheel_velocity_controller/commands', 10)
        self.pub_rwheel = self.create_publisher(Float64MultiArray,
            'right_wheel_velocity_controller/commands', 10)
        
        self.get_logger().info('Nodo cmd_vel_listener listo. Esperando comandos en /cmd_vel...')
    
    def sub_callback(self, msg: Twist):
        """
        Callback que procesa comandos Twist y calcula velocidades angulares de ruedas.
        
        Args:
            msg (Twist): Comando de velocidad lineal y angular
        """
        # Obtengo la velocidad lineal y angular deseada
        x_dot = msg.linear.x
        w_dot = msg.angular.z
        
        # Modelo cinemático inverso
        phi_dot_lwheel = (x_dot - ((self.wheel_sep/2) * w_dot)) / self.wheel_r
        phi_dot_rwheel = (x_dot + ((self.wheel_sep/2) * w_dot)) / self.wheel_r

        # Crear los mensajes y publicar
        lwheel_msg = Float64MultiArray()
        lwheel_msg.data = [phi_dot_lwheel]
        self.pub_lwheel.publish(lwheel_msg)

        rwheel_msg = Float64MultiArray()
        rwheel_msg.data = [phi_dot_rwheel]
        self.pub_rwheel.publish(rwheel_msg)
        


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