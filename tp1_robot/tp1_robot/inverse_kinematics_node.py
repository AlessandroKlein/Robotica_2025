#!/usr/bin/env python3
# tp1_robot/inverse_kinematics_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        
        # Parámetros configurables
        self.declare_parameter('wheel_radius', 0.035)  # m (de diffbot.xacro)
        self.declare_parameter('wheel_separation', 0.23)  # m (de diffbot.xacro)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        
        # Validar parámetros
        if self.wheel_radius <= 0 or self.wheel_separation <= 0:
            self.get_logger().error('Parámetros inválidos: radio o separación de ruedas <= 0')
            raise ValueError('Parámetros de ruedas deben ser positivos')
        
        # Suscripción a /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publicadores para controladores de velocidad
        self.left_cmd_pub = self.create_publisher(Float64MultiArray, '/left_velocity_controller/commands', 10)
        self.right_cmd_pub = self.create_publisher(Float64MultiArray, '/right_velocity_controller/commands', 10)
        
        self.get_logger().info('Nodo de cinemática inversa iniciado')
        self.get_logger().info(f'Radio de rueda: {self.wheel_radius} m, Separación: {self.wheel_separation} m')

    def cmd_vel_callback(self, msg):
        # Extraer velocidades lineales y angulares
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Calcular velocidades angulares de las ruedas (rad/s)
        try:
            left_vel = (linear_x - (angular_z * self.wheel_separation / 2)) / self.wheel_radius
            right_vel = (linear_x + (angular_z * self.wheel_separation / 2)) / self.wheel_radius
            
            # Publicar en controladores
            left_msg = Float64MultiArray()
            right_msg = Float64MultiArray()
            
            left_msg.data = [left_vel]
            right_msg.data = [right_vel]
            
            self.left_cmd_pub.publish(left_msg)
            self.right_cmd_pub.publish(right_msg)
            
        except ZeroDivisionError:
            self.get_logger().error('División por cero en cálculo de cinemática')
        except Exception as e:
            self.get_logger().error(f'Error en cálculo de cinemática: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    ik_node = InverseKinematics()
    
    try:
        rclpy.spin(ik_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Detener el robot al salir
        zero_msg = Float64MultiArray(data=[0.0])
        ik_node.left_cmd_pub.publish(zero_msg)
        ik_node.right_cmd_pub.publish(zero_msg)
        ik_node.get_logger().info('Robot detenido')
        ik_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()