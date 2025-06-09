#!/usr/bin/env python3
# tp1_robot/automatic_mover_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class AutomaticMover(Node):
    def __init__(self):
        super().__init__('automatic_mover_node')
        
        # Parámetros de configuración
        self.declare_parameter('linear_speed', 0.2)  # m/s
        self.declare_parameter('angular_speed', 0.5)  # rad/s
        self.declare_parameter('movement_pattern', 'square')  # square, circle, figure8
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.pattern = self.get_parameter('movement_pattern').value
        
        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.left_cmd_pub = self.create_publisher(Float64MultiArray, '/left_velocity_controller/commands', 10)
        self.right_cmd_pub = self.create_publisher(Float64MultiArray, '/right_velocity_controller/commands', 10)
        
        # Timer para actualizar el movimiento
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Estado para patrones de movimiento
        self.state = {
            'square': {'step': 0, 'start_time': self.get_clock().now()},
            'circle': {'angle': 0.0},
            'figure8': {'angle': 0.0, 'direction': 1}
        }
        
        self.get_logger().info(f'Automatic mover iniciado con patrón: {self.pattern}')

    def timer_callback(self):
        twist = Twist()
        now = self.get_clock().now()
        
        if self.pattern == 'square':
            # Movimiento en cuadrado (avanzar 2s, girar 3s)
            elapsed = (now - self.state['square']['start_time']).nanoseconds / 1e9
            
            if elapsed < 2.0:
                twist.linear.x = self.linear_speed
            elif elapsed < 5.0:
                twist.angular.z = self.angular_speed
            else:
                self.state['square']['start_time'] = now
        
        elif self.pattern == 'circle':
            # Movimiento en círculo
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed * math.sin(self.state['circle']['angle'])
            self.state['circle']['angle'] += 0.1
        
        elif self.pattern == 'figure8':
            # Movimiento en figura 8
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed * math.sin(self.state['figure8']['angle']) * self.state['figure8']['direction']
            self.state['figure8']['angle'] += 0.1
            if self.state['figure8']['angle'] > 2 * math.pi:
                self.state['figure8']['angle'] = 0.0
                self.state['figure8']['direction'] *= -1
        
        # Publicar comando en /cmd_vel
        self.cmd_vel_pub.publish(twist)
        
        # Convertir a comandos de ruedas
        left_cmd = Float64MultiArray()
        right_cmd = Float64MultiArray()
        
        # Radio de rueda y separación del robot (de diffbot.xacro)
        wheel_radius = 0.035  # m
        wheel_separation = 0.23  # m
        
        # Calcular velocidades de ruedas (cinemática directa)
        left_vel = (twist.linear.x - (twist.angular.z * wheel_separation / 2)) / wheel_radius
        right_vel = (twist.linear.x + (twist.angular.z * wheel_separation / 2)) / wheel_radius
        
        left_cmd.data = [left_vel]
        right_cmd.data = [right_vel]
        
        self.left_cmd_pub.publish(left_cmd)
        self.right_cmd_pub.publish(right_cmd)

def main(args=None):
    rclpy.init(args=args)
    automatic_mover = AutomaticMover()
    
    try:
        rclpy.spin(automatic_mover)
    except KeyboardInterrupt:
        pass
    finally:
        # Detener el robot al salir
        twist = Twist()
        automatic_mover.cmd_vel_pub.publish(twist)
        automatic_mover.get_logger().info('Robot detenido')
        automatic_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()