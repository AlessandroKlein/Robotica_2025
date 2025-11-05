#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import numpy as np

class PointFollower(Node):
    def __init__(self):
        super().__init__('point_follower')
        
        # Parámetros del controlador
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('k_rho', 0.5)
        self.declare_parameter('k_alpha', 1.0)
        self.declare_parameter('k_beta', -0.1)
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('epsilon_tol', 0.05)
        self.declare_parameter('control_frequency', 10.0)
        
        # Obtener parámetros
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.k_rho = self.get_parameter('k_rho').value
        self.k_alpha = self.get_parameter('k_alpha').value
        self.k_beta = self.get_parameter('k_beta').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.epsilon_tol = self.get_parameter('epsilon_tol').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        # Verificar criterio de estabilidad
        if not (self.k_rho > 0 and self.k_beta < 0 and (self.k_alpha - self.k_rho) > 0):
            self.get_logger().error('Los parámetros no cumplen con el criterio de estabilidad')
            self.get_logger().error(f'k_rho > 0: {self.k_rho > 0}')
            self.get_logger().error(f'k_beta < 0: {self.k_beta < 0}')
            self.get_logger().error(f'k_alpha - k_rho > 0: {(self.k_alpha - self.k_rho) > 0}')
        
        # Variables de estado
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_reached = False
        
        # Crear QoS profile para odometría
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Suscriptores y publicadores
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )
        
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        
        # Timer para el control
        self.timer = self.create_timer(1.0/self.control_frequency, self.control_loop)
        
        self.get_logger().info(f'Point Follower iniciado - Objetivo: ({self.goal_x}, {self.goal_y})')
    
    def odom_callback(self, msg):
        # Extraer posición y orientación del mensaje de odometría
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Convertir cuaternión a ángulo de Euler (yaw)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Conversión de cuaternión a ángulo de Euler (yaw)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.theta = math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        # Normalizar ángulo al rango (-pi, pi)
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def control_loop(self):
        if self.goal_reached:
            return
        
        # Calcular error de posición
        delta_x = self.goal_x - self.x
        delta_y = self.goal_y - self.y
        
        # Calcular rho (distancia al objetivo)
        rho = math.sqrt(delta_x**2 + delta_y**2)
        
        # Verificar si se alcanzó el objetivo
        if rho < self.epsilon_tol:
            self.get_logger().info('¡Objetivo alcanzado!')
            self.goal_reached = True
            
            # Enviar comando de velocidad cero
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd_vel_pub.publish(twist_msg)
            return
        
        # Calcular ángulos alpha y beta
        alpha = math.atan2(delta_y, delta_x) - self.theta
        alpha = self.normalize_angle(alpha)
        
        beta = -self.theta - alpha
        beta = self.normalize_angle(beta)
        
        # Calcular velocidades según la ley de control
        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta
        
        # Limitar velocidades
        v = min(max(-self.v_max, v), self.v_max)
        w = min(max(-self.w_max, w), self.w_max)
        
        # Crear y publicar mensaje de velocidad
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = v
        twist_msg.twist.angular.z = w
        self.cmd_vel_pub.publish(twist_msg)
        
        self.get_logger().debug(f'Posición: ({self.x:.2f}, {self.y:.2f}), Objetivo: ({self.goal_x:.2f}, {self.goal_y:.2f})')
        self.get_logger().debug(f'rho: {rho:.2f}, alpha: {alpha:.2f}, beta: {beta:.2f}')
        self.get_logger().debug(f'v: {v:.2f}, w: {w:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()