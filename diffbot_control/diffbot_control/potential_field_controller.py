#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import numpy as np

class PotentialFieldController(Node):
    def __init__(self):
        super().__init__('potential_field_controller')
        
        # Parámetros del controlador
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('k_a', 0.5)  # Ganancia de atracción
        self.declare_parameter('k_r', 0.8)  # Ganancia de repulsión
        self.declare_parameter('k_theta', 1.0)  # Ganancia angular
        self.declare_parameter('rho', 1.0)  # Distancia de influencia atractiva
        self.declare_parameter('eta_0', 0.5)  # Distancia de influencia repulsiva
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('epsilon_tol', 0.05)
        self.declare_parameter('control_frequency', 10.0)
        
        # Obtener parámetros
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.k_a = self.get_parameter('k_a').value
        self.k_r = self.get_parameter('k_r').value
        self.k_theta = self.get_parameter('k_theta').value
        self.rho = self.get_parameter('rho').value
        self.eta_0 = self.get_parameter('eta_0').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.epsilon_tol = self.get_parameter('epsilon_tol').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        # Variables de estado
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_reached = False
        self.laser_data = None
        
        # Crear QoS profile para odometría y laser
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
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile
        )
        
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        
        # Timer para el control
        self.timer = self.create_timer(1.0/self.control_frequency, self.control_loop)
        
        self.get_logger().info(f'Potential Field Controller iniciado - Objetivo: ({self.goal_x}, {self.goal_y})')
    
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
    
    def laser_callback(self, msg):
        self.laser_data = msg
    
    def normalize_angle(self, angle):
        # Normalizar ángulo al rango (-pi, pi)
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def segment_obstacles(self, ranges, angle_min, angle_increment):
        """
        Segmenta las mediciones del LIDAR en obstáculos distintos
        basándose en discontinuidades (rayos no reflejados)
        """
        obstacles = []
        current_obstacle = []
        
        for i, r in enumerate(ranges):
            if r < float('inf') and r > 0.01:  # Medición válida
                angle = angle_min + i * angle_increment
                current_obstacle.append((r, angle))
            elif current_obstacle:  # Fin de un obstáculo
                if len(current_obstacle) > 0:
                    obstacles.append(current_obstacle)
                current_obstacle = []
        
        # Añadir el último obstáculo si existe
        if current_obstacle:
            obstacles.append(current_obstacle)
        
        return obstacles
    
    def calculate_attractive_force(self):
        """
        Calcula la fuerza de atracción hacia el punto objetivo
        """
        delta_x = self.goal_x - self.x
        delta_y = self.goal_y - self.y
        distance = math.sqrt(delta_x**2 + delta_y**2)
        
        if distance <= self.rho:
            f_attr_x = -self.k_a * delta_x
            f_attr_y = -self.k_a * delta_y
        else:
            f_attr_x = -self.rho * self.k_a * delta_x / distance
            f_attr_y = -self.rho * self.k_a * delta_y / distance
        
        return np.array([f_attr_x, f_attr_y])
    
    def calculate_repulsive_forces(self, obstacles):
        """
        Calcula las fuerzas repulsivas de todos los obstáculos detectados
        """
        f_rep_total = np.array([0.0, 0.0])
        
        for obstacle in obstacles:
            # Encontrar el punto más cercano en el obstáculo
            min_dist_point = min(obstacle, key=lambda x: x[0])
            d_i = min_dist_point[0]  # Distancia mínima
            alpha_i = min_dist_point[1]  # Ángulo correspondiente
            
            # Calcular ángulo global
            gamma_i = self.theta + alpha_i
            
            # Calcular coordenadas del obstáculo en el marco inercial
            x_o = self.x + d_i * math.cos(gamma_i)
            y_o = self.y + d_i * math.sin(gamma_i)
            
            # Vector desde el obstáculo al robot
            vec_o_to_p = np.array([self.x - x_o, self.y - y_o])
            dist_o_to_p = np.linalg.norm(vec_o_to_p)
            
            # Calcular fuerza repulsiva solo si está dentro del rango de influencia
            if dist_o_to_p <= self.eta_0:
                f_rep_mag = self.k_r * (1.0/dist_o_to_p - 1.0/self.eta_0) / (dist_o_to_p**3)
                f_rep = f_rep_mag * vec_o_to_p
                f_rep_total += f_rep
        
        return f_rep_total
    
    def control_loop(self):
        if self.goal_reached or self.laser_data is None:
            return
        
        # Calcular error de posición
        delta_x = self.goal_x - self.x
        delta_y = self.goal_y - self.y
        distance = math.sqrt(delta_x**2 + delta_y**2)
        
        # Verificar si se alcanzó el objetivo
        if distance < self.epsilon_tol:
            self.get_logger().info('¡Objetivo alcanzado!')
            self.goal_reached = True
            
            # Enviar comando de velocidad cero
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd_vel_pub.publish(twist_msg)
            return
        
        # 1. Segmentar obstáculos del LIDAR
        obstacles = self.segment_obstacles(
            self.laser_data.ranges,
            self.laser_data.angle_min,
            self.laser_data.angle_increment
        )
        
        # 2. Calcular fuerza de atracción
        f_attr = self.calculate_attractive_force()
        
        # 3. Calcular fuerzas repulsivas
        f_rep = self.calculate_repulsive_forces(obstacles)
        
        # 4. Calcular fuerza total
        f_total = f_attr + f_rep
        
        # 5. Calcular componente angular
        theta_f_total = math.atan2(f_total[1], f_total[0])
        f_total_theta = self.k_theta * self.normalize_angle(theta_f_total - self.theta)
        
        # 6. Calcular velocidades lineal y angular
        v = f_total[0] * math.cos(self.theta) + f_total[1] * math.sin(self.theta)
        w = f_total_theta
        
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
        self.get_logger().debug(f'Fuerza total: ({f_total[0]:.2f}, {f_total[1]:.2f})')
        self.get_logger().debug(f'v: {v:.2f}, w: {w:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()