#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped, Point, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from enum import Enum


class Bug2State(Enum):
    """Estados de la máquina de estados para el algoritmo Bug2"""
    TURN_TO_GOAL = 0
    MOVE_TO_GOAL = 1
    FOLLOW_WALL = 2
    REACHED = 3


class Bug2Controller(Node):
    """
    Controlador Bug2 con máquina de estados finito
    Algoritmo de navegación que combina go-to-point con wall-following
    """
    
    def __init__(self):
        super().__init__('bug2_controller')
        
        # Parámetros
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('k_rho', 0.5)
        self.declare_parameter('k_alpha', 1.0)
        self.declare_parameter('k_beta', -0.1)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('w_max', 2.84)
        self.declare_parameter('epsilon_tol', 0.05)
        self.declare_parameter('epsilon_theta', 0.1)
        self.declare_parameter('obstacle_distance', 0.5)
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
        self.epsilon_theta = self.get_parameter('epsilon_theta').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        control_frequency = self.get_parameter('control_frequency').value
        
        # Estado actual del robot
        self.current_pose = Pose()
        self.current_position = Point()
        self.current_yaw = 0.0
        
        # Estado de la máquina de estados
        self.current_state = Bug2State.TURN_TO_GOAL
        self.hit_point = None  # Punto donde se encontró el obstáculo
        self.min_distance_to_goal = float('inf')
        self.goal_reached = False
        
        # Publishers y Subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', qos_profile)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile
        )
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile
        )
        
        # Timer para el control
        self.timer = self.create_timer(1.0/control_frequency, self.control_loop)
        
        # Datos del LIDAR
        self.laser_data = None
        self.front_distance = float('inf')
        self.front_left_distance = float('inf')
        self.front_right_distance = float('inf')
        
        self.get_logger().info(f'Bug2 Controller iniciado - Objetivo: ({self.goal_x}, {self.goal_y})')
        self.get_logger().info(f'Estado inicial: {self.current_state.name}')
    
    def odom_callback(self, msg):
        """Callback para la odometría"""
        self.current_pose = msg.pose.pose
        self.current_position = msg.pose.pose.position
        
        # Obtener orientación (yaw) del cuaternión
        orientation = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                       1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
    
    def laser_callback(self, msg):
        """Callback para los datos del LIDAR"""
        self.laser_data = msg
        
        # Segmentar los datos del LIDAR
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isnan(ranges), float('inf'), ranges)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        
        # Definir ángulos para las zonas
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)
        
        # Zona frontal (±15 grados)
        front_indices = []
        for i in range(num_ranges):
            angle = angle_min + i * angle_increment
            if -0.26 <= angle <= 0.26:  # ±15 grados
                front_indices.append(i)
        
        # Zona frontal-izquierda (15-45 grados)
        front_left_indices = []
        for i in range(num_ranges):
            angle = angle_min + i * angle_increment
            if 0.26 < angle <= 0.79:  # 15-45 grados
                front_left_indices.append(i)
        
        # Zona frontal-derecha (-45 a -15 grados)
        front_right_indices = []
        for i in range(num_ranges):
            angle = angle_min + i * angle_increment
            if -0.79 <= angle < -0.26:  # -45 a -15 grados
                front_right_indices.append(i)
        
        # Calcular distancias mínimas
        if front_indices:
            self.front_distance = np.min(ranges[front_indices])
        else:
            self.front_distance = float('inf')
            
        if front_left_indices:
            self.front_left_distance = np.min(ranges[front_left_indices])
        else:
            self.front_left_distance = float('inf')
            
        if front_right_indices:
            self.front_right_distance = np.min(ranges[front_right_indices])
        else:
            self.front_right_distance = float('inf')
    
    def calculate_polar_coordinates(self):
        """Calcular coordenadas polares respecto al objetivo"""
        dx = self.goal_x - self.current_position.x
        dy = self.goal_y - self.current_position.y
        
        # Distancia al objetivo
        rho = math.sqrt(dx*dx + dy*dy)
        
        # Ángulo hacia el objetivo
        goal_angle = math.atan2(dy, dx)
        alpha = goal_angle - self.current_yaw
        
        # Normalizar alpha al rango [-π, π]
        while alpha > math.pi:
            alpha -= 2*math.pi
        while alpha < -math.pi:
            alpha += 2*math.pi
        
        # Ángulo de orientación final
        beta = -self.current_yaw - alpha
        
        # Normalizar beta al rango [-π, π]
        while beta > math.pi:
            beta -= 2*math.pi
        while beta < -math.pi:
            beta += 2*math.pi
        
        return rho, alpha, beta
    
    def is_path_clear(self):
        """Verificar si el camino hacia el objetivo está libre"""
        return self.front_distance > self.obstacle_distance
    
    def is_on_goal_line(self):
        """Verificar si el robot está en la línea que conecta el punto inicial con el objetivo"""
        if self.hit_point is None:
            return False
        
        # Línea desde el punto inicial al objetivo
        x1, y1 = 0.0, 0.0  # Punto inicial (asumimos que es el origen)
        x2, y2 = self.goal_x, self.goal_y
        
        # Punto actual
        x0, y0 = self.current_position.x, self.current_position.y
        
        # Calcular la distancia perpendicular del punto actual a la línea
        # Fórmula: |Ax + By + C| / sqrt(A² + B²) donde Ax + By + C = 0
        # Para la línea que pasa por (x1,y1) y (x2,y2): (y2-y1)x - (x2-x1)y + (x2y1-x1y2) = 0
        A = y2 - y1
        B = x1 - x2
        C = x2*y1 - x1*y2
        
        distance_to_line = abs(A*x0 + B*y0 + C) / math.sqrt(A*A + B*B)
        
        # Considerar que está en la línea si la distancia es pequeña
        return distance_to_line < 0.1
    
    def get_velocity_commands(self, rho, alpha, beta):
        """Calcular comandos de velocidad basados en coordenadas polares"""
        # Ley de control
        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta
        
        # Limitar velocidades
        v = max(min(v, self.v_max), -self.v_max)
        w = max(min(w, self.w_max), -self.w_max)
        
        return v, w
    
    def wall_following(self):
        """Control para seguir paredes (wall-following)"""
        # Simple wall-following: girar hacia el lado con más espacio
        if self.front_left_distance > self.front_right_distance:
            # Más espacio a la izquierda, girar izquierda
            v = 0.1  # Velocidad baja hacia adelante
            w = 0.5  # Giro a la izquierda
        else:
            # Más espacio a la derecha, girar derecha
            v = 0.1  # Velocidad baja hacia adelante
            w = -0.5  # Giro a la derecha
        
        # Si hay obstáculo muy cerca, retroceder
        if self.front_distance < 0.3:
            v = -0.1
            w = 0.0
        
        return v, w
    
    def control_loop(self):
        """Bucle principal de control con máquina de estados"""
        if self.current_position is None or self.laser_data is None:
            return
        
        # Calcular coordenadas polares
        rho, alpha, beta = self.calculate_polar_coordinates()
        
        # Verificar si se alcanzó el objetivo
        if rho < self.epsilon_tol:
            self.current_state = Bug2State.REACHED
            self.goal_reached = True
            self.get_logger().info('¡Objetivo alcanzado!')
        
        # Máquina de estados
        if self.current_state == Bug2State.TURN_TO_GOAL:
            # Girar hacia el objetivo
            if abs(alpha) > self.epsilon_theta:
                v = 0.0
                w = self.k_alpha * alpha
                w = max(min(w, self.w_max), -self.w_max)
            else:
                # Ya está orientado, pasar a moverse
                self.current_state = Bug2State.MOVE_TO_GOAL
                self.get_logger().info('Transición: TURN_TO_GOAL -> MOVE_TO_GOAL')
                return
        
        elif self.current_state == Bug2State.MOVE_TO_GOAL:
            # Mover hacia el objetivo
            if not self.is_path_clear():
                # Hay obstáculo, cambiar a seguir pared
                self.current_state = Bug2State.FOLLOW_WALL
                self.hit_point = (self.current_position.x, self.current_position.y)
                self.min_distance_to_goal = rho
                self.get_logger().info(f'Transición: MOVE_TO_GOAL -> FOLLOW_WALL (Hit point: {self.hit_point})')
                return
            else:
                # Camino libre, mover hacia el objetivo
                v, w = self.get_velocity_commands(rho, alpha, beta)
        
        elif self.current_state == Bug2State.FOLLOW_WALL:
            # Seguir pared
            if self.is_on_goal_line() and rho < self.min_distance_to_goal:
                # Está en la línea y más cerca del objetivo, volver a moverse hacia el objetivo
                self.current_state = Bug2State.MOVE_TO_GOAL
                self.get_logger().info(f'Transición: FOLLOW_WALL -> MOVE_TO_GOAL (Distance: {rho:.2f})')
                return
            else:
                # Seguir la pared
                v, w = self.wall_following()
        
        elif self.current_state == Bug2State.REACHED:
            # Objetivo alcanzado, detenerse
            v = 0.0
            w = 0.0
            self.get_logger().info('Estado: REACHED - Robot detenido')
        
        # Publicar comandos de velocidad
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = v
        twist.twist.angular.z = w
        
        self.cmd_vel_pub.publish(twist)
        
        # Log de estado
        self.get_logger().debug(f'Estado: {self.current_state.name}, Distancia: {rho:.2f}, Alpha: {alpha:.2f}, Obstáculo frontal: {self.front_distance:.2f}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = Bug2Controller()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Detener el robot antes de cerrar
        twist = TwistStamped()
        twist.header.stamp = controller.get_clock().now().to_msg()
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.0
        controller.cmd_vel_pub.publish(twist)
        
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()