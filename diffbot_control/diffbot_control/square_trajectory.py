#!/usr/bin/env python3
"""
Nodo para ejecutar una trayectoria cuadrada de 4x4 metros para calibración de odometría.

Este nodo ejecuta una trayectoria cuadrada con rotaciones puras en las esquinas,
enviando comandos de velocidad preprogramados sin retroalimentación.

Autor: Sistema de calibración de odometría
Fecha: 2025
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math


class SquareTrajectoryNode(Node):
    def __init__(self):
        super().__init__('square_trajectory_node')
        
        # Declarar parámetros
        self.declare_parameter('direction', 'ccw')  # 'cw' para horario, 'ccw' para anti-horario
        self.declare_parameter('side_length', 4.0)  # Longitud del lado en metros
        self.declare_parameter('linear_velocity', 0.2)  # Velocidad lineal en m/s
        self.declare_parameter('angular_velocity', 0.314159)  # Velocidad angular en rad/s (π/10)
        
        # Obtener parámetros
        self.direction = self.get_parameter('direction').get_parameter_value().string_value
        self.side_length = self.get_parameter('side_length').get_parameter_value().double_value
        self.linear_vel = self.get_parameter('linear_velocity').get_parameter_value().double_value
        self.angular_vel = self.get_parameter('angular_velocity').get_parameter_value().double_value
        
        # Publisher para comandos de velocidad
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers para odometría
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.gazebo_odom_sub = self.create_subscription(
            Odometry, '/gazebo/odometry', self.gazebo_odom_callback, 10)
        
        # Variables para almacenar odometría
        self.robot_odom = None
        self.gazebo_odom = None
        
        # Configurar dirección de rotación
        if self.direction == 'cw':
            self.angular_vel = -abs(self.angular_vel)  # Horario (negativo)
        else:
            self.angular_vel = abs(self.angular_vel)   # Anti-horario (positivo)
        
        # Calcular tiempos
        self.move_time = self.side_length / self.linear_vel  # Tiempo para moverse un lado
        self.turn_time = (math.pi / 2) / abs(self.angular_vel)  # Tiempo para girar 90°
        
        self.get_logger().info(f'Iniciando trayectoria cuadrada {self.direction.upper()}')
        self.get_logger().info(f'Lado: {self.side_length}m, Vel. lineal: {self.linear_vel}m/s, Vel. angular: {self.angular_vel}rad/s')
        self.get_logger().info(f'Tiempo por lado: {self.move_time:.1f}s, Tiempo por giro: {self.turn_time:.1f}s')
        
        # Esperar un poco antes de comenzar
        time.sleep(2.0)
        
        # Ejecutar la trayectoria
        self.execute_square_trajectory()
    
    def odom_callback(self, msg):
        """Callback para odometría del robot calculada."""
        self.robot_odom = msg
    
    def gazebo_odom_callback(self, msg):
        """Callback para odometría real de Gazebo."""
        self.gazebo_odom = msg
    
    def send_velocity(self, linear_x=0.0, angular_z=0.0):
        """Envía comando de velocidad."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
    
    def stop_robot(self):
        """Detiene el robot."""
        self.send_velocity(0.0, 0.0)
        time.sleep(0.5)  # Asegurar que el comando se envíe
    
    def move_forward(self):
        """Mueve el robot hacia adelante por un lado del cuadrado."""
        self.get_logger().info(f'Moviendo hacia adelante por {self.move_time:.1f}s')
        
        start_time = time.time()
        while (time.time() - start_time) < self.move_time:
            self.send_velocity(self.linear_vel, 0.0)
            time.sleep(0.1)
        
        self.stop_robot()
    
    def turn_90_degrees(self):
        """Gira el robot 90 grados."""
        self.get_logger().info(f'Girando 90° ({self.direction}) por {self.turn_time:.1f}s')
        
        start_time = time.time()
        while (time.time() - start_time) < self.turn_time:
            self.send_velocity(0.0, self.angular_vel)
            time.sleep(0.1)
        
        self.stop_robot()
    
    def execute_square_trajectory(self):
        """Ejecuta la trayectoria cuadrada completa."""
        self.get_logger().info('Iniciando trayectoria cuadrada...')
        
        # Guardar posición inicial
        initial_robot_pos = None
        initial_gazebo_pos = None
        
        # Esperar a recibir datos de odometría
        while (self.robot_odom is None or self.gazebo_odom is None) and rclpy.ok():
            self.get_logger().info('Esperando datos de odometría...')
            rclpy.spin_once(self, timeout_sec=1.0)
        
        if self.robot_odom and self.gazebo_odom:
            initial_robot_pos = (
                self.robot_odom.pose.pose.position.x,
                self.robot_odom.pose.pose.position.y
            )
            initial_gazebo_pos = (
                self.gazebo_odom.pose.pose.position.x,
                self.gazebo_odom.pose.pose.position.y
            )
            self.get_logger().info(f'Posición inicial robot: {initial_robot_pos}')
            self.get_logger().info(f'Posición inicial Gazebo: {initial_gazebo_pos}')
        
        # Ejecutar los 4 lados del cuadrado
        for side in range(4):
            self.get_logger().info(f'--- Lado {side + 1}/4 ---')
            
            # Mover hacia adelante
            self.move_forward()
            
            # Girar 90 grados (excepto en el último lado)
            if side < 3:
                self.turn_90_degrees()
            
            # Pequeña pausa entre movimientos
            time.sleep(0.5)
        
        # Pausa final
        time.sleep(1.0)
        
        # Mostrar posiciones finales
        if self.robot_odom and self.gazebo_odom:
            final_robot_pos = (
                self.robot_odom.pose.pose.position.x,
                self.robot_odom.pose.pose.position.y
            )
            final_gazebo_pos = (
                self.gazebo_odom.pose.pose.position.x,
                self.gazebo_odom.pose.pose.position.y
            )
            
            self.get_logger().info('=== RESULTADOS FINALES ===')
            self.get_logger().info(f'Posición inicial robot: {initial_robot_pos}')
            self.get_logger().info(f'Posición final robot: {final_robot_pos}')
            self.get_logger().info(f'Posición inicial Gazebo: {initial_gazebo_pos}')
            self.get_logger().info(f'Posición final Gazebo: {final_gazebo_pos}')
            
            # Calcular errores
            if initial_robot_pos and initial_gazebo_pos:
                robot_error_x = final_robot_pos[0] - initial_robot_pos[0]
                robot_error_y = final_robot_pos[1] - initial_robot_pos[1]
                gazebo_error_x = final_gazebo_pos[0] - initial_gazebo_pos[0]
                gazebo_error_y = final_gazebo_pos[1] - initial_gazebo_pos[1]
                
                error_x = robot_error_x - gazebo_error_x
                error_y = robot_error_y - gazebo_error_y
                
                self.get_logger().info(f'Error en X (εx): {error_x:.6f} m')
                self.get_logger().info(f'Error en Y (εy): {error_y:.6f} m')
                self.get_logger().info('=== FIN DE TRAYECTORIA ===')
        
        self.get_logger().info('Trayectoria cuadrada completada')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SquareTrajectoryNode()
        # El nodo se ejecuta una vez y termina
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()