#!/usr/bin/env python3
"""
Nodo para recolección automática de datos de calibración de odometría.

Este nodo ejecuta múltiples trayectorias cuadradas y recolecta los errores
de posición para calcular los parámetros de calibración.

Autor: Sistema de calibración de odometría
Fecha: 2025
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import json
import os
from datetime import datetime


class CalibrationDataCollector(Node):
    def __init__(self):
        super().__init__('calibration_data_collector')
        
        # Declarar parámetros
        self.declare_parameter('num_runs', 5)  # Número de corridas por dirección
        self.declare_parameter('side_length', 4.0)  # Longitud del lado en metros
        self.declare_parameter('linear_velocity', 0.2)  # Velocidad lineal en m/s
        self.declare_parameter('angular_velocity', 0.314159)  # Velocidad angular en rad/s
        self.declare_parameter('output_file', 'calibration_data.json')  # Archivo de salida
        
        # Obtener parámetros
        self.num_runs = self.get_parameter('num_runs').get_parameter_value().integer_value
        self.side_length = self.get_parameter('side_length').get_parameter_value().double_value
        self.linear_vel = self.get_parameter('linear_velocity').get_parameter_value().double_value
        self.angular_vel = self.get_parameter('angular_velocity').get_parameter_value().double_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        
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
        
        # Datos de calibración
        self.calibration_data = {
            'metadata': {
                'timestamp': datetime.now().isoformat(),
                'side_length': self.side_length,
                'linear_velocity': self.linear_vel,
                'angular_velocity': self.angular_vel,
                'num_runs': self.num_runs
            },
            'ccw_runs': [],
            'cw_runs': []
        }
        
        # Calcular tiempos
        self.move_time = self.side_length / self.linear_vel
        self.turn_time = (math.pi / 2) / abs(self.angular_vel)
        
        self.get_logger().info(f'Iniciando recolección de datos de calibración')
        self.get_logger().info(f'Número de corridas por dirección: {self.num_runs}')
        self.get_logger().info(f'Archivo de salida: {self.output_file}')
        
        # Esperar un poco antes de comenzar
        time.sleep(3.0)
        
        # Ejecutar la recolección de datos
        self.collect_calibration_data()
    
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
        time.sleep(0.5)
    
    def wait_for_odometry(self):
        """Espera a recibir datos de odometría."""
        while (self.robot_odom is None or self.gazebo_odom is None) and rclpy.ok():
            self.get_logger().info('Esperando datos de odometría...')
            rclpy.spin_once(self, timeout_sec=1.0)
    
    def execute_square_trajectory(self, direction='ccw'):
        """Ejecuta una trayectoria cuadrada y retorna los errores."""
        self.get_logger().info(f'Ejecutando trayectoria {direction.upper()}...')
        
        # Configurar velocidad angular según dirección
        angular_vel = abs(self.angular_vel) if direction == 'ccw' else -abs(self.angular_vel)
        
        # Esperar datos de odometría
        self.wait_for_odometry()
        
        # Guardar posición inicial
        initial_robot_pos = (
            self.robot_odom.pose.pose.position.x,
            self.robot_odom.pose.pose.position.y
        )
        initial_gazebo_pos = (
            self.gazebo_odom.pose.pose.position.x,
            self.gazebo_odom.pose.pose.position.y
        )
        
        # Ejecutar los 4 lados del cuadrado
        for side in range(4):
            # Mover hacia adelante
            start_time = time.time()
            while (time.time() - start_time) < self.move_time:
                self.send_velocity(self.linear_vel, 0.0)
                rclpy.spin_once(self, timeout_sec=0.01)
            
            self.stop_robot()
            
            # Girar 90 grados (excepto en el último lado)
            if side < 3:
                start_time = time.time()
                while (time.time() - start_time) < self.turn_time:
                    self.send_velocity(0.0, angular_vel)
                    rclpy.spin_once(self, timeout_sec=0.01)
                
                self.stop_robot()
            
            time.sleep(0.5)
        
        # Pausa final y obtener posición final
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.5)
        
        final_robot_pos = (
            self.robot_odom.pose.pose.position.x,
            self.robot_odom.pose.pose.position.y
        )
        final_gazebo_pos = (
            self.gazebo_odom.pose.pose.position.x,
            self.gazebo_odom.pose.pose.position.y
        )
        
        # Calcular errores
        robot_error_x = final_robot_pos[0] - initial_robot_pos[0]
        robot_error_y = final_robot_pos[1] - initial_robot_pos[1]
        gazebo_error_x = final_gazebo_pos[0] - initial_gazebo_pos[0]
        gazebo_error_y = final_gazebo_pos[1] - initial_gazebo_pos[1]
        
        error_x = robot_error_x - gazebo_error_x
        error_y = robot_error_y - gazebo_error_y
        
        return {
            'initial_robot_pos': initial_robot_pos,
            'final_robot_pos': final_robot_pos,
            'initial_gazebo_pos': initial_gazebo_pos,
            'final_gazebo_pos': final_gazebo_pos,
            'error_x': error_x,
            'error_y': error_y
        }
    
    def reset_robot_position(self):
        """Resetea la posición del robot (simulado con pausa)."""
        self.get_logger().info('Reseteando posición del robot...')
        # En una implementación real, aquí se resetearía la posición del robot en Gazebo
        # Por ahora, solo hacemos una pausa
        time.sleep(2.0)
    
    def collect_calibration_data(self):
        """Recolecta todos los datos de calibración."""
        self.get_logger().info('=== INICIANDO RECOLECCIÓN DE DATOS ===')
        
        # Recolectar datos para trayectorias CCW (anti-horario)
        self.get_logger().info(f'Recolectando {self.num_runs} corridas CCW...')
        for run in range(self.num_runs):
            self.get_logger().info(f'--- Corrida CCW {run + 1}/{self.num_runs} ---')
            
            try:
                result = self.execute_square_trajectory('ccw')
                self.calibration_data['ccw_runs'].append(result)
                
                self.get_logger().info(f'Error X: {result["error_x"]:.6f} m')
                self.get_logger().info(f'Error Y: {result["error_y"]:.6f} m')
                
                if run < self.num_runs - 1:  # No resetear en la última corrida
                    self.reset_robot_position()
                    
            except Exception as e:
                self.get_logger().error(f'Error en corrida CCW {run + 1}: {e}')
        
        # Pausa entre direcciones
        time.sleep(5.0)
        
        # Recolectar datos para trayectorias CW (horario)
        self.get_logger().info(f'Recolectando {self.num_runs} corridas CW...')
        for run in range(self.num_runs):
            self.get_logger().info(f'--- Corrida CW {run + 1}/{self.num_runs} ---')
            
            try:
                result = self.execute_square_trajectory('cw')
                self.calibration_data['cw_runs'].append(result)
                
                self.get_logger().info(f'Error X: {result["error_x"]:.6f} m')
                self.get_logger().info(f'Error Y: {result["error_y"]:.6f} m')
                
                if run < self.num_runs - 1:  # No resetear en la última corrida
                    self.reset_robot_position()
                    
            except Exception as e:
                self.get_logger().error(f'Error en corrida CW {run + 1}: {e}')
        
        # Guardar datos
        self.save_calibration_data()
        
        # Calcular y mostrar estadísticas
        self.calculate_statistics()
    
    def save_calibration_data(self):
        """Guarda los datos de calibración en un archivo JSON."""
        try:
            with open(self.output_file, 'w') as f:
                json.dump(self.calibration_data, f, indent=2)
            self.get_logger().info(f'Datos guardados en: {self.output_file}')
        except Exception as e:
            self.get_logger().error(f'Error al guardar datos: {e}')
    
    def calculate_statistics(self):
        """Calcula y muestra estadísticas de los datos recolectados."""
        self.get_logger().info('=== ESTADÍSTICAS DE CALIBRACIÓN ===')
        
        # Calcular centroides CCW
        if self.calibration_data['ccw_runs']:
            ccw_errors_x = [run['error_x'] for run in self.calibration_data['ccw_runs']]
            ccw_errors_y = [run['error_y'] for run in self.calibration_data['ccw_runs']]
            ccw_mean_x = sum(ccw_errors_x) / len(ccw_errors_x)
            ccw_mean_y = sum(ccw_errors_y) / len(ccw_errors_y)
            
            self.get_logger().info(f'CCW - Media εx: {ccw_mean_x:.6f} m')
            self.get_logger().info(f'CCW - Media εy: {ccw_mean_y:.6f} m')
        
        # Calcular centroides CW
        if self.calibration_data['cw_runs']:
            cw_errors_x = [run['error_x'] for run in self.calibration_data['cw_runs']]
            cw_errors_y = [run['error_y'] for run in self.calibration_data['cw_runs']]
            cw_mean_x = sum(cw_errors_x) / len(cw_errors_x)
            cw_mean_y = sum(cw_errors_y) / len(cw_errors_y)
            
            self.get_logger().info(f'CW - Media εx: {cw_mean_x:.6f} m')
            self.get_logger().info(f'CW - Media εy: {cw_mean_y:.6f} m')
        
        self.get_logger().info('=== RECOLECCIÓN COMPLETADA ===')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CalibrationDataCollector()
        # El nodo se ejecuta una vez y termina
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()