#!/usr/bin/env python3
"""
Generador de Obstáculos Configurables para DiffBot

Este nodo genera obstáculos dinámicos en Gazebo con diferentes patrones
y configuraciones para crear entornos de simulación desafiantes.

Autor: Sistema DiffBot
Fecha: 2025
Versión: 1.0
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose, Point, Quaternion
import random
import math
import time


class ObstacleSpawner(Node):
    """
    Nodo para generar obstáculos configurables en Gazebo.
    
    Soporta diferentes patrones de colocación y tamaños de obstáculos
    para crear entornos de simulación variados y desafiantes.
    """
    
    def __init__(self):
        super().__init__('obstacle_spawner')
        
        # Parámetros configurables
        self.declare_parameter('obstacle_count', 5)
        self.declare_parameter('obstacle_size', 'medium')
        self.declare_parameter('obstacle_pattern', 'strategic')
        self.declare_parameter('enable_obstacles', True)
        
        # Obtener parámetros
        self.obstacle_count = self.get_parameter('obstacle_count').value
        self.obstacle_size = self.get_parameter('obstacle_size').value
        self.obstacle_pattern = self.get_parameter('obstacle_pattern').value
        self.enable_obstacles = self.get_parameter('enable_obstacles').value
        
        # Cliente para spawning de entidades
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Lista de obstáculos spawneados
        self.spawned_obstacles = []
        
        # Definir tamaños de obstáculos
        self.obstacle_sizes = {
            'small': {'x': 0.2, 'y': 0.2, 'z': 0.3},
            'medium': {'x': 0.4, 'y': 0.4, 'z': 0.5},
            'large': {'x': 0.6, 'y': 0.6, 'z': 0.8}
        }
        
        # Esperar a que el servicio esté disponible
        self.get_logger().info('Esperando servicio de spawn de Gazebo...')
        self.spawn_client.wait_for_service(timeout_sec=10.0)
        
        if self.enable_obstacles:
            # Timer para generar obstáculos después de un breve delay
            self.create_timer(2.0, self.spawn_obstacles_callback)
        
        self.get_logger().info(f'ObstacleSpawner iniciado - Patrón: {self.obstacle_pattern}, '
                              f'Cantidad: {self.obstacle_count}, Tamaño: {self.obstacle_size}')
    
    def generate_obstacle_sdf(self, name, size_dict):
        """
        Genera el SDF para un obstáculo con el tamaño especificado.
        
        Args:
            name (str): Nombre del obstáculo
            size_dict (dict): Diccionario con dimensiones x, y, z
            
        Returns:
            str: Contenido SDF del obstáculo
        """
        return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>{size_dict['x']} {size_dict['y']} {size_dict['z']}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{size_dict['x']} {size_dict['y']} {size_dict['z']}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    
    def get_strategic_positions(self, count):
        """
        Genera posiciones estratégicas para obstáculos.
        
        Args:
            count (int): Número de obstáculos a posicionar
            
        Returns:
            list: Lista de posiciones (x, y)
        """
        positions = []
        
        # Posiciones estratégicas que crean desafíos sin bloquear completamente
        strategic_spots = [
            (2.0, 1.0),   # Lado derecho
            (-1.5, -1.0), # Lado izquierdo
            (0.5, 2.0),   # Frente centro
            (-2.0, 0.5),  # Izquierda centro
            (1.0, -1.5),  # Derecha atrás
            (0.0, -2.5),  # Centro atrás
            (2.5, -0.5),  # Extremo derecho
            (-2.5, 1.5),  # Extremo izquierdo
        ]
        
        # Seleccionar posiciones según el conteo
        selected_positions = strategic_spots[:min(count, len(strategic_spots))]
        
        return selected_positions
    
    def get_corridor_positions(self, count):
        """
        Genera posiciones para crear un patrón de corredor.
        
        Args:
            count (int): Número de obstáculos
            
        Returns:
            list: Lista de posiciones (x, y)
        """
        positions = []
        corridor_width = 1.0
        
        for i in range(count // 2):
            x = i * 1.5 + 1.0
            positions.append((x, corridor_width))
            positions.append((x, -corridor_width))
        
        # Si hay un obstáculo impar, agregarlo al final
        if count % 2 == 1:
            positions.append((count * 0.75, 0.0))
        
        return positions[:count]
    
    def get_maze_positions(self, count):
        """
        Genera posiciones para crear un patrón tipo laberinto.
        
        Args:
            count (int): Número de obstáculos
            
        Returns:
            list: Lista de posiciones (x, y)
        """
        positions = []
        
        # Patrón de laberinto simple
        maze_pattern = [
            (1.0, 0.5),
            (1.0, -0.5),
            (2.0, 1.0),
            (2.0, -1.0),
            (0.0, 1.5),
            (0.0, -1.5),
            (3.0, 0.0),
            (-1.0, 0.0),
        ]
        
        return maze_pattern[:count]
    
    def get_random_positions(self, count):
        """
        Genera posiciones aleatorias para obstáculos.
        
        Args:
            count (int): Número de obstáculos
            
        Returns:
            list: Lista de posiciones (x, y)
        """
        positions = []
        
        for _ in range(count):
            # Generar posiciones aleatorias evitando el área del robot (cerca del origen)
            while True:
                x = random.uniform(-3.0, 3.0)
                y = random.uniform(-3.0, 3.0)
                
                # Evitar área cercana al origen (donde inicia el robot)
                if math.sqrt(x*x + y*y) > 0.8:
                    positions.append((x, y))
                    break
        
        return positions
    
    def get_obstacle_positions(self):
        """
        Obtiene las posiciones de obstáculos según el patrón configurado.
        
        Returns:
            list: Lista de posiciones (x, y)
        """
        if self.obstacle_pattern == 'strategic':
            return self.get_strategic_positions(self.obstacle_count)
        elif self.obstacle_pattern == 'corridor':
            return self.get_corridor_positions(self.obstacle_count)
        elif self.obstacle_pattern == 'maze':
            return self.get_maze_positions(self.obstacle_count)
        elif self.obstacle_pattern == 'random':
            return self.get_random_positions(self.obstacle_count)
        else:
            self.get_logger().warn(f'Patrón desconocido: {self.obstacle_pattern}, usando strategic')
            return self.get_strategic_positions(self.obstacle_count)
    
    def get_obstacle_size(self, index):
        """
        Obtiene el tamaño del obstáculo según la configuración.
        
        Args:
            index (int): Índice del obstáculo
            
        Returns:
            dict: Diccionario con dimensiones del obstáculo
        """
        if self.obstacle_size == 'mixed':
            # Alternar entre tamaños
            sizes = ['small', 'medium', 'large']
            size_key = sizes[index % len(sizes)]
            return self.obstacle_sizes[size_key]
        else:
            return self.obstacle_sizes.get(self.obstacle_size, self.obstacle_sizes['medium'])
    
    def spawn_obstacles_callback(self):
        """
        Callback para generar obstáculos en Gazebo.
        """
        if not self.enable_obstacles:
            return
        
        positions = self.get_obstacle_positions()
        
        self.get_logger().info(f'Generando {len(positions)} obstáculos con patrón {self.obstacle_pattern}')
        
        for i, (x, y) in enumerate(positions):
            obstacle_name = f'obstacle_{i+1}'
            size_dict = self.get_obstacle_size(i)
            
            # Crear pose
            pose = Pose()
            pose.position = Point(x=x, y=y, z=size_dict['z']/2)  # Colocar sobre el suelo
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            # Generar SDF
            sdf_content = self.generate_obstacle_sdf(obstacle_name, size_dict)
            
            # Crear request
            request = SpawnEntity.Request()
            request.name = obstacle_name
            request.xml = sdf_content
            request.initial_pose = pose
            
            # Enviar request
            future = self.spawn_client.call_async(request)
            
            # Agregar a la lista de obstáculos
            self.spawned_obstacles.append(obstacle_name)
            
            self.get_logger().info(f'Obstáculo {obstacle_name} generado en ({x:.1f}, {y:.1f})')
            
            # Pequeño delay entre spawns
            time.sleep(0.1)
        
        self.get_logger().info(f'Generación de obstáculos completada: {len(self.spawned_obstacles)} obstáculos')
    
    def cleanup_obstacles(self):
        """
        Limpia todos los obstáculos generados.
        """
        for obstacle_name in self.spawned_obstacles:
            request = DeleteEntity.Request()
            request.name = obstacle_name
            
            future = self.delete_client.call_async(request)
            self.get_logger().info(f'Eliminando obstáculo: {obstacle_name}')
        
        self.spawned_obstacles.clear()


def main(args=None):
    """
    Función principal del nodo ObstacleSpawner.
    """
    rclpy.init(args=args)
    
    obstacle_spawner = ObstacleSpawner()
    
    try:
        rclpy.spin(obstacle_spawner)
    except KeyboardInterrupt:
        obstacle_spawner.get_logger().info('Deteniendo ObstacleSpawner...')
        obstacle_spawner.cleanup_obstacles()
    finally:
        obstacle_spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()