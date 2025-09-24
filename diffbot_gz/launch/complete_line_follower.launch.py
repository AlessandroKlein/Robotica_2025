#!/usr/bin/env python3
"""
Launch file completo para el sistema de seguimiento de líneas

Este archivo configura y ejecuta automáticamente:
- Gazebo con el modelo LineTrack
- Robot diffbot con sensor de cámara
- Bridge ROS-Gazebo para comunicación
- Nodo LineDetector con parámetros optimizados
- Motor de renderizado ogre para mejor rendimiento

Uso:
    ros2 launch diffbot_gz complete_line_follower.launch.py

Autor: Sistema de seguimiento de líneas
Fecha: 2025
"""

import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Configurar variables de entorno para Gazebo
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            FindPackageShare('diffbot_gz').find('diffbot_gz'),
            'worlds'
        )
    )
    
    # Configurar motor de renderizado
    set_env_vars_ogre = AppendEnvironmentVariable(
        'GZ_SIM_RENDER_ENGINE',
        'ogre'
    )

    # Incluir el launch básico de Gazebo con el robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('diffbot_gz'),
                'launch',
                'line_follower.launch.py'
            ])
        ])
    )

    # Nodo LineDetector con parámetros optimizados para curvas cerradas
    line_detector_node = Node(
        package='diffbot_control',
        executable='line_detector',
        name='line_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Parámetros HSV optimizados para detección de líneas negras
            'hsv_lower_h': 0,      # Matiz mínimo
            'hsv_lower_s': 0,      # Saturación mínima  
            'hsv_lower_v': 0,      # Valor mínimo (negro puro)
            'hsv_upper_h': 180,    # Matiz máximo (todo el rango)
            'hsv_upper_s': 30,     # Saturación máxima reducida para evitar colores
            'hsv_upper_v': 30,     # Valor máximo reducido para detectar solo negro/gris oscuro
            # Parámetros de control mejorados
            'linear_speed': 0.2,   # Velocidad lineal aumentada
            'angular_gain': 2.0,   # Ganancia angular aumentada para mejor respuesta
            'search_angular_speed': 0.6,   # Velocidad de búsqueda aumentada
            'min_line_area': 50    # Área mínima reducida para mayor sensibilidad
        }]
    )

    # Nodo controlador diferencial para convertir cmd_vel a comandos de ruedas
    differential_drive_controller_node = Node(
        package='diffbot_control',
        executable='differential_drive_controller',
        name='differential_drive_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'wheel_radius': 0.035,
            'wheel_separation': 0.135
        }]
    )

    return LaunchDescription([
        set_env_vars_resources,
        set_env_vars_ogre,
        gazebo_launch,
        line_detector_node,
        differential_drive_controller_node
    ])