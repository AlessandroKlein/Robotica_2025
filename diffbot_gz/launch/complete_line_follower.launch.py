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

    return LaunchDescription([
        set_env_vars_resources,
        set_env_vars_ogre,
        gazebo_launch
    ])