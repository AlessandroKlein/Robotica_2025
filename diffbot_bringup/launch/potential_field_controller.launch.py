#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Obtener directorios de paquetes
    pkg_diffbot_bringup = get_package_share_directory('diffbot_bringup')
    
    # Argumentos de lanzamiento
    goal_x = LaunchConfiguration('goal_x', default='1.0')
    goal_y = LaunchConfiguration('goal_y', default='1.0')
    k_a = LaunchConfiguration('k_a', default='0.5')
    k_r = LaunchConfiguration('k_r', default='0.8')
    k_theta = LaunchConfiguration('k_theta', default='1.0')
    rho = LaunchConfiguration('rho', default='1.0')
    eta_0 = LaunchConfiguration('eta_0', default='0.5')
    v_max = LaunchConfiguration('v_max', default='0.5')
    w_max = LaunchConfiguration('w_max', default='1.0')
    epsilon_tol = LaunchConfiguration('epsilon_tol', default='0.05')
    control_frequency = LaunchConfiguration('control_frequency', default='10.0')
    
    # Incluir el launch file del sistema completo con obstáculos
    diffbot_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_diffbot_bringup, 'launch', 'diffbot_complete_system.launch.py')
        ),
        launch_arguments={
            'enable_obstacles': 'true',
            'obstacle_count': '6',
            'obstacle_size': 'medium',
            'obstacle_pattern': 'strategic'
        }.items()
    )
    
    # Nodo controlador de campos potenciales
    potential_field_controller_node = Node(
        package='diffbot_control',
        executable='potential_field_controller',
        name='potential_field_controller',
        output='screen',
        parameters=[{
            'goal_x': goal_x,
            'goal_y': goal_y,
            'k_a': k_a,
            'k_r': k_r,
            'k_theta': k_theta,
            'rho': rho,
            'eta_0': eta_0,
            'v_max': v_max,
            'w_max': w_max,
            'epsilon_tol': epsilon_tol,
            'control_frequency': control_frequency
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='1.0', description='Coordenada X del punto objetivo'),
        DeclareLaunchArgument('goal_y', default_value='1.0', description='Coordenada Y del punto objetivo'),
        DeclareLaunchArgument('k_a', default_value='0.5', description='Ganancia de atracción'),
        DeclareLaunchArgument('k_r', default_value='0.8', description='Ganancia de repulsión'),
        DeclareLaunchArgument('k_theta', default_value='1.0', description='Ganancia angular'),
        DeclareLaunchArgument('rho', default_value='1.0', description='Distancia de influencia atractiva'),
        DeclareLaunchArgument('eta_0', default_value='0.5', description='Distancia de influencia repulsiva'),
        DeclareLaunchArgument('v_max', default_value='0.5', description='Velocidad lineal máxima'),
        DeclareLaunchArgument('w_max', default_value='1.0', description='Velocidad angular máxima'),
        DeclareLaunchArgument('epsilon_tol', default_value='0.05', description='Tolerancia para considerar alcanzado el objetivo'),
        DeclareLaunchArgument('control_frequency', default_value='10.0', description='Frecuencia de control en Hz'),
        diffbot_system_launch,
        potential_field_controller_node
    ])