#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Obtener el directorio del paquete
    diffbot_bringup_dir = get_package_share_directory('diffbot_bringup')
    
    # Argumentos de lanzamiento
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='3.0',
        description='Coordenada X del punto objetivo'
    )
    
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='3.0',
        description='Coordenada Y del punto objetivo'
    )
    
    k_rho_arg = DeclareLaunchArgument(
        'k_rho',
        default_value='0.5',
        description='Ganancia para la distancia al objetivo'
    )
    
    k_alpha_arg = DeclareLaunchArgument(
        'k_alpha',
        default_value='1.0',
        description='Ganancia para el ángulo hacia el objetivo'
    )
    
    k_beta_arg = DeclareLaunchArgument(
        'k_beta',
        default_value='-0.1',
        description='Ganancia para el ángulo de orientación final'
    )
    
    v_max_arg = DeclareLaunchArgument(
        'v_max',
        default_value='0.22',
        description='Velocidad lineal máxima (m/s)'
    )
    
    w_max_arg = DeclareLaunchArgument(
        'w_max',
        default_value='2.84',
        description='Velocidad angular máxima (rad/s)'
    )
    
    epsilon_tol_arg = DeclareLaunchArgument(
        'epsilon_tol',
        default_value='0.05',
        description='Tolerancia para considerar que se alcanzó el objetivo (m)'
    )
    
    epsilon_theta_arg = DeclareLaunchArgument(
        'epsilon_theta',
        default_value='0.1',
        description='Tolerancia angular para orientación (rad)'
    )
    
    obstacle_distance_arg = DeclareLaunchArgument(
        'obstacle_distance',
        default_value='0.5',
        description='Distancia mínima para detectar obstáculos (m)'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='10.0',
        description='Frecuencia de ejecución del controlador (Hz)'
    )
    
    generate_obstacles_arg = DeclareLaunchArgument(
        'generate_obstacles',
        default_value='true',
        description='Generar obstáculos en la simulación'
    )
    
    obstacle_pattern_arg = DeclareLaunchArgument(
        'obstacle_pattern',
        default_value='mixed',
        description='Patrón de obstáculos (random, line, circle, mixed)'
    )
    
    # Incluir el lanzamiento del sistema completo
    diffbot_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                diffbot_bringup_dir,
                'launch',
                'diffbot_complete_system.launch.py'
            ])
        ]),
        launch_arguments={
            'generate_obstacles': LaunchConfiguration('generate_obstacles'),
            'obstacle_pattern': LaunchConfiguration('obstacle_pattern'),
            'rviz_config': 'navigation.rviz'
        }.items()
    )
    
    # Nodo del controlador Bug2
    bug2_controller_node = Node(
        package='diffbot_control',
        executable='bug2_controller',
        name='bug2_controller',
        output='screen',
        parameters=[
            {
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'k_rho': LaunchConfiguration('k_rho'),
                'k_alpha': LaunchConfiguration('k_alpha'),
                'k_beta': LaunchConfiguration('k_beta'),
                'v_max': LaunchConfiguration('v_max'),
                'w_max': LaunchConfiguration('w_max'),
                'epsilon_tol': LaunchConfiguration('epsilon_tol'),
                'epsilon_theta': LaunchConfiguration('epsilon_theta'),
                'obstacle_distance': LaunchConfiguration('obstacle_distance'),
                'control_frequency': LaunchConfiguration('control_frequency'),
            }
        ]
    )
    
    return LaunchDescription([
        # Argumentos
        goal_x_arg,
        goal_y_arg,
        k_rho_arg,
        k_alpha_arg,
        k_beta_arg,
        v_max_arg,
        w_max_arg,
        epsilon_tol_arg,
        epsilon_theta_arg,
        obstacle_distance_arg,
        control_frequency_arg,
        generate_obstacles_arg,
        obstacle_pattern_arg,
        
        # Nodos
        diffbot_system_launch,
        bug2_controller_node,
    ])