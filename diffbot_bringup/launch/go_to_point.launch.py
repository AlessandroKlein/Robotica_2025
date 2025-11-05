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
    pkg_diffbot_gz = get_package_share_directory('diffbot_gz')
    
    # Argumentos de lanzamiento
    goal_x = LaunchConfiguration('goal_x', default='1.0')
    goal_y = LaunchConfiguration('goal_y', default='1.0')
    k_rho = LaunchConfiguration('k_rho', default='0.5')
    k_alpha = LaunchConfiguration('k_alpha', default='1.0')
    k_beta = LaunchConfiguration('k_beta', default='-0.1')
    v_max = LaunchConfiguration('v_max', default='0.5')
    w_max = LaunchConfiguration('w_max', default='1.0')
    epsilon_tol = LaunchConfiguration('epsilon_tol', default='0.05')
    control_frequency = LaunchConfiguration('control_frequency', default='10.0')
    
    # Declarar argumentos
    declare_goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='1.0',
        description='Coordenada X del punto objetivo'
    )
    
    declare_goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='1.0',
        description='Coordenada Y del punto objetivo'
    )
    
    declare_k_rho_arg = DeclareLaunchArgument(
        'k_rho',
        default_value='0.5',
        description='Ganancia para la distancia (k_rho)'
    )
    
    declare_k_alpha_arg = DeclareLaunchArgument(
        'k_alpha',
        default_value='1.0',
        description='Ganancia para el ángulo alpha (k_alpha)'
    )
    
    declare_k_beta_arg = DeclareLaunchArgument(
        'k_beta',
        default_value='-0.1',
        description='Ganancia para el ángulo beta (k_beta)'
    )
    
    # Incluir el launch file del sistema completo
    diffbot_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_diffbot_bringup, 'launch', 'diffbot_complete_system.launch.py')
        )
    )
    
    # Nodo point_follower
    point_follower_node = Node(
        package='diffbot_control',
        executable='point_follower',
        name='point_follower',
        output='screen',
        parameters=[{
            'goal_x': goal_x,
            'goal_y': goal_y,
            'k_rho': k_rho,
            'k_alpha': k_alpha,
            'k_beta': k_beta,
            'v_max': v_max,
            'w_max': w_max,
            'epsilon_tol': epsilon_tol,
            'control_frequency': control_frequency
        }]
    )
    
    return LaunchDescription([
        declare_goal_x_arg,
        declare_goal_y_arg,
        declare_k_rho_arg,
        declare_k_alpha_arg,
        declare_k_beta_arg,
        DeclareLaunchArgument('v_max', default_value='0.5', description='Velocidad lineal máxima'),
        DeclareLaunchArgument('w_max', default_value='1.0', description='Velocidad angular máxima'),
        DeclareLaunchArgument('epsilon_tol', default_value='0.05', description='Tolerancia para considerar alcanzado el objetivo'),
        DeclareLaunchArgument('control_frequency', default_value='10.0', description='Frecuencia de control en Hz'),
        diffbot_system_launch,
        point_follower_node
    ])