#!/usr/bin/env python3
"""
Sistema de Lanzamiento para Robot DiffBot - Clase 13

Este archivo de lanzamiento integra los componentes necesarios para el
ejercicio de la Clase 13, incluyendo:
- Simulación en Gazebo
- Controladores de movimiento
- Sensores IMU y LiDAR
- Detector de obstáculos con LiDAR
- Obstáculos configurables para entorno desafiante
- Herramientas de visualización

Autor: Sistema de Integración DiffBot
Fecha: 2025
Versión: 1.1 - Clase 13 (IMU, LiDAR y Obstáculos)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable


def generate_launch_description():
    """
    Genera la descripción del lanzamiento del sistema DiffBot para Clase 13.
    
    Configura y lanza los nodos necesarios para el ejercicio de sensores
    IMU y LiDAR, incluyendo simulación, control, obstáculos configurables
    y procesamiento de obstáculos.
    
    Returns:
        LaunchDescription: Descripción del sistema a lanzar
    """
    # Argumento para activar herramientas de desarrollo y prueba
    declare_testing_arg = DeclareLaunchArgument(
        'testing',
        default_value='true',
        description='Activa herramientas de desarrollo: joint_state_publisher_gui y RViz'
    )
    testing = LaunchConfiguration('testing')
    
    # Argumentos para configurar obstáculos
    declare_obstacles_arg = DeclareLaunchArgument(
        'enable_obstacles',
        default_value='true',
        description='Activa la generación de obstáculos en el entorno'
    )
    
    declare_obstacle_count_arg = DeclareLaunchArgument(
        'obstacle_count',
        default_value='5',
        description='Número de obstáculos a generar (1-10)'
    )
    
    declare_obstacle_size_arg = DeclareLaunchArgument(
        'obstacle_size',
        default_value='medium',
        description='Tamaño de obstáculos: small, medium, large, mixed'
    )
    
    declare_obstacle_pattern_arg = DeclareLaunchArgument(
        'obstacle_pattern',
        default_value='strategic',
        description='Patrón de colocación: random, strategic, corridor, maze'
    )
    
    # Configuraciones de obstáculos
    enable_obstacles = LaunchConfiguration('enable_obstacles')
    obstacle_count = LaunchConfiguration('obstacle_count')
    obstacle_size = LaunchConfiguration('obstacle_size')
    obstacle_pattern = LaunchConfiguration('obstacle_pattern')
 
    # Nodo opcional: Interfaz gráfica para control manual de articulaciones
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(testing),
        output='screen'
    )

    # Puente de comunicación entre Gazebo y ROS2 para sensores IMU y LiDAR
    # Configura la comunicación bidireccional para IMU, LiDAR y comandos de control
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([
                FindPackageShare('diffbot_control'),
                'config',
                'gz_bridge.yaml' ]),
            'use_sim_time': True,
        }],
        output='screen'
    )
    
    # Nodo opcional: RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('diffbot_description'),
            'rviz',
            'diffbot.rviz'
        ])],
        condition=IfCondition(testing),
        parameters=[{'use_sim_time': True}]
    )

    # Incluir launch de Gazebo + spawn + controladores
    sim_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('diffbot_gz'),
                'launch',
                'gzbo2.launch.py'
            ])
        )
    )

    # Controlador de tracción diferencial
    # Convierte comandos de velocidad Twist en velocidades angulares específicas para cada rueda
    nodo_control = Node(
        package='diffbot_control',
        executable='differential_drive_controller',
        name='diff_drive_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Calculador de odometría del robot
    # Estima la posición y orientación del robot basándose en las posiciones de las ruedas
    nodo_odometria = Node(
        package='diffbot_control',
        executable='odometry_calculator',
        name='diffbot_odometry_node',
        output='screen',
        parameters=[
            {'wheel_r': 0.035},          # Radio de las ruedas (m)
            {'wheel_sep': 0.135},        # Separación entre ruedas (m)
            {'left_wheel_joint': 'left_wheel_joint'},   # Nombre del joint izquierdo
            {'right_wheel_joint': 'right_wheel_joint'}, # Nombre del joint derecho
            {'publish_tf': True},        # Publicar transformaciones TF
            {'use_sim_time': True},      # Usar tiempo de simulación
        ]
    )
    
    # Detector de obstáculos basado en LiDAR
    # Analiza datos del sensor láser para detectar obstáculos en zonas específicas
    lidar_detector_node = Node(
        package='diffbot_control',
        executable='detector_lidar',
        name='lidar_detector',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'distance_threshold': 0.5},  # Distancia mínima de detección (m)
            {'zone_angle': 30.0}          # Ángulo de cada zona de detección (grados)
        ]
    )
    
    # Generador de obstáculos configurables
    # Crea obstáculos dinámicos en Gazebo según los parámetros especificados
    obstacle_spawner_node = Node(
        package='diffbot_control',
        executable='obstacle_spawner',
        name='obstacle_spawner',
        output='screen',
        condition=IfCondition(enable_obstacles),
        parameters=[
            {'use_sim_time': True},
            {'obstacle_count': obstacle_count},
            {'obstacle_size': obstacle_size},
            {'obstacle_pattern': obstacle_pattern},
            {'enable_obstacles': enable_obstacles}
        ]
    )

    return LaunchDescription([
        declare_testing_arg,
        declare_obstacles_arg,
        declare_obstacle_count_arg,
        declare_obstacle_size_arg,
        declare_obstacle_pattern_arg,
        joint_state_publisher_gui,
        rviz,
        sim_stack,
        bridge_node,
        nodo_control,
        nodo_odometria,
        lidar_detector_node,
        obstacle_spawner_node
    ])