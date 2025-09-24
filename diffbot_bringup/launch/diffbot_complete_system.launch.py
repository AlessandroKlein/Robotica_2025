#!/usr/bin/env python3
"""
Sistema Completo de Lanzamiento para Robot DiffBot

Este archivo de lanzamiento integra todos los componentes necesarios para el
funcionamiento completo del robot DiffBot, incluyendo:
- Simulación en Gazebo
- Controladores de movimiento
- Sensores (LiDAR, cámara, IMU)
- Nodos de procesamiento (detección de líneas, obstáculos)
- Herramientas de visualización

Autor: Sistema de Integración DiffBot
Fecha: 2025
Versión: 1.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable


def generate_launch_description():
    """
    Genera la descripción completa del lanzamiento del sistema DiffBot.
    
    Configura y lanza todos los nodos necesarios para el funcionamiento completo
    del robot, incluyendo simulación, control, sensores y procesamiento.
    
    Returns:
        LaunchDescription: Descripción completa del sistema a lanzar
    """
    # Argumento para activar herramientas de desarrollo y prueba
    declare_testing_arg = DeclareLaunchArgument(
        'testing',
        default_value='false',
        description='Activa herramientas de desarrollo: joint_state_publisher_gui y RViz'
    )
    testing = LaunchConfiguration('testing')
 
    # Nodo opcional: Interfaz gráfica para control manual de articulaciones
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(testing),
        output='screen'
    )

    # Puente de comunicación entre Gazebo y ROS2 para todos los sensores
    # Configura la comunicación bidireccional para LiDAR, cámara, IMU y comandos
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
    
    # Detector de líneas basado en visión por computadora
    # Procesa imágenes de la cámara para seguimiento de líneas usando filtros HSV
    line_detector_node = Node(
        package='diffbot_control',
        executable='line_detector',
        name='line_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Parámetros HSV para detección de líneas (rango de colores)
            'hsv_lower_h': 0,      # Matiz mínimo
            'hsv_lower_s': 0,      # Saturación mínima
            'hsv_lower_v': 0,      # Valor mínimo
            'hsv_upper_h': 180,    # Matiz máximo
            'hsv_upper_s': 255,    # Saturación máxima
            'hsv_upper_v': 50,     # Valor máximo
            # Parámetros de control
            'linear_speed': 0.2,   # Velocidad lineal base (m/s)
            'angular_gain': 0.5    # Ganancia para corrección angular
        }]
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
    
    # Visualizador de imágenes de la cámara
    # Proporciona interfaz gráfica para monitorear el stream de video en tiempo real
    camera_viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_viewer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_testing_arg,
        joint_state_publisher_gui,
        rviz,
        sim_stack,
        bridge_node,
        nodo_control,
        nodo_odometria,
        line_detector_node,
        lidar_detector_node,
        camera_viewer
    ])