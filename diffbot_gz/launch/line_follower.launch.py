#!/usr/bin/env python3
"""
Launch file para el sistema de seguimiento de líneas (Line Follower)

Este archivo configura:
- Gazebo con el modelo LineTrack
- Robot diffbot con sensor de cámara
- Bridge ROS-Gazebo para comunicación
- Motor de renderizado ogre para mejor rendimiento

Autor: Sistema de seguimiento de líneas
Fecha: 2025
"""

import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Configurar la ruta de recursos de Gazebo para incluir los modelos
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([FindPackageShare("diffbot_gz"), "models"]),
    )
    
    # Lanzar Gazebo con el motor de renderizado ogre y mundo vacío
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            ),
        ),
        launch_arguments={
            'gz_args': '-r --render-engine ogre empty.sdf',
        }.items()
    )
    
    # Eliminar el suelo por defecto de Gazebo
    remove_ground_plane = Node(
        package="ros_gz_sim",
        executable="remove",
        parameters=[
            {'entity_name': 'ground_plane'},
        ],
        output="screen",
    )
    
    # Cargar el modelo LineTrack (pista de seguimiento de líneas)
    load_track = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-entity", "track",
            "-file", "model://LineTrack",
        ],
        output="screen",
    )
    
    # Incluir el launch del robot diffbot
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('diffbot_description'),
                'launch',
                'description.launch.py'
            ])
        ])
    )
    
    # Spawner del robot en Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'diffbot',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # Bridge ROS-Gazebo para comunicación de sensores
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": PathJoinSubstitution(
                [FindPackageShare("diffbot_gz"), "config", "gz_bridge.yaml"]
            ),
            "use_sim_time": True,
        }],
        output="screen"
    )
    
    # Bridge para el reloj de simulación
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )
    
    # Controladores necesarios para el movimiento del robot
    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster','--use-sim-time'],
        output='screen'
    )

    velocity_controller_l = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller_l','--use-sim-time'],
        output='screen'
    )

    velocity_controller_r = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller_r','--use-sim-time'],
        output='screen'
    )

    diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller','--use-sim-time'],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        remove_ground_plane,
        load_track,
        robot_description,
        spawn_robot,
        ros_gz_bridge,
        clock_bridge,
        joint_state_broadcaster,
        velocity_controller_l,
        velocity_controller_r,
        diff_drive_controller,
    ])