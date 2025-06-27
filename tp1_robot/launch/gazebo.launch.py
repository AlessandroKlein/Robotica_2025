# =========================================================
#  Archivo: gazebo.launch.py
#  Resumen:
#  Este archivo lanza la simulación de diffbot en Gazebo con ROS 2.
#  Incluye:
#    - Lanzamiento de Gazebo
#    - Publicación de la descripción del robot (URDF)
#    - Creación de la entidad en el simulador
#    - Bridges para comunicación ROS 2 <-> Gazebo
#    - Lanzamiento secuencial de controladores (joint_state y diff_drive)
# =========================================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  # Usar tiempo de simulación
    pkg_share = FindPackageShare('tp1_robot')  # Buscar el paquete tp1_robot

    # Cargar el URDF usando xacro
    urdf_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'diffbot.xacro'  # Ruta al archivo URDF/Xacro
    ])

    robot_description_content = Command(['xacro ', urdf_file])  # Procesar el xacro para obtener el URDF
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'use_sim_time': use_sim_time
    }

    # Lanzar Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'  # Lanzador de Gazebo
            ])
        ),
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf',  # -v 4: nivel de detalle, empty.sdf: mundo vacío
            'use_sim_time': use_sim_time
        }.items(),
    )

    # Publicar descripción del robot
    robot_state_publisher = Node(
        package='robot_state_publisher',  # Nodo para publicar el estado del robot
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Crear entidad en Gazebo
    create = Node(
        package='ros_gz_sim',  # Nodo para crear la entidad en Gazebo
        executable='create',
        arguments=['-entity', 'diffbot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen',
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',  # Nodo para el puente ROS 2 <-> Gazebo
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',  # Velocidades
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',    # Odometría
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',       # Transformaciones
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',  # Estados articulares
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'     # Reloj de simulación
        ],
        output='screen',
        respawn=True,
        respawn_delay=2,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Controlador joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',  # Nodo para lanzar el controlador de estados articulares
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file', PathJoinSubstitution([
                FindPackageShare('tp1_robot'),
                'config',
                'joint_state_broadcaster.yaml'
            ]),
        ],
    )

    # Controlador de movimiento diferencial (con retardo)
    diff_drive_controller_spawner = Node(
        package='controller_manager',  # Nodo para lanzar el controlador diferencial
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            PathJoinSubstitution([
                FindPackageShare('tp1_robot'),
                'config',
                'diff_drive_controller.yaml'
            ]),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'  # Argumento para usar tiempo de simulación
        ),

        # Lanzamientos iniciales
        gazebo,  # Lanzar Gazebo
        robot_state_publisher,  # Publicar estado del robot
        create,  # Crear entidad en Gazebo
        bridge,  # Puente ROS 2 <-> Gazebo

        # Secuencia de controladores
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=create,
                on_exit=[joint_state_broadcaster_spawner],  # Lanzar joint_state_broadcaster al terminar create
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    TimerAction(
                        period=2.0,
                        actions=[diff_drive_controller_spawner]  # Lanzar diff_drive_controller tras 2s
                    )
                ],
            )
        )
    ])