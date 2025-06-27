# =========================================================
#  Archivo: description.launch.py
#  Resumen:
#  Este archivo lanza los nodos necesarios para visualizar y manipular el modelo del robot diffbot:
#    - Publica la descripción del robot (URDF generado desde Xacro)
#    - Publica los estados articulares (joint_state)
#    - Permite manipular las articulaciones con GUI
#    - Lanza RViz con la configuración específica
# =========================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('tp1_robot')  # Buscar el paquete tp1_robot
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'diffbot.xacro'])  # Ruta al archivo Xacro

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file  # Ejecutar xacro para generar el URDF
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',  # Nodo que publica la descripción del robot
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str)  # Parámetro con el URDF generado
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',  # Nodo que publica los estados articulares
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',  # Nodo GUI para manipular articulaciones
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz = Node(
        package='rviz2',  # Nodo para lanzar RViz
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('tp1_robot'),
            'rviz',
            'diffbot.rviz'
        ])],  # Cargar configuración específica de RViz
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,  # Publicar descripción del robot
        joint_state_publisher,  # Publicar estados articulares
        joint_state_publisher_gui,  # GUI para manipular articulaciones
        rviz  # Lanzar RViz
    ])
