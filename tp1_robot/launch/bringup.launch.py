# =========================================================
#  Archivo: bringup.launch.py
#  Resumen:
#  Este archivo lanza todo el sistema del robot diffbot:
#    - Incluye la descripción del robot
#    - Lanza la simulación en Gazebo
#    - Lanza los controladores y spawners
#    - Lanza el nodo de odometría
#    - Lanza RViz (opcional)
# =========================================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')  # Variable para decidir si lanzar RViz

    DeclareLaunchArgument(name='use_rviz', default_value='true', description='Usar RViz'),  # Argumento para activar/desactivar RViz
    
    # Incluir description.launch
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('tp1_robot_description'), '/launch/', 'description.launch.py'  # Incluye la descripción del robot
        ]),
        launch_arguments={'testing': 'true'}.items()
    )

    # Incluir gazebo.launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('tp1_robot_gz'), '/launch/', 'gazebo.launch.py'  # Incluye el lanzamiento de Gazebo
        ])
    )

    # Cargar controladores
    controller_manager = Node(
        package='controller_manager',  # Nodo principal de control ros2_control
        executable='ros2_control_node',
        parameters=[PathJoinSubstitution([FindPackageShare('tp1_robot_control'), 'config', 'diffbot_controllers.yaml'])],  # Configuración de controladores
        output='screen'
    )

    # Spawners
    spawner_jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])  # Spawner para joint_state_broadcaster
    spawner_diff_drive = Node(package='controller_manager', executable='spawner', arguments=['diff_drive_base_controller'])  # Spawner para el controlador diferencial

    # Nodo de odometría
    odometry_node = Node(
        package='tp1_robot',  # Nodo de odometría personalizado
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )

    # RViz
    rviz_config = PathJoinSubstitution([FindPackageShare('tp1_robot'), 'diffbot.rviz'])  # Ruta al archivo de configuración de RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),  # Solo lanzar si use_rviz es true
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config], output='screen'
    )

    return LaunchDescription([
        description_launch,  # Lanzar descripción del robot
        gazebo_launch,  # Lanzar simulador Gazebo
        controller_manager,  # Lanzar ros2_control_node
        spawner_jsb,  # Lanzar spawner de joint_state_broadcaster
        spawner_diff_drive,  # Lanzar spawner de diff_drive_base_controller
        odometry_node,  # Lanzar nodo de odometría
        rviz_node  # Lanzar RViz (opcional)
    ])