# tp1_robot_gz/launch/gazebo.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Parámetro para ajustar la posición inicial del robot (opcional)
    world_name = LaunchConfiguration('world_name')
    declare_world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty.world',
        description='Nombre del mundo de Gazebo'
    )

    # Ruta al paquete principal (tp1_robot)
    pkg_tp1_robot = get_package_share_directory('tp1_robot')
    
    # Procesar XACRO directamente en este launch file
    xacro_file = os.path.join(pkg_tp1_robot, 'urdf', 'diffbot.xacro')
    robot_description_raw = xacro.process_file(xacro_file)
    robot_description_str = robot_description_raw.toxml()

    # Incluir el launch file de descripción del robot (sin GUI)
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('tp1_robot'),
                'launch',
                'description.launch.py'
            ])
        ),
        launch_arguments={'testing': 'false'}.items()
    )

    # Iniciar Gazebo con un mundo vacío (o el especificado)
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--world', world_name],
        output='screen'
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Nodo para publicar el estado del robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_str
        }]
    )

    # Cargar el modelo URDF en Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'diffbot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_world_name_arg,
        description_launch,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_entity
    ])