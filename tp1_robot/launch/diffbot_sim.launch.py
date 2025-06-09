from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = FindPackageShare('tp1_robot')

    urdf_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'diffbot.xacro'
    ])

    robot_description = {
        'robot_description': urdf_file
    }

    # Lanzar Gazebo con mundo vacío
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'on_exit_shutdown': 'True'
        }.items(),
    )

    # Publicar robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Crear entidad en Gazebo
    create = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-entity', 'diffbot', '-topic', 'robot_description'],
        output='screen',
    )

    # Iniciar controller_manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare('tp1_robot'),
                'config',
                'diffbot_controllers.yaml'
            ])
        ],
        output='screen'
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diffbot_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diffbot_base_controller'],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        create,

        # Esperar a que Gazebo esté listo antes de iniciar el controller_manager
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=create,
                on_start=[controller_manager],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[joint_state_broadcaster_spawner],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=joint_state_broadcaster_spawner,
                on_start=[diffbot_base_controller_spawner],
            )
        ),
    ])