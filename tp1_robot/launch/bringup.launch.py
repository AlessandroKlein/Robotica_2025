# tp1_robot/launch/bringup.launch.py

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Parámetros configurables
    declare_testing_arg = DeclareLaunchArgument(
        'testing',
        default_value='true',
        description='Activa RViz y herramientas de prueba si es true'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usa tiempo de simulación (relevante para Gazebo)'
    )

    declare_publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Activa/desactiva la publicación de transformaciones TF'
    )

    # Incluir launch file de descripción del robot (sin GUI)
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

    # Incluir launch file de Gazebo (si existe)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('tp1_robot_gz'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

    # Nodo de teleoperación
    teleop_twist_keyboard = Node(
        package='tp1_robot',
        executable='teleop_twist_keyboard_node',
        name='teleop_twist_keyboard',
        output='screen'
    )

    # Nodo de cinemática inversa
    inverse_kinematics = Node(
        package='tp1_robot',
        executable='inverse_kinematics_node',
        name='inverse_kinematics',
        output='screen'
    )

    # Nodo de odometría
    odometry = Node(
        package='tp1_robot',
        executable='odometry_node',
        name='odometry',
        output='screen'
    )

    # Nodo de transformaciones TF (condicional)
    tf_publisher = Node(
        package='tp1_robot',
        executable='tf_publisher_node',
        name='tf_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_tf'))
    )

    # Nodo de RViz (condicional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('tp1_robot'),
            'rviz',
            'diffbot.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('testing'))
    )

    return LaunchDescription([
        declare_testing_arg,
        declare_use_sim_time_arg,
        declare_publish_tf_arg,
        description_launch,
        gazebo_launch,
        teleop_twist_keyboard,
        inverse_kinematics,
        odometry,
        tf_publisher,
        rviz
    ])