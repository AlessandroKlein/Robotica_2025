from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Lanza todos los nodos necesarios para simular y controlar el robot diffbot en Gazebo y ROS 2:
    - Simulación de Gazebo (usando gazebo.launch.py existente)
    - Controladores de ROS2 control
    - Nodos de los ejercicios 8 y 9
    """

    # Incluir el archivo gazebo.launch.py del paquete diffbot_gz
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("diffbot_gz"), "launch", "gazebo.launch.py"]
            )
        ),
    )

    # Controller spawners
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    velocity_controller_left_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['velocity_controller_left'],
                output='screen'
            )
        ]
    )

    velocity_controller_right_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['velocity_controller_right'],
                output='screen'
            )
        ]
    )

    # Exercise 8 node: cmd_vel_listener
    node_diffbot_controller = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='diffbot_control',
                executable='cmd_vel_listener',
                output='screen'
            )
        ]
    )

    # Exercise 9 node: odometry_publisher
    node_diffbot_odometry = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='diffbot_control',
                executable='odometry_publisher',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        simulation,
        joint_state_broadcaster_spawner,
        velocity_controller_left_spawner,
        velocity_controller_right_spawner,
        node_diffbot_controller,
        node_diffbot_odometry,
    ])
