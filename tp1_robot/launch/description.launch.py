import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Parámetro para activar/desactivar herramientas de prueba
    declare_testing_arg = DeclareLaunchArgument(
        'testing',
        default_value='true',
        description='Activa joint_state_publisher_gui y RViz si es true'
    )

    # Ruta absoluta al archivo XACRO
    xacro_file = os.path.join(
        get_package_share_directory('tp1_robot'),
        'urdf',
        'diffbot.xacro'
    )

    # Procesar el archivo XACRO
    robot_description_raw = xacro.process_file(xacro_file)
    robot_description_str = robot_description_raw.toxml()

    # Nodo robot_state_publisher con robot_description como string
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_str, value_type=str)
        }],
    )

    # Nodo joint_state_publisher_gui (condicional)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('testing')),
    )

    # Nodo RViz (condicional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('tp1_robot'),
            'rviz',
            'diffbot.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('testing'))
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_velocity_controller", "right_velocity_controller"],
    )

    return LaunchDescription([
        declare_testing_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
