from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    # Parámetro testing
    declare_testing_arg = DeclareLaunchArgument(
        'testing',
        default_value='true',
        description='Activa herramientas GUI si es true'
    )

    # Procesar XACRO
    xacro_file = os.path.join(get_package_share_directory('tp1_robot'), 'urdf', 'diffbot.xacro')
    robot_description_raw = xacro.process_file(xacro_file)
    robot_description_str = robot_description_raw.toxml()

    # Nodo robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description_str, value_type=str)}]
    )

    # Nodo joint_state_publisher_gui (opcional)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('testing'))
    )

    # Nodo RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('tp1_robot'), 'rviz', 'diffbot.rviz'])],
        condition=IfCondition(LaunchConfiguration('testing'))
    )

    automatic_mover = Node(
        package='tp1_robot',
        executable='automatic_mover_node',
        name='automatic_mover'
    )

    return LaunchDescription([
        declare_testing_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
        automatic_mover
    ])