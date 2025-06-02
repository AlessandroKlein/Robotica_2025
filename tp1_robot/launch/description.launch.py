from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Parámetro para activar/desactivar herramientas de prueba
    declare_testing_arg = DeclareLaunchArgument(
        'testing',
        default_value='true',
        description='Activa joint_state_publisher_gui y RViz si es true'
    )

    # Ruta al archivo XACRO
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('tp1_robot'),
            'urdf',
            'diffbot.xacro'
        ])
    ])

    # Nodo robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
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
        condition=IfCondition(LaunchConfiguration('testing'))  # Uso de IfCondition
    )

    return LaunchDescription([
        declare_testing_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])