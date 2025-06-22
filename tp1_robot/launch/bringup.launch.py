from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    DeclareLaunchArgument(name='use_rviz', default_value='true', description='Usar RViz'),
    
    # Incluir description.launch
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('tp1_robot_description'), '/launch/', 'description.launch.py'
        ]),
        launch_arguments={'testing': 'true'}.items()
    )

    # Incluir gazebo.launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('tp1_robot_gz'), '/launch/', 'gazebo.launch.py'
        ])
    )

    # Cargar controladores
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[PathJoinSubstitution([FindPackageShare('tp1_robot_control'), 'config', 'diffbot_controllers.yaml'])],
        output='screen'
    )

    # Spawners
    spawner_jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])
    spawner_diff_drive = Node(package='controller_manager', executable='spawner', arguments=['diff_drive_base_controller'])

    # Nodo de odometría
    odometry_node = Node(
        package='tp1_robot',
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )

    # RViz
    rviz_config = PathJoinSubstitution([FindPackageShare('tp1_robot'), 'diffbot.rviz'])
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config], output='screen'
    )

    return LaunchDescription([
        description_launch,
        gazebo_launch,
        controller_manager,
        spawner_jsb,
        spawner_diff_drive,
        odometry_node,
        rviz_node
    ])