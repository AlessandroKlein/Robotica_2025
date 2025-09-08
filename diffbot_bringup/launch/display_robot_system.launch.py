from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# =========================================================
#   Archivo: display_robot_system.launch.py
#   Ejercicio 12: Demostración del Sistema de Visualización
#   
#   Este archivo extiende el ejercicio 10 (gazebo.launch.py) y añade:
#   - Controladores del robot
#   - Nodo de odometría con transformaciones tf2
#   - RViz2 para visualizar el robot en el frame 'odom'
# =========================================================

def generate_launch_description():
    # Launch simulation (ejercicio 10)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('diffbot_gz'), 'launch', 'gazebo.launch.py']
            )
        )
    )

    # Controller node (ejercicio 8)
    node_diffbot_controller = Node(
        package='diffbot_control',
        executable='cmd_vel_listener',
        output='screen'
    )
    
    # Odometry node with tf2 transformations (ejercicio 11)
    node_diffbot_odometry = Node(
        package='diffbot_control',
        executable='odometry_publisher',
        output='screen',
        parameters=[{'publish_tf': True}]
    )
    
    # RViz2 node with odom configuration
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution(
                [FindPackageShare('diffbot_bringup'), 'rviz', 'odom.rviz']
            )
        ]
    )

    return LaunchDescription([
        simulation,
        node_diffbot_controller,
        node_diffbot_odometry,
        node_rviz
    ])