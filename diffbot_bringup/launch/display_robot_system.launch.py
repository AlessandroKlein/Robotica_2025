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
    # Launch simulation with controllers (ejercicio 11)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('diffbot_gz'), 'launch', 'gztwist.launch.py']
            )
        )
    )

    # Note: cmd_vel_listener is already included in gzodom.launch.py
    
    # Odometry node with tf2 transformations (ejercicio 11)
    node_diffbot_odometry = Node(
        package='diffbot_control',
        executable='odometry_publisher',
        output='screen',
        parameters=[{'publish_tf': True}]
    )
    
    # Go to pose controller for position control
    node_go_to_pose_controller = Node(
        package='diffbot_control',
        executable='go_to_pose_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'linear_tolerance': 0.05},
            {'angular_tolerance': 0.05},
            {'max_linear_speed': 0.5},
            {'max_angular_speed': 1.0}
        ]
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
        node_diffbot_odometry,
        node_go_to_pose_controller,
        node_rviz
    ])