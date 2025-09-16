from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    """
    Launch file para el sistema de detección de líneas con cámara.
    Incluye:
    - Gazebo con el robot y la cámara
    - Nodo detector de líneas
    """
    
    # Incluir el launch de Gazebo con cámara
    gazebo_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("diffbot_gz"), "launch", "gzcamera.launch.py"]
            )
        )
    )
    
    # Nodo detector de líneas
    line_detector_node = Node(
        package="diffbot_control",
        executable="line_detector",
        name="line_detector",
        output="screen",
        parameters=[
            # Aquí se pueden agregar parámetros si es necesario
        ]
    )
    
    return LaunchDescription([
        gazebo_camera_launch,
        line_detector_node,
    ])