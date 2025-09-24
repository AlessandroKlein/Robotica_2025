from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node as RosNode
from launch.actions import TimerAction

# =========================================================
#   Archivo: gzcamera.launch.py
#   Resumen:
#   Este archivo lanza todos los nodos necesarios para simular el robot diffbot 
#   en Gazebo con cámara:
#     - Publica la descripción del robot (URDF generado desde Xacro)
#     - Lanza Gazebo con mundo vacío
#     - Spawnea la entidad en Gazebo
#     - Puente de reloj entre Gazebo y ROS
#     - Puente para la cámara
#     - Spawners de controladores (con temporización)
# =========================================================

def generate_launch_description():
    """
    Lanza Gazebo con el robot diffbot y la cámara.
    """

    # Incluir el archivo description.launch.py del paquete diffbot_description
    # con testing=False para no lanzar joint_state_publisher_gui ni RViz
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("diffbot_description"), "launch", "description.launch.py"]
            )
        ),
        launch_arguments={
            "testing": "False",  # No lanzar joint_state_publisher_gui ni RViz
        }.items(),
    )

    # Launch Gazebo with the custom sensor test world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": ["-r ", PathJoinSubstitution([
                FindPackageShare("diffbot_gz"),
                "worlds",
                "sensor_test_world.sdf"
            ])],
        }.items(),
    )

    # Spawnar el robot en Gazebo usando spawn_entity.py
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-entity", "diffbot",
            "-topic", "robot_description",
            "-z", "0.05",   # Al menos media rueda por encima del suelo
        ],
        output="screen",
    )

    # Bridge manual para /clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Bridge para la cámara usando archivo de configuración
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": PathJoinSubstitution(
                [FindPackageShare("diffbot_gz"), "config", "gz_bridge.yaml"]
            ),
        }],
        output="screen",
    )

    # Controller Manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("diffbot_control"), "config", "controllers.yaml"]
            ),
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # Spawner para el controlador de velocidad (con delay)
    velocity_controller_spawner = TimerAction(
        period=3.0,  # Esperar 3 segundos
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["velocity_controller"],
                output="screen",
            )
        ]
    )

    # Spawner para el controlador de estado de articulaciones (con delay)
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,  # Esperar 2 segundos
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            )
        ]
    )

    return LaunchDescription([
        robot_description_launch,
        gz_sim,
        spawn_entity,
        clock_bridge,
        camera_bridge,
        controller_manager,
        joint_state_broadcaster_spawner,
        velocity_controller_spawner,
    ])