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
#   Archivo: gzodom_calibrated.launch.py
#   Resumen:
#   Este archivo lanza todos los nodos necesarios para simular y controlar el robot diffbot en Gazebo y ROS 2
#   con parámetros de calibración de odometría aplicados:
#     - Publica la descripción del robot (URDF generado desde Xacro)
#     - Lanza Gazebo (ros_gz_sim)
#     - Spawnea la entidad en Gazebo
#     - Puente de reloj entre Gazebo y ROS
#     - Spawners de controladores (con temporización)
#     - Nodo de cinemática inversa (differential_drive_controller) con calibración
#     - Nodo de odometría (odometry_calculator) con calibración
# =========================================================

def generate_launch_description():
    # -----------------------------------------------------
    # 1. Declaración de argumentos de lanzamiento (Launch Arguments)
    # -----------------------------------------------------
    declare_robot_name = DeclareLaunchArgument(
        "name",
        default_value="diffbot",
        description="Nombre de la entidad a spawnear en Gazebo",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Usar tiempo simulado (reloj de Gazebo)",
    )

    # Parámetros para el radio y separación de ruedas (usados por nodos de control y odometría)
    declare_wheel_radius = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.035",  # r = 0.035 m (from diffbot.urdf.xacro)
        description="Radio de las ruedas del robot",
    )

    declare_wheel_separation = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.135",  # b = 0.135 m (from diffbot.urdf.xacro)
        description="Separación nominal entre las ruedas del robot",
    )

    # Parámetros de calibración de odometría
    declare_c_L = DeclareLaunchArgument(
        "c_L",
        default_value="1.0",
        description="Coeficiente de corrección para la rueda izquierda",
    )

    declare_c_R = DeclareLaunchArgument(
        "c_R", 
        default_value="1.0",
        description="Coeficiente de corrección para la rueda derecha",
    )

    declare_b_actual = DeclareLaunchArgument(
        "b_actual",
        default_value="0.135",
        description="Separación corregida entre ruedas (calculada por calibración)",
    )

    # Nombres de las juntas para la odometría
    declare_left_wheel_joint_name = DeclareLaunchArgument(
        "left_wheel_joint_name",
        default_value="left_wheel_joint",
        description="Nombre de la junta de la rueda izquierda",
    )
    declare_right_wheel_joint_name = DeclareLaunchArgument(
        "right_wheel_joint_name",
        default_value="right_wheel_joint",
        description="Nombre de la junta de la rueda derecha",
    )

    # -----------------------------------------------------
    # 2. Configuración de variables de lanzamiento (LaunchConfiguration)
    # -----------------------------------------------------
    name = LaunchConfiguration("name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    c_L = LaunchConfiguration("c_L")
    c_R = LaunchConfiguration("c_R")
    b_actual = LaunchConfiguration("b_actual")
    left_wheel_joint_name = LaunchConfiguration("left_wheel_joint_name")
    right_wheel_joint_name = LaunchConfiguration("right_wheel_joint_name")

    # -----------------------------------------------------
    # 3. Procesar el archivo Xacro para obtener la descripción del robot (URDF)
    # -----------------------------------------------------
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("diffbot_description"),
                        "urdf",
                        "diffbot.urdf.xacro",
                    ]
                ),
            ]
        ),
        value_type=str,
    )

    # -----------------------------------------------------
    # 4. Nodo para publicar la descripción del robot (robot_state_publisher)
    # -----------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
        ],
    )

    # -----------------------------------------------------
    # 5. Lanzar Gazebo (ros_gz_sim)
    # -----------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # -----------------------------------------------------
    # 6. Spawnear la entidad en Gazebo
    # -----------------------------------------------------
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", name,
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
        ],
        output="screen",
    )

    # -----------------------------------------------------
    # 7. Puente de reloj entre Gazebo y ROS
    # -----------------------------------------------------
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # -----------------------------------------------------
    # 8. Spawners de controladores (con temporización para evitar errores de inicialización)
    # -----------------------------------------------------
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ],
    )

    velocity_controller_l_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["velocity_controller_l", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ],
    )

    velocity_controller_r_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["velocity_controller_r", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ],
    )

    # -----------------------------------------------------
    # 9. Nodo de control diferencial calibrado (differential_drive_controller)
    # -----------------------------------------------------
    differential_drive_controller_node = TimerAction(
        period=8.0, # Lanzar después de los controladores de velocidad
        actions=[
            Node(
                package="diffbot_control",
                executable="differential_drive_controller",
                name="differential_drive_controller",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"wheel_radius": wheel_radius},
                    {"wheel_separation": wheel_separation},
                    {"c_L": c_L},
                    {"c_R": c_R},
                ],
                output="screen",
            )
        ],
    )

    # -----------------------------------------------------
    # 10. Nodo de odometría calibrado (odometry_calculator)
    # -----------------------------------------------------
    odometry_calculator_node = TimerAction(
        period=10.0, # Lanzar después de que joint_state_broadcaster esté activo
        actions=[
            Node(
                package="diffbot_control",
                executable="odometry_calculator",
                name="odometry_calculator",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"wheel_r": wheel_radius},
                    {"wheel_sep": wheel_separation},
                    {"b_actual": b_actual},
                    {"left_wheel_joint": left_wheel_joint_name},
                    {"right_wheel_joint": right_wheel_joint_name},
                    {"c_L": c_L},
                    {"c_R": c_R},
                    {"publish_tf": True},
                ],
                output="screen",
            )
        ],
    )

    # -----------------------------------------------------
    # 11. Puente para odometría de Gazebo (ros_gz_bridge)
    # -----------------------------------------------------
    gazebo_odometry_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"config_file": PathJoinSubstitution(
                [FindPackageShare("diffbot_gz"), "config", "gz_bridge.yaml"]
            )}
        ],
        output="screen",
    )

    # -----------------------------------------------------
    # 12. Retornar la descripción del lanzamiento, incluyendo todos los nodos y argumentos
    # -----------------------------------------------------
    return LaunchDescription(
        [
            declare_robot_name,              # Argumento: nombre de la entidad
            declare_use_sim_time,            # Argumento: usar tiempo simulado
            declare_wheel_radius,            # Argumento: radio de rueda
            declare_wheel_separation,        # Argumento: separación nominal de ruedas
            declare_c_L,                     # Argumento: coeficiente calibración rueda izquierda
            declare_c_R,                     # Argumento: coeficiente calibración rueda derecha
            declare_b_actual,                # Argumento: separación corregida de ruedas
            declare_left_wheel_joint_name,   # Argumento: nombre de la junta izquierda
            declare_right_wheel_joint_name,  # Argumento: nombre de la junta derecha
            robot_state_publisher,           # Nodo: publica la descripción del robot
            gz_sim,                         # Nodo: lanza Gazebo
            spawn_entity,                   # Nodo: spawnea el robot en Gazebo
            clock_bridge,                   # Nodo: puente de reloj
            joint_state_broadcaster_spawner,# Nodo: spawner de joint_state_broadcaster
            velocity_controller_l_spawner,  # Nodo: spawner de controlador izquierdo
            velocity_controller_r_spawner,  # Nodo: spawner de controlador derecho
            differential_drive_controller_node, # Nodo: control diferencial calibrado
            odometry_calculator_node,       # Nodo: odometría calibrada
            gazebo_odometry_bridge,         # Nodo: puente odometría Gazebo
        ]
    )