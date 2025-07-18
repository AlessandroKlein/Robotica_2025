# /gz_robot/launch/robot_system.launch.py


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
#   Archivo: robot_system.launch.py
#   Resumen:
#   Este archivo lanza todos los nodos necesarios para simular y controlar el robot diffbot en Gazebo y ROS 2:
#     - Publica la descripción del robot (URDF generado desde Xacro)
#     - Lanza Gazebo (ros_gz_sim)
#     - Spawnea la entidad en Gazebo
#     - Puente de reloj entre Gazebo y ROS
#     - Spawners de controladores (con temporización)
#     - Nodo de cinemática inversa (cmd_vel_listener)
#     - Nodo de odometría (odometry_publisher)
#     - NUEVO: Nodo de controlador de pose (go_to_pose_controller)
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
        description="Separación entre las ruedas del robot",
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

    # Parámetros para el nuevo controlador de pose
    declare_linear_tolerance = DeclareLaunchArgument(
        "linear_tolerance",
        default_value="0.1",
        description="Tolerancia lineal para alcanzar la pose objetivo (m)",
    )
    declare_angular_tolerance = DeclareLaunchArgument(
        "angular_tolerance",
        default_value="0.1",
        description="Tolerancia angular para alcanzar la pose objetivo (rad)",
    )
    declare_max_linear_speed = DeclareLaunchArgument(
        "max_linear_speed",
        default_value="0.5",
        description="Velocidad lineal máxima del controlador de pose (m/s)",
    )
    declare_max_angular_speed = DeclareLaunchArgument(
        "max_angular_speed",
        default_value="0.5",
        description="Velocidad angular máxima del controlador de pose (rad/s)",
    )

    # -----------------------------------------------------
    # 2. Configuración de variables de lanzamiento (LaunchConfiguration)
    # -----------------------------------------------------
    name = LaunchConfiguration("name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    left_wheel_joint_name = LaunchConfiguration("left_wheel_joint_name")
    right_wheel_joint_name = LaunchConfiguration("right_wheel_joint_name")

    linear_tolerance = LaunchConfiguration("linear_tolerance")
    angular_tolerance = LaunchConfiguration("angular_tolerance")
    max_linear_speed = LaunchConfiguration("max_linear_speed")
    max_angular_speed = LaunchConfiguration("max_angular_speed")

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
                        FindPackageShare("robot_description"),
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
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )

    # -----------------------------------------------------
    # 5. Lanzar Gazebo (ros_gz_sim) con un mundo vacío
    # -----------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf",  # Mundo vacío
            "on_exit_shutdown": "True",  # Apagar ROS si se cierra Gazebo
        }.items(),
    )

    # -----------------------------------------------------
    # 6. Nodo para spawnear la entidad del robot en Gazebo
    # -----------------------------------------------------
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-entity", name, "-topic", "robot_description"],
        output="screen",
    )

    # -----------------------------------------------------
    # 7. Puente de reloj entre Gazebo y ROS (sincroniza el tiempo)
    # -----------------------------------------------------
    clock_bridge = RosNode(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # -----------------------------------------------------
    # 8. Spawners de controladores (con temporización para evitar conflictos)
    # -----------------------------------------------------
    # Los controladores se definen en control_robot/config/controllers.yaml
    # El JointStateBroadcaster debe cargarse primero
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,  # espera 5s antes de lanzarlo
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        ],
    )

    velocity_controller_l_spawner = TimerAction(
        period=7.0,  # 2s después del anterior
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["velocity_controller_left"],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        ],
    )

    velocity_controller_r_spawner = TimerAction(
        period=9.0,  # 2s después del anterior
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["velocity_controller_right"],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        ],
    )

    # -----------------------------------------------------
    # 9. Nodo de cinemática inversa (cmd_vel_listener)
    #     Definido en control_robot/control_robot/cmd_vel_listener.py
    # -----------------------------------------------------
    cmd_vel_listener_node = TimerAction(
        period=11.0,  # Lanzar después de los controladores de velocidad
        actions=[
            Node(
                package="control_robot",
                executable="cmd_vel_listener",
                name="cmd_vel_listener",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"wheel_radius": wheel_radius},
                    {"wheel_separation": wheel_separation},
                ],
                output="screen",
            )
        ],
    )

    # -----------------------------------------------------
    # 10. Nodo de odometría (odometry_publisher)
    #     Definido en control_robot/control_robot/odometry_publisher.py
    # -----------------------------------------------------
    odometry_publisher_node = TimerAction(
        period=13.0,  # Lanzar después de que joint_state_broadcaster esté activo
        actions=[
            Node(
                package="control_robot",
                executable="odometry_publisher",
                name="odometry_publisher",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"wheel_radius": wheel_radius},
                    {"wheel_separation": wheel_separation},
                    {"left_wheel_joint_name": left_wheel_joint_name},
                    {"right_wheel_joint_name": right_wheel_joint_name},
                ],
                output="screen",
            )
        ],
    )

    # -----------------------------------------------------
    # 11. Nodo del controlador de pose (go_to_pose_controller)
    #     Definido en control_robot/control_robot/go_to_pose_controller.py
    # -----------------------------------------------------
    go_to_pose_controller_node = TimerAction(
        period=15.0,  # Lanzar después de que odometry_publisher esté activo
        actions=[
            Node(
                package="control_robot",
                executable="go_to_pose_controller",
                name="go_to_pose_controller",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"linear_tolerance": linear_tolerance},
                    {"angular_tolerance": angular_tolerance},
                    {"max_linear_speed": max_linear_speed},
                    {"max_angular_speed": max_angular_speed},
                ],
                output="screen",
            )
        ],
    )

    # -----------------------------------------------------
    # 12. Retornar la descripción del lanzamiento, incluyendo todos los nodos y argumentos
    # -----------------------------------------------------
    return LaunchDescription(
        [
            declare_robot_name,  # Argumento: nombre de la entidad
            declare_use_sim_time,  # Argumento: usar tiempo simulado
            declare_wheel_radius,  # Argumento: radio de rueda
            declare_wheel_separation,  # Argumento: separación de ruedas
            declare_left_wheel_joint_name,  # Argumento: nombre de la junta izquierda
            declare_right_wheel_joint_name,  # Argumento: nombre de la junta derecha
            declare_linear_tolerance,  # NUEVO Argumento: tolerancia lineal
            declare_angular_tolerance,  # NUEVO Argumento: tolerancia angular
            declare_max_linear_speed,  # NUEVO Argumento: velocidad lineal máxima
            declare_max_angular_speed,  # NUEVO Argumento: velocidad angular máxima
            robot_state_publisher,  # Nodo: publica la descripción del robot
            gz_sim,  # Nodo: lanza Gazebo
            spawn_entity,  # Nodo: spawnea el robot en Gazebo
            clock_bridge,  # Nodo: puente de reloj
            joint_state_broadcaster_spawner,  # Nodo: spawner de joint_state_broadcaster
            velocity_controller_l_spawner,  # Nodo: spawner de controlador izquierdo
            velocity_controller_r_spawner,  # Nodo: spawner de controlador derecho
            cmd_vel_listener_node,  # Nodo: cinemática inversa
            odometry_publisher_node,  # Nodo: odometría
            go_to_pose_controller_node,  # NUEVO: Nodo: controlador de pose
        ]
    )
