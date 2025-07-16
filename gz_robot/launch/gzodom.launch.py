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
from launch.actions import TimerAction
import yaml # Necesario para cargar el archivo YAML
import os   # Necesario para obtener la ruta del paquete

def generate_launch_description():
    """
    Genera la descripción de lanzamiento para simular el robot en Gazebo con ROS 2 Control.
    Este archivo:
    1. Lanza Gazebo (gz_sim).
    2. Publica la descripción del robot (robot_state_publisher).
    3. Spawnea el robot en Gazebo (ros_gz_sim create).
    4. Configura el puente de reloj para usar el tiempo de simulación.
    5. Lanza el nodo del controller_manager de ROS 2 Control.
    6. Carga y activa los controladores de ROS 2 Control utilizando 'ros2 control load_controller'.
    7. Lanza el nodo de odometría para calcular la pose del robot.
    """

    # Declaración de argumentos de lanzamiento
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

    # Obtención de los valores de los argumentos de lanzamiento
    name = LaunchConfiguration("name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Procesamiento del archivo URDF/XACRO del robot.
    # Ahora el URDF incluirá la configuración de ros2_control y gazebo.
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
                " use_sim_time:=", use_sim_time # Pasar use_sim_time al xacro si es necesario para el plugin de Gazebo
            ]
        ),
        value_type=str,
    )

    # Nodo para publicar el estado del robot.
    robot_state_publisher_node = Node(
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

    # Inclusión del lanzamiento de Gazebo (gz_sim).
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf", # Usamos empty.sdf para que Gazebo inicie vacío
            "on_exit_shutdown": "True",
        }.items(),
    )

    # Nodo para spawnear la entidad (robot) en Gazebo.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-entity", name, "-topic", "robot_description", "-x", "0", "-y", "0", "-z", "0.5"], # Añadimos posición inicial
        output="screen",
    )

    # Puente para el reloj entre ROS 2 y Gazebo.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # --- Nodos para ROS 2 Control ---

    # Ruta al archivo de configuración de los controladores
    # Usamos os.path.join y FindPackageShare para construir la ruta absoluta.
    pkg_share_dir = FindPackageShare("control_robot").find("control_robot")
    controllers_config_file_path = os.path.join(pkg_share_dir, "config", "controllers.yaml")

    # Cargar el contenido del archivo YAML como un diccionario
    # Esto evita que el controller_manager intente cargar el archivo con --params-file
    # y en su lugar le pasa los parámetros directamente.
    with open(controllers_config_file_path, 'r') as f:
        controllers_config = yaml.safe_load(f)

    # Nodo del controller_manager
    # IMPORTANTE: Pasamos el diccionario cargado directamente.
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            controllers_config, # ¡PASAMOS EL DICCIONARIO CARGADO!
            {"use_sim_time": use_sim_time}
        ],
        output="screen",
    )

    # --- Nodo de Odometría (Ejercicio 9) ---
    odometry_node = Node(
        package="control_robot",
        executable="odometry_node",
        parameters=[
            {"wheel_radius": 0.05},  # Ajusta según el radio de tu rueda
            {"track_width": 0.3},   # Ajusta según el ancho de vía de tu robot
            {"left_wheel_joint_name": "left_wheel_joint"},
            {"right_wheel_joint_name": "right_wheel_joint"},
            {"odom_frame_id": "odom"},
            {"base_link_frame_id": "base_link"},
            {"use_sim_time": use_sim_time} # Es crucial que el nodo de odometría use el tiempo de simulación
        ],
        output="screen",
    )


    # --- Comandos para cargar y activar los controladores de ROS 2 Control ---
    # Utilizamos TimerAction para asegurar que estos comandos se ejecuten
    # después de que Gazebo, el robot y el controller_manager hayan sido completamente inicializados.

    # Cargar y activar el joint_state_broadcaster
    # Se añade un retraso para asegurar que el controller_manager esté activo.
    load_joint_state_broadcaster = TimerAction(
        period=7.0,  # Retraso para dar tiempo a Gazebo y al robot a inicializarse
        actions=[
            ExecuteProcess(
                cmd=["ros2", "control", "load_controller", "joint_state_broadcaster", "--set-state", "active"],
                output="screen",
                shell=True
            )
        ],
    )

    # Cargar y activar el velocity_controller_left
    load_velocity_controller_left = TimerAction(
        period=9.0,  # Retraso adicional para el siguiente controlador
        actions=[
            ExecuteProcess(
                cmd=["ros2", "control", "load_controller", "velocity_controller_left", "--set-state", "active"],
                output="screen",
                shell=True
            )
        ],
    )

    # Cargar y activar el velocity_controller_right
    load_velocity_controller_right = TimerAction(
        period=11.0,  # Retraso adicional para el último controlador
        actions=[
            ExecuteProcess(
                cmd=["ros2", "control", "load_controller", "velocity_controller_right", "--set-state", "active"],
                output="screen",
                shell=True
            )
        ],
    )

    # Retorna la descripción de lanzamiento con todos los nodos y acciones.
    return LaunchDescription(
        [
            declare_robot_name,
            declare_use_sim_time,
            robot_state_publisher_node,
            gz_sim,
            spawn_entity,
            clock_bridge,
            controller_manager_node,
            odometry_node, # ¡Añadido el nodo de odometría aquí!
            load_joint_state_broadcaster,
            load_velocity_controller_left,
            load_velocity_controller_right,
        ]
    )
