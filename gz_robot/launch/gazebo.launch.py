from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess, # Aunque no se usa directamente para un proceso externo, es útil tenerlo
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
from launch_ros.actions import Node as RosNode # Renombrado para evitar conflicto con launch.actions.Node
from launch.actions import TimerAction


def generate_launch_description():
    """
    Genera la descripción de lanzamiento para simular el robot en Gazebo.
    Este archivo:
    1. Lanza Gazebo (gz_sim).
    2. Publica la descripción del robot (robot_state_publisher).
    3. Spawnea el robot en Gazebo (ros_gz_sim create).
    4. Configura el puente de reloj para usar el tiempo de simulación.
    5. Lanza los controladores del robot con retrasos para evitar condiciones de carrera.
    """

    # Declaración de argumentos de lanzamiento
    # Permite al usuario especificar el nombre del robot al lanzar el archivo.
    declare_robot_name = DeclareLaunchArgument(
        "name",
        default_value="diffbot",
        description="Nombre de la entidad a spawnear en Gazebo",
    )

    # Declaración del argumento para usar el tiempo de simulación de Gazebo.
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Usar tiempo simulado (reloj de Gazebo)",
    )

    # Obtención de los valores de los argumentos de lanzamiento
    name = LaunchConfiguration("name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Procesamiento del archivo URDF/XACRO del robot.
    # Utiliza 'xacro' para procesar el archivo 'diffbot.urdf.xacro'
    # ubicado en el paquete 'robot_description'.
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"), # Busca el ejecutable xacro
                " ", # Espacio necesario entre el ejecutable y el argumento del archivo
                PathJoinSubstitution( # Construye la ruta completa al archivo xacro
                    [
                        FindPackageShare("robot_description"), # Busca el paquete robot_description
                        "urdf", # Directorio dentro del paquete
                        "diffbot.urdf.xacro", # Nombre del archivo xacro
                    ]
                ),
            ]
        ),
        value_type=str, # Asegura que el valor se interprete como una cadena
    )

    # Nodo para publicar el estado del robot.
    # Lee la descripción del robot generada y la publica en el tópico '/robot_description'.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_content, # La descripción del robot procesada
                "use_sim_time": use_sim_time, # Usa el tiempo de simulación
            }
        ],
        output="screen", # Muestra la salida del nodo en la consola
    )

    # Inclusión del lanzamiento de Gazebo (gz_sim).
    # Lanza una instancia de Gazebo con un mundo vacío.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf", # Argumentos para Gazebo: -r para ejecutar, empty.sdf es el mundo
            "on_exit_shutdown": "True", # Apaga el lanzamiento cuando Gazebo se cierra
        }.items(),
    )

    # Nodo para spawnear la entidad (robot) en Gazebo.
    # Utiliza el ejecutable 'create' de 'ros_gz_sim' para cargar el robot en el simulador.
    # El robot se define a partir del tópico '/robot_description'.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create", # Ejecutable para spawnear modelos
        arguments=["-entity", name, "-topic", "robot_description"], # Argumentos: nombre de la entidad y tópico de descripción
        output="screen",
    )

    # Puente para el reloj entre ROS 2 y Gazebo.
    # Es crucial para que los nodos ROS 2 utilicen el tiempo de simulación de Gazebo.
    clock_bridge = RosNode(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"], # Mapeo del tópico de reloj
        output="screen",
    )

    # Spawners de controladores con retrasos.
    # Se usan TimerAction para introducir un retraso antes de lanzar cada spawner
    # de controlador. Esto ayuda a asegurar que Gazebo y el robot estén
    # completamente cargados antes de intentar activar los controladores,
    # evitando así posibles errores por condiciones de carrera.

    # Spawner para el broadcaster del estado de las articulaciones.
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,  # Espera 5 segundos antes de lanzarlo
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"], # Nombre del controlador a spawnear
                parameters=[{"use_sim_time": use_sim_time}], # Usa el tiempo de simulación
                output="screen",
            )
        ],
    )

    # Spawner para el controlador de velocidad de la rueda izquierda.
    velocity_controller_l_spawner = TimerAction(
        period=7.0,  # Espera 7 segundos (2s después del anterior)
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["velocity_controller_left"], # Nombre del controlador a spawnear
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        ],
    )

    # Spawner para el controlador de velocidad de la rueda derecha.
    velocity_controller_r_spawner = TimerAction(
        period=9.0,  # Espera 9 segundos (2s después del anterior)
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["velocity_controller_right"], # Nombre del controlador a spawnear
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        ],
    )

    # Retorna la descripción de lanzamiento con todos los nodos y acciones.
    return LaunchDescription(
        [
            declare_robot_name,
            declare_use_sim_time,
            robot_state_publisher,
            gz_sim,
            spawn_entity,
            clock_bridge,
            joint_state_broadcaster_spawner,
            velocity_controller_l_spawner,
            velocity_controller_r_spawner,
        ]
    )

