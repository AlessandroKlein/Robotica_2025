from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# =========================================================
#   Archivo: gazebo.launch.py
#   Resumen:
#   Este archivo lanza todos los nodos necesarios para simular el robot diffbot en Gazebo:
#     - Lanza Gazebo (ros_gz_sim)
#     - Incluye la descripción del robot (desde robot_description)
#     - Spawnea el robot en Gazebo
# =========================================================

def generate_launch_description():
    """
    Genera la descripción de lanzamiento para simular el robot en Gazebo.
    Este archivo:
    1. Lanza Gazebo (ros_gz_sim).
    2. Incluye la descripción del robot desde robot_description.
    3. Spawnea el robot en Gazebo (ros_gz_sim create).
    """

    # -----------------------------------------------------
    # 1. Declaración de argumentos de lanzamiento
    # -----------------------------------------------------
    # Permite al usuario especificar el nombre del robot al lanzar el archivo.
    declare_robot_name = DeclareLaunchArgument(
        "name",
        default_value="diffbot",
        description="Nombre de la entidad a spawnear en Gazebo",
    )

    # -----------------------------------------------------
    # 2. Obtención de los valores de los argumentos de lanzamiento
    # -----------------------------------------------------
    name = LaunchConfiguration("name")

    # -----------------------------------------------------
    # 3. Incluir el archivo description.launch.py del paquete robot_description
    #    con el parámetro testing=False para no lanzar joint_state_publisher_gui ni RViz
    # -----------------------------------------------------
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_description"), "launch", "description.launch.py"]
            )
        ),
        launch_arguments={
            "testing": "False",  # No lanzar joint_state_publisher_gui ni RViz
        }.items(),
    )

    # -----------------------------------------------------
    # 4. Inclusión del lanzamiento de Gazebo (gz_sim)
    # -----------------------------------------------------
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

    # -----------------------------------------------------
    # 5. Nodo para spawnear la entidad (robot) en Gazebo
    # -----------------------------------------------------
    # Utiliza el ejecutable 'spawn_entity.py' para cargar el robot en el simulador.
    # El robot se define a partir del tópico '/robot_description'.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create", # Ejecutable para spawnear modelos
        arguments=["-entity", name, "-topic", "robot_description"], # Argumentos: nombre de la entidad y tópico de descripción
        output="screen",
    )

    # -----------------------------------------------------
    # 6. Retornar la descripción de lanzamiento con todos los nodos y acciones
    # -----------------------------------------------------
    return LaunchDescription(
        [
            declare_robot_name,              # Argumento: nombre de la entidad
            robot_description_launch,        # Incluye: descripción del robot
            gz_sim,                         # Nodo: lanza Gazebo
            spawn_entity,                   # Nodo: spawnea el robot en Gazebo
        ]
    )

