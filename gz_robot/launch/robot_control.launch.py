import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,  # Asegúrate de que ExecuteProcess esté importado
)
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # Sigue siendo necesaria para otros nodos como robot_state_publisher
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
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

    # Configuraciones para usar en el lanzamiento
    name = LaunchConfiguration("name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Obtener la descripción del robot (URDF) usando xacro
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("robot_description"),  # Paquete que contiene tu URDF
                        "urdf",
                        "diffbot.urdf.xacro",  # Tu archivo URDF principal
                    ]
                ),
            ]
        ),
        value_type=str,
    )

    # Nodo para publicar los estados del robot
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

    # Lanzar el simulador Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf",  # Lanza Gazebo con un mundo vacío y lo resetea
            "on_exit_shutdown": "True",  # Apaga Gazebo cuando el launch file termina
        }.items(),
    )

    # Spawnea la entidad (tu robot) en Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-entity", name, "-topic", "robot_description"],
        output="screen",
    )

    # Puente para el reloj de Gazebo a ROS (necesario para sincronización)
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # --- Configuración para el nodo robot_twist_controller ---
    # Obtiene la ruta al directorio 'share' de tu paquete de control
    your_control_package_share_dir = get_package_share_directory('control_robot')

    # Ruta al archivo de parámetros para el nodo robot_twist_controller
    # Asume que twist_controller_params.yaml está en control_robot/config/
    twist_controller_params_file = os.path.join(your_control_package_share_dir, 'config', 'twist_controller_params.yaml')

    # ******* MODIFICACIÓN CLAVE PARA EJECUTAR EL NODO PYTHON DIRECTAMENTE *******
    # Construye la ruta directa a tu script Python fuente.
    # Esto asume la siguiente estructura: ~/tp-1/src/control_robot/control_robot/robot_twist_controller.py
    # La ruta desde share/control_robot a src/control_robot/control_robot/robot_twist_controller.py es:
    #   ../src/control_robot/control_robot/robot_twist_controller.py
    twist_controller_source_script_path = os.path.join(
        your_control_package_share_dir,  # Esto te lleva a install/control_robot/share/control_robot
        '..',                            # Sube un nivel a install/control_robot
        'src',                           # Sube de install a src (¡ahora en ~/tp-1/src!)
        'control_robot',                 # Entra al directorio del paquete ROS 2 'control_robot'
        'control_robot',                 # Entra al directorio del módulo Python 'control_robot'
        'robot_twist_controller.py'      # El nombre real de tu script Python
    )

    # Nodo para el controlador de cinemática inversa (robot_twist_controller)
    # Usamos ExecuteProcess para ejecutar el script Python directamente
    robot_twist_controller_node = ExecuteProcess(
        cmd=[
            'python3',  # Comando para ejecutar scripts Python
            twist_controller_source_script_path,  # La ruta directa a tu script .py
            '--ros-args',  # Importante para pasar argumentos de ROS 2 al script
            '--params-file', twist_controller_params_file,  # Pasa el archivo de parámetros
            '-p', 'use_sim_time:=true',  # Pasa el parámetro use_sim_time
        ],
        output='screen',
        name='robot_twist_controller',  # Nombre que aparecerá en los logs
        # Opcional: Define el directorio de trabajo para ExecuteProcess
        # cwd=os.path.dirname(twist_controller_source_script_path),
        # Puedes añadir este argumento si necesitas un '__ros_package_name' en los logs de tu nodo,
        # pero para el funcionamiento básico no es estrictamente necesario con ExecuteProcess
        # arguments=[f"__ros_package_name:=control_robot"],
    )
    # --- Fin de la configuración del nodo ---

    # Retraso para el nodo robot_twist_controller
    # Asegura que Gazebo y los controladores de ros2_control estén activos antes de que tu nodo de cinemática inversa intente publicar comandos.
    twist_controller_delayed_launch = TimerAction(
        period=5.0,  # Puedes ajustar este valor si tu simulación tarda más en inicializar
        actions=[robot_twist_controller_node]
    )

    # Definición de la descripción de lanzamiento
    return LaunchDescription(
        [
            declare_robot_name,
            declare_use_sim_time,
            robot_state_publisher,
            gz_sim,
            spawn_entity,
            clock_bridge,
            # Eliminamos los spawners de joint_state_broadcaster y velocity_controllers.
            # El plugin 'gz_ros2_control' en tu URDF los carga automáticamente.
            twist_controller_delayed_launch,  # Solo lanzamos tu nodo de control de Twist
        ]
    )