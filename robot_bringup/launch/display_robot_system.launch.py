from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node as RosNode # Renombramos para evitar conflicto con launch.actions.Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declarar los argumentos que se pasarán al launch del sistema del robot y a RViz
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Usar tiempo simulado (reloj de Gazebo)",
    )
    declare_robot_name = DeclareLaunchArgument(
        "name",
        default_value="diffbot",
        description="Nombre de la entidad a spawnear en Gazebo",
    )
    declare_wheel_radius = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.07",
        description="Radio de las ruedas del robot",
    )
    declare_wheel_separation = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.135",
        description="Separación entre las ruedas del robot",
    )
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

    # Obtener los valores de los argumentos
    use_sim_time = LaunchConfiguration("use_sim_time")
    name = LaunchConfiguration("name")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    left_wheel_joint_name = LaunchConfiguration("left_wheel_joint_name")
    right_wheel_joint_name = LaunchConfiguration("right_wheel_joint_name")

    # Incluir el archivo de lanzamiento del sistema del robot (Ejercicio 10)
    # Este ya contiene el robot_state_publisher, los controladores y el odometry_publisher
    # y el bridge del clock.
    robot_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_bringup"), "launch", "robot_system.launch.py"]
            )
        ),
        launch_arguments={
            "name": name,
            "use_sim_time": use_sim_time,
            "wheel_radius": wheel_radius,
            "wheel_separation": wheel_separation,
            "left_wheel_joint_name": left_wheel_joint_name,
            "right_wheel_joint_name": right_wheel_joint_name,
        }.items(),
    )

    # Ruta al archivo de configuración de RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "rviz", "diffbot.rviz"]
    )

    # Nodo de RViz2 con un retraso suficiente para asegurar que TODA la cadena TF esté publicada.
    # El robot_system.launch.py ya tiene sus propios temporizadores internos,
    # pero RViz es un consumidor final que necesita que todo esté listo.
    rviz_node = TimerAction(
        period=20.0, # Aumentado a 20s para mayor seguridad. Ajusta si es necesario.
        actions=[
            RosNode(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],
                parameters=[{"use_sim_time": use_sim_time}],
                # Remapear /tf y /tf_static si es necesario, aunque normalmente no lo es
                # remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
            )
        ]
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_robot_name,
            declare_wheel_radius,
            declare_wheel_separation,
            declare_left_wheel_joint_name,
            declare_right_wheel_joint_name,
            robot_system_launch, # Lanza Gazebo, robot_state_publisher, controladores, etc.
            rviz_node,           # Lanza RViz (con retraso)
        ]
    )