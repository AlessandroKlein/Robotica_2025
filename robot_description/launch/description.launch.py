# =========================================================
#   Archivo: description.launch.py
#   Resumen:
#   Este archivo lanza los nodos necesarios para visualizar y manipular el modelo del robot diffbot:
#     - Publica la descripción del robot (URDF generado desde Xacro)
#     - Publica los estados articulares (joint_state)
#     - Permite manipular las articulaciones con GUI (condicional)
#     - Lanza RViz con la configuración específica (condicional)
# =========================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition # Importar para condiciones

def generate_launch_description():
    # Define el nombre de tu paquete de descripción del robot aquí.
    pkg_share = FindPackageShare('robot_description')

    # Declarar el argumento 'testing'
    # Por defecto, puede ser False (no ejecutar GUI y RViz)
    declare_testing_arg = DeclareLaunchArgument(
        'testing',
        default_value='False', # Valor por defecto
        description='Set to "True" to launch joint_state_publisher_gui and RViz for testing.'
    )

    # Obtener el valor del argumento 'testing'
    testing_param = LaunchConfiguration('testing')

    # Ruta al archivo Xacro principal que define tu robot (diffbot.urdf.xacro)
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'diffbot.urdf.xacro'])

    # Comando para procesar el archivo Xacro y generar el URDF
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ])

    # Nodo para publicar la descripción del robot (robot_description)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }]
    )

    # Nodo para publicar los estados articulares del robot.
    # Siempre se lanza para que el robot_state_publisher tenga información de los joints.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Nodo GUI para manipular manualmente los estados articulares.
    # Se lanza SOLO si 'testing' es True.
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(testing_param) # Condición para lanzar
    )

    # Nodo para lanzar RViz2 con una configuración predefinida.
    # Se lanza SOLO si 'testing' es True.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            pkg_share,
            'rviz',
            'diffbot.rviz'
        ])],
        output='screen',
        condition=IfCondition(testing_param) # Condición para lanzar
    )

    # Retorna la descripción del lanzamiento, incluyendo todos los nodos a ejecutar.
    return LaunchDescription([
        declare_testing_arg, # Primero declaramos el argumento
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node, # Estos se lanzan condicionalmente
        rviz_node # Estos se lanzan condicionalmente
    ])