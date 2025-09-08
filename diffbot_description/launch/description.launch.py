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

# =========================================================
# Función principal que define el lanzamiento de nodos para la visualización y manipulación del robot
# =========================================================
def generate_launch_description():
    # -----------------------------------------------------
    # 1. Definir el paquete donde se encuentra la descripción del robot
    # -----------------------------------------------------
    pkg_share = FindPackageShare('diffbot_description')

    # -----------------------------------------------------
    # 2. Declarar argumentos de lanzamiento
    # -----------------------------------------------------
    # Argumento 'testing' para activar/desactivar GUI y RViz
    declare_testing_arg = DeclareLaunchArgument(
        'testing',
        default_value='False', # Valor por defecto
        description='Set to "True" to launch joint_state_publisher_gui and RViz for testing.'
    )
    
    # Argumento 'use_sim_time' para usar el reloj de simulación
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Obtener los valores de los argumentos en tiempo de ejecución
    testing_param = LaunchConfiguration('testing')
    use_sim_time_param = LaunchConfiguration('use_sim_time')

    # -----------------------------------------------------
    # 3. Ruta al archivo Xacro principal que define el robot
    # -----------------------------------------------------
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'diffbot.urdf.xacro'])

    # -----------------------------------------------------
    # 4. Comando para procesar el archivo Xacro y generar el URDF
    # -----------------------------------------------------
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ])

    # -----------------------------------------------------
    # 5. Nodo para publicar la descripción del robot (robot_state_publisher)
    # -----------------------------------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': use_sim_time_param  # Usar el reloj de simulación si es true
        }]
    )

    # -----------------------------------------------------
    # 6. Nodo para publicar los estados articulares (joint_state_publisher)
    #    Solo se lanza cuando 'testing' es True
    # -----------------------------------------------------
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(testing_param) # Condición para lanzar solo en testing
    )

    # -----------------------------------------------------
    # 7. Nodo GUI para manipular manualmente los estados articulares (solo si 'testing' es True)
    # -----------------------------------------------------
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(testing_param) # Condición para lanzar solo en testing
    )

    # -----------------------------------------------------
    # 8. Nodo para lanzar RViz2 con una configuración predefinida (solo si 'testing' es True)
    # -----------------------------------------------------
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
        condition=IfCondition(testing_param) # Condición para lanzar solo en testing
    )

    # -----------------------------------------------------
    # 9. Retornar la descripción del lanzamiento, incluyendo todos los nodos a ejecutar
    # -----------------------------------------------------
    # El orden es importante: primero el argumento, luego los nodos
    return LaunchDescription([
        declare_testing_arg, # Primero declaramos el argumento
        declare_use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node, # Estos se lanzan condicionalmente
        rviz_node # Estos se lanzan condicionalmente
    ])