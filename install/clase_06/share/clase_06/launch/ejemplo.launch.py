from launch import LaunchDescription
from launch_ros import actions
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    node = Node(
        package='clase_06',             # Nombre del paquete que contiene el ejecutable
        executable='nodo_principal',     # Nombre del ejecutable que se generó al compilar
        name='nodo_clase_06',             # Nombre que quieres darle al nodo
        namespace='clase06',              # Namespace bajo el cual se lanzará el nodo
        parameters=[
            {'parametro_velocidad': 1.5}, # Un parámetro de ejemplo
            {'parametro_nombre': 'robot_1'}
        ],
        remappings=[
            ('/topic_original', '/topic_remapeado') # Redirige un topic
        ],
        output='screen',                  # Salida de los logs (puede ser 'screen', 'log' o 'both')
        ros_arguments=[
            '--log-level', 'info'          # Establece el nivel de logs (debug, info, warn, error, fatal)
        ],
        arguments=[
            '--use_sim_time', 'true'       # Argumentos adicionales que el ejecutable acepte
        ]
    )

    return LaunchDescription([
        node
    ])
