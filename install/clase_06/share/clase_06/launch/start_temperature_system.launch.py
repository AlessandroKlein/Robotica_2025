from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Genera y devuelve la descripción del lanzamiento (LaunchDescription) para el sistema de sensores de temperatura.
    
    Este lanzamiento configura dos nodos:
    - temperature_sensor: simula o mide temperaturas con ciertas variaciones.
    - temperature_monitor: supervisa las temperaturas y activa una alarma si se supera un umbral.
    
    Los parámetros de ambos nodos son configurables mediante argumentos de lanzamiento.
    """

    # Argumentos configurables desde la línea de comando o un archivo de lanzamiento
    launch_arguments = [
        DeclareLaunchArgument(
            'base_temperature',
            default_value='27.0',
            description='Temperatura base inicial para el sensor.'
        ),
        DeclareLaunchArgument(
            'max_variation',
            default_value='5.0',
            description='Máxima variación permitida en las lecturas de temperatura.'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='2',
            description='Frecuencia de publicación de datos del sensor (en Hz).'
        ),
        DeclareLaunchArgument(
            'alarm_threshold',
            default_value='30.0',
            description='Temperatura umbral que, al ser superada, activa una alarma.'
        ),
        DeclareLaunchArgument(
            'temperature_unit',
            default_value='C',
            description='Unidad de temperatura (C para Celsius, F para Fahrenheit).'
        ),
    ]

    # Nodo del sensor de temperatura
    temperature_sensor_node = Node(
        package='clase_06',
        executable='temperature_sensor',
        name='temperature_sensor',
        output='screen',
        parameters=[{
            'base_temperature': LaunchConfiguration('base_temperature'),
            'max_variation': LaunchConfiguration('max_variation'),
            'publish_rate': LaunchConfiguration('publish_rate')
        }],
        ros_arguments=['--log-level', 'info']  # Nivel de logueo configurado como "info" para ver mensajes relevantes
    )

    # Nodo que monitorea la temperatura
    temperature_monitor_node = Node(
        package='clase_06',
        executable='temperature_monitor',
        name='temperature_monitor',
        output='screen',
        parameters=[{
            'alarm_threshold': LaunchConfiguration('alarm_threshold'),
            'temperature_unit': LaunchConfiguration('temperature_unit')
        }],
        ros_arguments=['--log-level', 'info']
    )

    # Se devuelve una lista de todas las acciones que ROS2 debe ejecutar
    return LaunchDescription(
        launch_arguments + [temperature_sensor_node, temperature_monitor_node]
    )
