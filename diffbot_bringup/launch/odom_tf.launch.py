from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Incluir el launch del Ej. 10
    completo_stack = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('diffbot_bringup'),
            'launch',
            'completo.launch.py'
        ])
    ),
    launch_arguments={
        'testing': 'false'  #  fuerza testing:=false en el launch del Ej. 10
    }.items()
)

    # Nodo RViz con configuración personalizada
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
         arguments=['-d', PathJoinSubstitution(
                [FindPackageShare('diffbot_bringup'), 'rviz', 'odom.rviz']
            )
        ]
    )

    return LaunchDescription([
        completo_stack,
        rviz_node
    ])