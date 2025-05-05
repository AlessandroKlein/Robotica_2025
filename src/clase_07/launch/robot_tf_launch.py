from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clase_07',
            executable='static_tf_broadcaster',
            name='static_tf_broadcaster'
        ),
        Node(
            package='clase_07',
            executable='dynamic_tf_broadcaster',
            name='dynamic_tf_broadcaster',
            parameters=[{'radius': 2.0, 'hz': 50.0}]
        ),
    ])