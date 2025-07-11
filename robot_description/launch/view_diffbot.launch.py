from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'robot_description'

    xacro_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'urdf',
        'diffbot.xacro'
    ])

    robot_description_content = Command([
        TextSubstitution(text='xacro '),
        xacro_file
    ])

    robot_description = ParameterValue(robot_description_content, value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }],
            output='screen'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
