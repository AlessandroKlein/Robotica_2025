from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = FindPackageShare('tp1_robot')

    # Cargar el URDF usando xacro
    urdf_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'diffbot.urdf.xacro'
    ])

    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'use_sim_time': use_sim_time
    }

    # Lanzar Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf',
            'use_sim_time': use_sim_time
        }.items(),
    )

    # Publicar descripción del robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Crear entidad en Gazebo
    create = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-entity', 'diffbot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen',
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen',
        respawn=True,
        respawn_delay=2,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Controlador joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Retraso para asegurar que todo esté listo
    delayed_diff_drive_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'diff_drive_controller',
                    '--param-file',
                    PathJoinSubstitution([
                        FindPackageShare('tp1_robot'),
                        'config',
                        'diff_drive_controller.yaml'
                    ]),
                ],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        gazebo,
        robot_state_publisher,
        create,
        bridge,

        # Una vez que el robot esté creado, lanzamos los controladores
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=create,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[delayed_diff_drive_controller_spawner],
            )
        )
    ])