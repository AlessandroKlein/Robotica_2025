from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Argumento del nombre del robot
    declare_robot_name = DeclareLaunchArgument(
        'name',
        default_value='diffbot',
        description='Nombre de la entidad a spawnear en Gazebo'
    )

    # Procesar archivo XACRO
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('diffbot_description'),
            'urdf',
            'diffbot.urdf.xacro'
        ])
    ])

    # Publicar robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
        }],
        output='screen'
    )

    # Lanzar Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'on_exit_shutdown': 'True'
        }.items()
    )

    # Spawn del robot en Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', LaunchConfiguration('name'),
            '-topic', 'robot_description',
            '-z', '0.07'
        ],
        output='screen'
    )

  
    return LaunchDescription([
        declare_robot_name,
        robot_state_publisher,
        gz_sim,
        spawn_entity,
        
      
    ])