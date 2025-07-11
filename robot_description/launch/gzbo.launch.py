from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node as RosNode

def generate_launch_description():
    # Argumento del nombre del robot
    declare_robot_name = DeclareLaunchArgument(
        'name',
        default_value='robot',
        description='Nombre de la entidad a spawnear en Gazebo'
    )

    # Argumento para activar uso del tiempo simulado
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar tiempo simulado (reloj de Gazebo)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Procesar archivo XACRO
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('robot_description'),
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
            'use_sim_time': use_sim_time
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
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Bridge manual para /clock
    clock_bridge = RosNode(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # Spawner de joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Spawner del controlador de velocidad izquierdo
    velocity_controller_l_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller_l'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Spawner del controlador de velocidad derecho
    velocity_controller_r_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller_r'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_robot_name,
        declare_use_sim_time,
        robot_state_publisher,
        gz_sim,
        spawn_entity,
        clock_bridge,
        joint_state_broadcaster_spawner,
        velocity_controller_l_spawner,
        velocity_controller_r_spawner
    ])