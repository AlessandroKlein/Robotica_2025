from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, AppendEnvironmentVariable, TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

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
            'robot_description': ParameterValue(
                robot_description_content,
                value_type=str
            ),
            'use_sim_time': True,
        }],
        output='screen'
    )

    # Agregar ruta de modelos al entorno
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([FindPackageShare("diffbot_gz"), "models"]),
    )

    # Lanzar Gazebo con mundo personalizado
    gz_sim2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                TextSubstitution(text='-r '),
                FindPackageShare('diffbot_gz'),
                'models',
                'obstaculos.sdf'
            ])
        }.items()
    )

    remove_ground_plane = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="remove",
                parameters=[{ 'entity_name': 'ground_plane' }],
                output="screen",
            )
        ]
    )

    load_track = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-entity", "track",
                    "-file", "model://LineTrack",
                ],
                output="screen",
            )
        ]
    )

    spawn_entity = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-entity', 'diffbot',
                    '-topic', 'robot_description',
                    '-z', '0.07'
                ],
                output='screen'
            )
        ]
    )

    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster','--use-sim-time'],
        output='screen'
    )

    velocity_controller_l = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller_l','--use-sim-time'],
        output='screen'
    )

    velocity_controller_r = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller_r','--use-sim-time'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gz_resource_path,
        gz_sim2,
        spawn_entity,
        joint_state_broadcaster,
        velocity_controller_l,
        velocity_controller_r,
        remove_ground_plane,
        load_track
    ])