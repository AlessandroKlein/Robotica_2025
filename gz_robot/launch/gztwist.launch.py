from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node as RosNode
from launch.actions import TimerAction


def generate_launch_description():
    declare_robot_name = DeclareLaunchArgument(
        "name",
        default_value="diffbot",
        description="Nombre de la entidad a spawnear en Gazebo",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Usar tiempo simulado (reloj de Gazebo)",
    )

    # Declare parameters for wheel radius and wheel separation
    declare_wheel_radius = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.07",  # r = 0.07 m
        description="Radio de las ruedas del robot",
    )

    declare_wheel_separation = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.135",  # b = 0.135 m
        description="Separación entre las ruedas del robot",
    )

    name = LaunchConfiguration("name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("robot_description"),
                        "urdf",
                        "diffbot.urdf.xacro",
                    ]
                ),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf",
            "on_exit_shutdown": "True",
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-entity", name, "-topic", "robot_description"],
        output="screen",
    )

    clock_bridge = RosNode(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # Spawners con retrasos para evitar llamadas simultáneas
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,  # espera 5s antes de lanzarlo
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        ],
    )

    velocity_controller_l_spawner = TimerAction(
        period=7.0,  # 2s después del anterior
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["velocity_controller_left"],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        ],
    )

    velocity_controller_r_spawner = TimerAction(
        period=9.0,  # 2s después del anterior
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["velocity_controller_right"],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        ],
    )

    # Nuevo nodo para el listener de cmd_vel
    cmd_vel_listener_node = TimerAction(
        period=11.0, # Lanzar después de los controladores de velocidad
        actions=[
            Node(
                package="control_robot",
                executable="cmd_vel_listener",
                name="cmd_vel_listener",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"wheel_radius": wheel_radius},
                    {"wheel_separation": wheel_separation}
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            declare_robot_name,
            declare_use_sim_time,
            declare_wheel_radius,
            declare_wheel_separation,
            robot_state_publisher,
            gz_sim,
            spawn_entity,
            clock_bridge,
            joint_state_broadcaster_spawner,
            velocity_controller_l_spawner,
            velocity_controller_r_spawner,
            cmd_vel_listener_node,  # Añadir el nodo del listener
        ]
    )