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
        default_value="0.07",  # r = 0.07 m (from diffbot.urdf.xacro)
        description="Radio de las ruedas del robot",
    )

    declare_wheel_separation = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.135",  # b = 0.135 m (from diffbot.urdf.xacro)
        description="Separación entre las ruedas del robot",
    )

    # Declare parameters for joint names for odometry
    declare_left_wheel_joint_name = DeclareLaunchArgument(
        "left_wheel_joint_name",
        default_value="left_wheel_joint",
        description="Nombre de la junta de la rueda izquierda",
    )
    declare_right_wheel_joint_name = DeclareLaunchArgument(
        "right_wheel_joint_name",
        default_value="right_wheel_joint",
        description="Nombre de la junta de la rueda derecha",
    )

    name = LaunchConfiguration("name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    left_wheel_joint_name = LaunchConfiguration("left_wheel_joint_name")
    right_wheel_joint_name = LaunchConfiguration("right_wheel_joint_name")


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
    # Los controladores se definen en control_robot/config/controllers.yaml
    # El JointStateBroadcaster debe cargarse primero
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

    # Nodo de cinemática inversa
    # Definido en control_robot/control_robot/cmd_vel_listener.py
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

    # Nodo de odometría
    # Definido en control_robot/control_robot/odometry_publisher.py
    odometry_publisher_node = TimerAction(
        period=13.0, # Lanzar después de que joint_state_broadcaster esté activo
        actions=[
            Node(
                package="control_robot",
                executable="odometry_publisher",
                name="odometry_publisher",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"wheel_radius": wheel_radius},
                    {"wheel_separation": wheel_separation},
                    {"left_wheel_joint_name": left_wheel_joint_name},
                    {"right_wheel_joint_name": right_wheel_joint_name},
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
            declare_left_wheel_joint_name,
            declare_right_wheel_joint_name,
            robot_state_publisher,
            gz_sim,
            spawn_entity,
            clock_bridge,
            joint_state_broadcaster_spawner,
            velocity_controller_l_spawner,
            velocity_controller_r_spawner,
            cmd_vel_listener_node,
            odometry_publisher_node,
        ]
    )