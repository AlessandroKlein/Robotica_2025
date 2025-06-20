##  ros2 launch tp1_robot bringup.launch.py use_rviz:=true

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ========================
    # Argumentos
    # ========================
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # ========================
    # Paths
    # ========================
    pkg_tp1 = FindPackageShare('tp1_robot')
    urdf_file = PathJoinSubstitution([pkg_tp1, 'urdf', 'diffbot.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution([pkg_tp1, 'config', 'diffbot_controllers.yaml'])
    rviz_config = PathJoinSubstitution([pkg_tp1, 'rviz', 'diffbot.rviz'])

    robot_description = Command(['xacro ', urdf_file])

    # ========================
    # Nodos ROS 2
    # ========================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            controllers_yaml
        ],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'diffbot', '-topic', 'robot_description'],
        output='screen'
    )

    load_js_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_left_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'left_wheel_velocity_controller'],
        output='screen'
    )

    load_right_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'right_wheel_velocity_controller'],
        output='screen'
    )

    # ========================
    # Gazebo (vía ros_gz_sim)
    # ========================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': 'empty.sdf'}.items()
    )

    # ========================
    # RViz opcional
    # ========================
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ========================
    # Ensamblado final
    # ========================
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false', description='Lanzar RViz'),
        robot_state_publisher,
        gazebo_launch,
        controller_manager,
        spawn_entity,
        load_js_broadcaster,
        load_left_controller,
        load_right_controller,
        rviz
    ])
