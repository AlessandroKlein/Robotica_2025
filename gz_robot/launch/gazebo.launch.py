from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Lanza Gazebo, procesa la descripción del robot y spawna el robot.
    """

    # Incluir el archivo description.launch.py del paquete robot_description
    # con testing=False para no lanzar joint_state_publisher_gui ni RViz
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_description"), "launch", "description.launch.py"]
            )
        ),
        launch_arguments={
            "testing": "False",  # No lanzar joint_state_publisher_gui ni RViz
        }.items(),
    )

    # Lanzar Gazebo con mundo vacío
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf",
        }.items(),
    )

    # Spawnar el robot en Gazebo usando spawn_entity.py
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-entity", "diffbot",
            "-topic", "robot_description",
            "-z", "0.05",   # Al menos media rueda por encima del suelo
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_description_launch,
        gz_sim,
        spawn_entity,
    ])

