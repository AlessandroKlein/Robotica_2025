from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable


def generate_launch_description():
    # Argumento para activar herramientas de prueba
    declare_testing_arg = DeclareLaunchArgument(
        'testing',
        default_value='false',
        description='Activa joint_state_publisher_gui y RViz si es true'
    )
    testing = LaunchConfiguration('testing')
 
    # Nodo opcional: joint_state_publisher_gui
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(testing),
    )

    # Nodo ros_gz_bridge para todos los sensores
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([
                FindPackageShare('diffbot_control'),
                'config',
                'gz_bridge.yaml' ]),
            'use_sim_time': True,
        }],
        output='screen'
    )
    
    # Nodo opcional: RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('diffbot_description'),
            'rviz',
            'diffbot.rviz'
        ])],
        condition=IfCondition(testing),
        parameters=[{'use_sim_time': True}]
    )

    # Incluir launch de Gazebo + spawn + controladores
    sim_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('diffbot_gz'),
                'launch',
                'gzbo2.launch.py'
            ])
        )
    )

    # Nodo de control diferencial: traduce cmd_vel → velocidades angulares
    nodo_control = Node(
        package='diffbot_control',
        executable='ejercicio8',
        name='diff_drive_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Nodo de odometría: calcula odometría desde joint_states
    nodo_odometria = Node(
        package='diffbot_control',
        executable='ejercicio9',
        name='diffbot_odometry_node',
        output='screen',
        parameters=[
            {'wheel_r': 0.035},
            {'wheel_sep': 0.135},
            {'left_wheel_joint': 'left_wheel_joint'},
            {'right_wheel_joint': 'right_wheel_joint'},
            {'publish_tf': True},
            {'use_sim_time': True},
        ]
    )
    
    # Nodo detector de líneas para la cámara
    line_detector_node = Node(
        package='diffbot_control',
        executable='line_detector',
        name='line_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Parámetros HSV configurables para detección de línea
            'hsv_lower_h': 0,
            'hsv_lower_s': 0,
            'hsv_lower_v': 0,
            'hsv_upper_h': 180,
            'hsv_upper_s': 255,
            'hsv_upper_v': 50,
            'linear_speed': 0.2,
            'angular_gain': 0.5
        }]
    )
    
    # Nodo detector de obstáculos usando LiDAR
    lidar_detector_node = Node(
        package='diffbot_control',
        executable='detector_lidar',
        name='lidar_detector',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'distance_threshold': 0.5},
            {'zone_angle': 30.0}
        ]
    )
    
    # Nodo para visualización de imágenes de la cámara
    camera_viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_viewer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_testing_arg,
        joint_state_publisher_gui,
        rviz,
        sim_stack,
        bridge_node,
        nodo_control,
        nodo_odometria,
        line_detector_node,
        lidar_detector_node,
        camera_viewer
    ])