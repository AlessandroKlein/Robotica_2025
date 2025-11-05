from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diffbot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
       ('share/' + package_name + '/config', ['config/gz_bridge.yaml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'nav_msgs',
        'sensor_msgs',
        'tf2_ros',
        'std_msgs',
        'cv_bridge',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='ale',
    maintainer_email='ale@todo.todo',
    description='Control configuration for diffbot robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Controladores principales del robot
            'differential_drive_controller = diffbot_control.differential_drive_controller:main',
            'odometry_calculator = diffbot_control.odometry_calculator:main',
            
            # Controladores avanzados
            'go_to_pose_controller = diffbot_control.go_to_pose_controller:main',
            
            # Procesamiento de sensores
            'detector_lidar = diffbot_control.detector_lidar:main',
            'line_detector = diffbot_control.line_detector:main',
            'obstacle_spawner = diffbot_control.obstacle_spawner:main',
            'point_follower = diffbot_control.point_follower:main',
            'potential_field_controller = diffbot_control.potential_field_controller:main',
            
            # Scripts de calibración de odometría
            'square_trajectory = diffbot_control.square_trajectory:main',
            'calibration_data_collector = diffbot_control.calibration_data_collector:main',
            'calibration_calculator = diffbot_control.calibration_calculator:main',
        ],
    },
)