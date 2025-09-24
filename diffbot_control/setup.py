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
            'cmd_vel_listener = diffbot_control.cmd_vel_listener:main',
            'odometry_publisher = diffbot_control.odometry_publisher:main',
            'go_to_pose_controller = diffbot_control.go_to_pose_controller:main',
            'detector_lidar = diffbot_control.detector_lidar:main',
            'line_detector = diffbot_control.line_detector:main',
            'ejercicio8 = diffbot_control.ejercicio8:main',
            'ejercicio9 = diffbot_control.ejercicio9:main',
        ],
    },
)