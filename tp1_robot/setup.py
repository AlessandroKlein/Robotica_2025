from setuptools import find_packages, setup  # Herramientas de setuptools para distribución 
import os  # Manipulación de rutas del sistema
from glob import glob  # Búsqueda de archivos con patrones

package_name = 'tp1_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),  # Modelos URDF
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),  # Mallas 3D
        ('share/' + 'tp1_robot/launch', ['launch/description.launch.py']),
        #('share/' + 'tp1_robot/urdf', ['urdf/diffbot.xacro']),  # Asegúrate de que el archivo XACRO exista
        ('share/' + 'tp1_robot/meshes', glob('meshes/*.stl')),  # Incluye todos los .stl
        ('share/' + 'tp1_robot/', glob('diffbot.rviz')),  # Incluye todos los .stl
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ale',
    maintainer_email='ale@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_twist_keyboard_node = tp1_robot.teleop_twist_keyboard_node:main',
        ],
    },
)

# rm -rf build/tp1_robot install/tp1_robot
# colcon build --packages-select tp1_robot
# source install/setup.bash

# ros2 launch tp1_robot description.launch.py
# ros2 launch tp1_robot description.launch.py testing:=true
# ros2 launch tp1_robot description.launch.py testing:=false

# ros2 node list | grep robot_state_publisher
# ros2 topic list | grep tf
# ros2 topic echo /tf

# ros2 run xacro xacro /home/ale/robotica-2025/src/tp1_robot/urdf/diffbot.xacro
# ros2 run tp1_robot teleop_twist_keyboard_node

# ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/ale/robotica-2025/src/tp1_robot/urdf/diffbot.xacro)"


# que utilice base link   -   <link name="base_link"> </link>
# ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link
# 


# ros2 run tf2_ros tf2_monitor
# ros2 run rqt_tf_tree rqt_tf_tree


# ros-jazzy-rqt-tf-tree