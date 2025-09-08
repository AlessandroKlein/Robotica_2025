from setuptools import find_packages, setup  # Herramientas de setuptools para distribución 
import os  # Manipulación de rutas del sistema
from glob import glob  # Búsqueda de archivos con patrones

package_name = 'diffbot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
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
        ],
    },
)

# rm -rf build/robot_description install/robot_description
# colcon build --packages-select robot_description
# source install/setup.bash

# ros2 launch robot_description description.launch.py
# ros2 launch robot_description description.launch.py testing:=true
# ros2 launch robot_description description.launch.py testing:=false

# ros2 node list | grep robot_state_publisher
# ros2 topic list | grep tf
# ros2 topic echo /tf

# ros2 run xacro xacro /home/ale/robotica-2025/src/robot_description/urdf/diffbot.xacro
# ros2 run robot_description teleop_twist_keyboard_node

# ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/ale/robotica-2025/src/robot_description/urdf/diffbot.xacro)"


# que utilice base link   -   <link name="base_link"> </link>
# ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link
# 


# ros2 run tf2_ros tf2_monitor
# ros2 run rqt_tf_tree rqt_tf_tree


# ros-jazzy-rqt-tf-tree


# Convertir el archivo .xacro a .urdf directamente
# xacro `ros2 pkg prefix robot_description`/share/robot_description/urdf/diffbot.urdf.xacro > /tmp/diffbot.urdf
# Ejecutar robot_state_publisher con el URDF generado
# ros2 run robot_state_publisher robot_state_publisher /tmp/diffbot.urdf
# (Opcional) Ejecutar joint_state_publisher_gui si tu robot lo requiere
# ros2 run joint_state_publisher_gui joint_state_publisher_gui
# Ejecutar RViz con tu archivo .rviz
# rviz2 -d `ros2 pkg prefix robot_description`/share/robot_description/rviz/diffbot.rviz