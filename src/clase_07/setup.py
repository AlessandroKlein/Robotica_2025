from setuptools import find_packages
from setuptools import setup

import os
from glob import glob

package_name = 'clase_07'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
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
            'static_tf_broadcaster = clase_07.static_tf_broadcaster:main',
            'dynamic_tf_broadcaster = clase_07.dynamic_tf_broadcaster:main',
        ],
    },
)


# colcon build --packages-select clase_07 --symlink-install
# source install/setup.bash
# ros2 run clase_07 static_tf_broadcaster  # verás logs periódicos
# ros2 run clase_07 dynamic_tf_broadcaster # verás logs del movimiento