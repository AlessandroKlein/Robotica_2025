from setuptools import find_packages
from setuptools import setup

import os
from glob import glob

package_name = 'clase_06'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir todos los archivos de la carpeta launch
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
            'temperature_sensor = clase_06.temperature_sensor:main',
            'temperature_monitor = clase_06.temperature_monitor:main',
        ],
    },
)

## source install/setup.bash
## ros2 run clase_06 temperature_sensor
## ros2 run clase_06 temperature_monitor
## ros2 launch clase_06 start_temperature_system.launch.py