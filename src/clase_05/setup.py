from setuptools import setup
import os
from glob import glob

package_name = 'clase_05'  # O el nombre que le hayas dado a tu paquete

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu@email.com',
    description='Laboratorio Clase 05: Publisher y Subscriber orientado a objetos',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = clase_05.simple_publisher:main',
            'simple_subscriber = clase_05.simple_subscriber:main',
        ],
    },
)

## source install/setup.bash
## ros2 run clase_05 simple_publisher
## ros2 run clase_05 simple_subscriber