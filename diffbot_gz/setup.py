from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'diffbot_gz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')), # rutas de los archivos 
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        #  Mundo obstaculos.sdf
        (os.path.join('share', package_name, 'models'), [
            'models/obstaculos.sdf'
        ]),
        #  Modelo LineTrack incluido en la instalación
        (os.path.join('share', package_name, 'models', 'LineTrack'), [
            'models/LineTrack/model.config',
            'models/LineTrack/model.sdf'
        ]),
        #ruta mallas
        (os.path.join('share', package_name, 'models', 'LineTrack', 'meshes'), 
            glob('models/LineTrack/meshes/*')),
        #ruta materiales
        (os.path.join('share', package_name, 'models', 'LineTrack', 'materials'), 
        glob('models/LineTrack/materials/*')),
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
