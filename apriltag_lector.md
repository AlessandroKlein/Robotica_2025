# Lector de AprilTag Sencillo

Este documento describe cómo implementar un lector de AprilTag sencillo utilizando la teoría presentada en la **Clase 20**. Se utilizarán las librerías `apriltag_ros` e `image_proc`.

## Teoría de Procesamiento

Según la Clase 20, el algoritmo de detección sigue estos pasos:

1.  **Escala de grises + decimación**: Se convierte la imagen y se puede reducir su resolución para ganar velocidad.
2.  **Umbral adaptativo**: Se binariza la imagen para resaltar los bordes.
3.  **Segmentación**: Se agrupan los píxeles para identificar regiones candidatas.
4.  **Detección de cuadriláteros**: Se buscan formas de 4 lados en las regiones segmentadas.
5.  **Decodificación de datos**: Se lee el código binario interno para obtener el ID y verificar la integridad.
6.  **Resultado final**: Se ajusta el cuadrilátero externo para mayor precisión.

## Requisitos previos

Según la teoría, es necesario tener instalado el paquete `apriltag_ros`:

```bash
sudo apt install ros-jazzy-apriltag-ros
```

Y el nodo se ejecuta con:

```bash
ros2 run apriltag_ros apriltag_node
```

## Configuración del Nodo (`config/apriltag-config.yaml`)

Se debe crear un archivo de configuración YAML para definir los parámetros del detector. Este archivo se ubica típicamente en la carpeta `config` del paquete.

```yaml
apriltag:                 # Nombre del nodo
  ros__parameters:
    image_transport: raw 
    family: 36h11         # Familia de la etiqueta (Ej: 36h11)
    size: 0.12            # Tamaño de la etiqueta en metros
    profile: false

    # Ajuste de la detección
    max_hamming: 0        # No aceptar etiquetas con errores si es 0
    detector:
      threads: 1          # Cantidad de hilos de CPU
      decimate: 1.0       # Reducción de resolución (1.0 = sin reducción)
      blur: 0.0           # Suavizado para reducir ruido
      refine: true        # Refinar detección
      sharpening: 0.25    # Enfoque de bordes
      debug: false        # Mostrar resultados intermedios

    pose_estimation_method: "pnp" # Algoritmo para estimar la Pose 3D
```

## Archivo de Lanzamiento (`launch/bringup.launch.py`)

Para ejecutar el lector de manera completa, incluyendo la rectificación de imagen (necesaria para una pose 3D correcta), se utiliza un archivo de lanzamiento en Python.

Este archivo inicia dos nodos:
1.  `rectify_node` (del paquete `image_proc`): Corrige la distorsión de la cámara.
2.  `apriltag_node` (del paquete `apriltag_ros`): Detecta las etiquetas en la imagen rectificada.

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Nodo de rectificación de imagen
    # Se suscribe a /camera/image_raw y publica en /camera/image_rect
    node_image_proc = Node(
        package = 'image_proc',
        executable = 'rectify_node',
        output = 'screen',
        parameters = [{ 'use_sim_time': True }],
        remappings=[
            ('image', '/camera'), # Remapeo para que escuche en el tópico correcto
        ],
    )
    
    # Ruta al archivo de configuración
    # Reemplazar <NOMBRE_PAQUETE> con el nombre real de tu paquete
    apriltag_config_file = PathJoinSubstitution(
        [FindPackageShare('<NOMBRE_PAQUETE>'), 'config', 'apriltag-config.yaml']
    )
    
    # Nodo detector de AprilTag
    node_apriltag_detector = Node(
        package = 'apriltag_ros',
        executable = 'apriltag_node',
        name = 'apriltag',
        output = 'screen',
        parameters = [
            { 'use_sim_time': True }, 
            apriltag_config_file 
        ],
        # Nota: El nodo apriltag se suscribe automáticamente a:
        # - /image_rect (imagen rectificada)
        # - /camera_info
    )

    return LaunchDescription([
        node_image_proc,
        node_apriltag_detector
    ])
```

## Tópicos Importantes

El sistema utiliza y publica en los siguientes tópicos (según la teoría):

-   **Suscripciones**:
    -   `/image_rect` (`sensor_msgs/Image`): Imagen rectificada donde busca los tags.
    -   `/camera_info` (`sensor_msgs/CameraInfo`): Información de calibración de la cámara.

-   **Publicaciones**:
    -   `/detections` (`apriltag_msgs/AprilTagDetectionArray`): Array con todas las detecciones.
    -   `/tf` (`tf2_msgs/TFMessage`): Transformadas 3D de los tags detectados.

### Estructura Detallada de Mensajes

#### `AprilTagDetectionArray.msg`
```msg
std_msgs/Header header
AprilTagDetection[] detections
```

#### `AprilTagDetection.msg`
```msg
string family          # Familia del tag (Ej: tag36h11)
int32 id               # Identificador único
int32 hamming          # Bits distintos entre el tag detectado y el válido
float32 goodness       # Calidad de coincidencia del borde
float32 decision_margin # Claridad entre cuadrados blancos/negros
Point centre           # Centro en coordenadas de píxeles (x,y)
Point[4] corners       # Las 4 esquinas de la etiqueta ((x1,y1),(x2,y2),...)
float64[9] homography  # Matriz homográfica para conversión a Pose 3D
```

## Implementación del Lector en Python (`robot_control/z_apriltag.py`)

Para "leer" y procesar los datos de las etiquetas detectadas por el nodo de ROS 2, se implementa un suscriptor que procesa el arreglo `AprilTagDetectionArray`.

```python
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray

class AprilTagReader(Node):
    def __init__(self):
        super().__init__('apriltag_reader')
        
        # Suscripción al tópico de detecciones definido en la teoría
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.listener_callback,
            10)
        self.subscription  # prevenir que la variable sea eliminada por el garbage collector

    def listener_callback(self, msg):
        # Según la teoría, el mensaje contiene un arreglo de detecciones
        for detection in msg.detections:
            # Extraemos los campos definidos en AprilTagDetection.msg
            tag_id = detection.id
            center_x = detection.centre.x
            center_y = detection.centre.y
            
            self.get_logger().info(f'Tag detectado - ID: {tag_id} | Centro: ({center_x:.2f}, {center_y:.2f})')
            
            # También se dispone de las esquinas (corners) y la matriz homográfica
            # corners = detection.corners
            # homography = detection.homography

def main(args=None):
    rclpy.init(args=args)
    reader = AprilTagReader()
    try:
        rclpy.spin(reader)
    except KeyboardInterrupt:
        pass
    reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Estructura del Paquete ROS 2

Para que el nodo y los archivos de configuración funcionen correctamente, el paquete debe estar bien definido.

### `package.xml`
Se deben incluir las dependencias mencionadas en la teoría (`apriltag_ros`, `image_proc`) y las necesarias para el nodo Python.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_control</name>
  <version>0.0.0</version>
  <description>Paquete para el control de robot y lectura de AprilTags</description>
  <maintainer email="franco@unl.edu.ar">Franco</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>apriltag_msgs</depend>
  <depend>apriltag_ros</depend>
  <depend>image_proc</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### `setup.py`
Para registrar el script del lector y los archivos de lanzamiento:

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir archivos de lanzamiento y configuración
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Franco',
    maintainer_email='franco@unl.edu.ar',
    description='Lector de AprilTags basado en Clase 20',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'z_apriltag = robot_control.z_apriltag:main',
        ],
    },
)
```
