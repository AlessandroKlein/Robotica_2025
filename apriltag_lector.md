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
