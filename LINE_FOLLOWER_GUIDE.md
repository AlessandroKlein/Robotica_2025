# Guía Completa del Sistema de Seguimiento de Líneas

## Descripción General

Este sistema implementa un robot seguidor de líneas utilizando ROS2, Gazebo y OpenCV. El robot utiliza una cámara para detectar líneas negras en el suelo y navegar siguiéndolas automáticamente.

## Componentes del Sistema

### 1. Hardware Simulado
- **Robot**: DiffBot con tracción diferencial
- **Sensores**: 
  - Cámara RPiCamV2 (640x480)
  - IMU
  - LiDAR
  - Encoders de ruedas

### 2. Software
- **Gazebo**: Simulador 3D con mundo LineTrack
- **ROS2 Bridge**: Comunicación entre Gazebo y ROS2
- **LineDetector**: Nodo de procesamiento de imágenes y control

## Instalación y Configuración

### Prerrequisitos
```bash
# Instalar dependencias
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-cv-bridge
sudo apt install python3-opencv
```

### Compilación del Workspace
```bash
cd /home/ale/tp-1
colcon build
source install/setup.bash
```

## Métodos de Ejecución

### Método 1: Launch File Completo (RECOMENDADO)

Este método ejecuta todo el sistema con un solo comando:

```bash
# Compilar y sourcear el workspace
cd /home/ale/tp-1
colcon build
source install/setup.bash

# Ejecutar el sistema completo
ros2 launch diffbot_gz complete_line_follower.launch.py
```

**¿Qué hace este comando?**
- Inicia Gazebo con el mundo LineTrack
- Carga el robot DiffBot con todos sus sensores
- Configura el bridge ROS2-Gazebo
- Lanza el nodo LineDetector con parámetros optimizados
- Configura el motor de renderizado Ogre para mejor rendimiento

### Método 2: Ejecución Manual (Paso a Paso)

Si prefieres ejecutar cada componente por separado:

#### Terminal 1: Gazebo y Robot
```bash
cd /home/ale/tp-1
source install/setup.bash
ros2 launch diffbot_gz line_follower.launch.py
```

#### Terminal 2: LineDetector
```bash
cd /home/ale/tp-1
source install/setup.bash
ros2 run diffbot_control line_detector --ros-args --params-file src/diffbot_control/config/line_detector_params.yaml
```

## Configuración de Parámetros

### Archivo de Configuración
Los parámetros del sistema están en: `src/diffbot_control/config/line_detector_params.yaml`

```yaml
line_detector:
  ros__parameters:
    # Parámetros HSV para detectar líneas negras
    hsv_lower_h: 0      # Hue mínimo (0-180)
    hsv_lower_s: 0      # Saturación mínima (0-255)
    hsv_lower_v: 0      # Valor mínimo (0-255) - negro
    hsv_upper_h: 180    # Hue máximo (0-180)
    hsv_upper_s: 255    # Saturación máxima (0-255)
    hsv_upper_v: 80     # Valor máximo (0-255) - gris oscuro/negro
    
    # Parámetros de control
    linear_speed: 0.15          # Velocidad lineal (m/s)
    angular_gain: 1.2           # Ganancia angular para corrección
    search_angular_speed: 0.4   # Velocidad angular en modo búsqueda
    min_line_area: 50           # Área mínima de píxeles para detectar línea
```

### Ajuste de Parámetros en Tiempo Real

```bash
# Cambiar velocidad lineal
ros2 param set /line_detector linear_speed 0.2

# Cambiar ganancia angular
ros2 param set /line_detector angular_gain 1.5

# Ajustar detección HSV para líneas más claras
ros2 param set /line_detector hsv_upper_v 120

# Cambiar área mínima de detección
ros2 param set /line_detector min_line_area 100
```

## Comandos de Monitoreo y Debug

### Verificar Topics Activos
```bash
ros2 topic list
```

### Monitorear Comandos de Velocidad
```bash
ros2 topic echo /cmd_vel
```

### Verificar Frecuencia de la Cámara
```bash
ros2 topic hz /camera
```

### Ver Información de Topics
```bash
ros2 topic info /camera
ros2 topic info /cmd_vel
```

### Logs del LineDetector
```bash
ros2 node info /line_detector
```

## Acceso a la Interfaz Gráfica

### Gazebo Web Interface
- **URL**: http://localhost:8080
- **Descripción**: Interfaz web de Gazebo para visualizar la simulación
- **Controles**: 
  - Clic y arrastrar para rotar la vista
  - Scroll para zoom
  - Clic derecho para mover la cámara

## Estructura de Archivos

```
tp-1/
├── src/
│   ├── diffbot_control/
│   │   ├── config/
│   │   │   └── line_detector_params.yaml    # Configuración de parámetros
│   │   └── diffbot_control/
│   │       └── line_detector.py             # Nodo principal de detección
│   ├── diffbot_description/
│   │   └── urdf/
│   │       └── diffbot.urdf.xacro          # Descripción del robot
│   └── diffbot_gz/
│       ├── launch/
│       │   ├── line_follower.launch.py      # Launch básico
│       │   └── complete_line_follower.launch.py  # Launch completo
│       ├── models/
│       │   └── LineTrack/                   # Modelo del circuito
│       └── worlds/
│           └── line_track.sdf               # Mundo de simulación
```

## Solución de Problemas

### El robot no se mueve
1. Verificar que la cámara esté publicando:
   ```bash
   ros2 topic info /camera
   ```
2. Verificar parámetros HSV:
   ```bash
   ros2 param get /line_detector hsv_upper_v
   ```
3. Revisar logs del LineDetector para ver si detecta píxeles

### El robot se mueve pero no sigue la línea
1. Ajustar parámetros HSV para mejor detección
2. Reducir la velocidad lineal
3. Aumentar la ganancia angular

### Gazebo no inicia correctamente
1. Verificar que el workspace esté compilado:
   ```bash
   colcon build
   source install/setup.bash
   ```
2. Verificar variables de entorno de Gazebo

### Problemas de rendimiento
1. El launch completo ya configura el motor Ogre para mejor rendimiento
2. Cerrar aplicaciones innecesarias
3. Reducir la resolución de la cámara si es necesario

## Comandos de Limpieza

### Detener todos los procesos
```bash
# Ctrl+C en cada terminal activo
# O usar:
pkill -f gazebo
pkill -f line_detector
```

### Limpiar y recompilar
```bash
cd /home/ale/tp-1
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

## Información Técnica

### Algoritmo de Detección
1. **Captura**: Imagen de 640x480 desde la cámara
2. **Preprocesamiento**: Reducción a 25% del tamaño original
3. **Conversión**: BGR a HSV para mejor detección de color
4. **Filtrado**: Máscara HSV para detectar líneas negras
5. **Análisis**: Cálculo del centroide de la línea detectada
6. **Control**: Generación de comandos Twist basados en la posición de la línea

### Comportamientos del Robot
- **Línea detectada**: Sigue la línea ajustando la velocidad angular
- **Línea perdida (< 25 frames)**: Continúa en la última dirección conocida
- **Línea perdida (> 25 frames)**: Activa modo búsqueda con rotación

### Topics ROS2
- `/camera`: Imágenes de la cámara (sensor_msgs/Image)
- `/camera_info`: Información de calibración de la cámara
- `/cmd_vel`: Comandos de velocidad (geometry_msgs/Twist)
- `/imu/data`: Datos del IMU
- `/lidar/points`: Nube de puntos del LiDAR
- `/gazebo/odometry`: Odometría del robot

---

**Autor**: Sistema de seguimiento de líneas - Clase 15  
**Fecha**: 2025  
**Versión**: 1.0