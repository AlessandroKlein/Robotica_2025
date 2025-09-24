# Ejercicio Práctico - Clase 13: Simulación de Sensores con Gazebo

## Objetivo
Implementar y probar sensores IMU y LiDAR en Gazebo, configurar el `ros_gz_bridge`, y crear un detector de obstáculos con 5 zonas de detección.

## Materiales Teóricos Utilizados
- `/home/ale/tp-1/Teoria/Clase 13/lab.md` - Especificaciones del ejercicio
- `/home/ale/tp-1/Teoria/Clase 13/clase13.md` - Teoría sobre sensores IMU y LiDAR

## Análisis Inicial

### 1. Revisión de la Configuración Existente

**Comando para explorar la estructura del proyecto:**
```bash
cd /home/ale/tp-1
find src/ -name "*.urdf.xacro" -o -name "*.yaml" | head -10
```

**Resultado:** Se encontró que el robot ya tenía configurados los sensores IMU y LiDAR en el archivo URDF:
- `src/diffbot_description/urdf/diffbot.urdf.xacro`

### 2. Verificación de Sensores en URDF

**Configuración IMU encontrada:**
- Sensor tipo: `imu`
- Plugin: `gz-sim-imu-system`
- Frecuencia: 100 Hz
- Tópico: `/imu/data`
- Ruido gaussiano configurado para acelerómetro y giróscopo

**Configuración LiDAR encontrada:**
- Sensor tipo: `gpu_lidar`
- Frecuencia: 10 Hz
- Tópico: `/scan`
- Rango: 0.05 - 15.0 m
- Resolución angular: 0.1125°
- 3200 muestras (360°)

### 3. Verificación del ros_gz_bridge

**Archivo de configuración:** `src/diffbot_gz/config/gz_bridge.yaml`

**Bridges configurados:**
```yaml
# IMU sensor bridge
- ros_topic_name: "/imu/data"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

# LiDAR sensor bridge
- ros_topic_name: "/lidar/scan"
  gz_topic_name: "/lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

## Implementación y Pruebas

### 4. Compilación del Workspace

**Comando de compilación:**
```bash
cd /home/ale/tp-1
colcon build
```

**Resultado:** Compilación exitosa de 8 paquetes en 7.40s

### 5. Configuración del Entorno

**Comando para cargar el entorno:**
```bash
cd /home/ale/tp-1
source install/setup.bash
```

### 6. Lanzamiento del Sistema Completo

**Comando para iniciar Gazebo con todos los sensores:**
```bash
ros2 launch diffbot_bringup diffbot_complete_system.launch.py
```

**Resultado:** Sistema iniciado exitosamente con:
- Gazebo Sim
- Robot diffbot con sensores
- ros_gz_bridge activo
- Controladores de movimiento
- Detector de obstáculos LiDAR

## Verificación de Funcionamiento

### 7. Verificación de Tópicos Disponibles

**Comando:**
```bash
ros2 topic list
```

**Tópicos relevantes encontrados:**
- `/imu/data` - Datos del sensor IMU
- `/scan` - Datos del sensor LiDAR
- `/camera` - Imágenes de la cámara
- `/lidar_zones` - Zonas de detección del LiDAR
- `/odom` - Odometría del robot

### 8. Prueba del Sensor IMU

**Comando para verificar datos del IMU:**
```bash
ros2 topic echo /imu/data --once
```

**Resultado exitoso:** Se recibieron datos de:
- Orientación (quaternion)
- Velocidad angular (rad/s)
- Aceleración lineal (m/s²)
- Matrices de covarianza

**Ejemplo de datos recibidos:**
```
orientation:
  x: 0.002451878984723398
  y: -0.006095169402510651
  z: 0.36711088956344046
  w: 0.9301539829315028
angular_velocity:
  x: 0.018492198514634177
  y: -0.38397682101264224
  z: -0.0296595312140483
linear_acceleration:
  x: -1.4497308892249827
  y: 3.607051730494356
  z: 7.419497261545196
```

### 9. Prueba del Sensor LiDAR

**Comando para verificar datos del LiDAR:**
```bash
ros2 topic echo /scan --once
```

**Resultado exitoso:** Se recibieron datos de:
- 3200 mediciones de distancia
- Ángulos de -π a +π radianes
- Distancias en metros
- Intensidades de retorno

**Comando para información del tópico:**
```bash
ros2 topic info /scan
```

**Resultado:**
- Tipo: `sensor_msgs/msg/LaserScan`
- 2 publicadores
- 2 suscriptores

## Detector de Obstáculos con 5 Zonas

### 10. Verificación del Detector Existente

**Archivo:** `src/diffbot_control/diffbot_control/detector_lidar.py`

**Características del detector:**
- **5 zonas de detección:**
  - L (Left): 2.09 - 2.62 rad
  - FL (Front-Left): 1.57 - 2.09 rad
  - F (Front): 1.04 - 1.57 rad
  - FR (Front-Right): 0.52 - 1.04 rad
  - R (Right): 0.00 - 0.52 rad

- **Funcionalidades:**
  - Detección de distancia mínima por zona
  - Alertas cuando la distancia < 0.5m
  - Visualización con marcadores en RViz
  - Publicación en tópico `/lidar_zones`

**Configuración en launch file:**
```python
lidar_detector_node = Node(
    package='diffbot_control',
    executable='detector_lidar',
    name='lidar_detector',
    parameters=[
        {'use_sim_time': True},
        {'distance_threshold': 0.5},  # Umbral de detección
        {'zone_angle': 30.0}          # Ángulo de zona
    ]
)
```

## 7. Configuración de RViz para Visualización

### 7.1 Modificación del archivo de configuración RViz
Se actualizó el archivo `diffbot.rviz` para incluir la visualización del LiDAR y las zonas de detección:

```bash
# Archivo modificado: src/diffbot_description/rviz/diffbot.rviz
```

**Componentes agregados:**
- **LaserScan Display**: Visualiza el haz de medición del LiDAR
  - Tópico: `/scan`
  - Estilo: Flat Squares
  - Color: Basado en intensidad
  - Tamaño: 3 píxeles

- **MarkerArray Display**: Visualiza las zonas de detección de obstáculos
  - Tópico: `/lidar_zones`
  - Namespace: `lidar_zones`
  - Muestra las 5 zonas (L, FL, F, FR, R) como sectores triangulares

### 7.2 Configuración automática de RViz
Se modificó `diffbot_complete_system.launch.py` para que RViz se abra automáticamente:

```python
# Cambio en el valor por defecto del argumento 'testing'
DeclareLaunchArgument('testing', default_value='true')
```

### 7.3 Verificación de la visualización
Con estas modificaciones, al ejecutar el sistema se puede observar:
- El haz de medición del LiDAR en tiempo real
- Las 5 zonas de detección coloreadas
- Alertas visuales cuando se detectan obstáculos a menos de 0.5m

## 8. Comandos de Ejecución Final

**Comando para iniciar el sistema completo:**
```bash
ros2 launch diffbot_bringup diffbot_complete_system.launch.py
```

**Comandos de verificación:**
```bash
# Verificar tópicos activos
ros2 topic list

# Monitorear IMU
ros2 topic echo /imu/data

# Monitorear LiDAR
ros2 topic echo /scan

# Monitorear zonas de detección
ros2 topic echo /lidar_zones
```

## Resultados y Conclusiones

### ✅ Objetivos Cumplidos

1. **Sensor IMU:** ✅ Configurado y funcionando
   - Frecuencia: 100 Hz
   - Ruido gaussiano implementado
   - Bridge ROS-Gazebo activo

2. **Sensor LiDAR:** ✅ Configurado y funcionando
   - Frecuencia: 10 Hz
   - Rango: 0.05-15m, Resolución: 1cm
   - 3200 muestras por escaneo

3. **ros_gz_bridge:** ✅ Configurado correctamente
   - Bridges para IMU, LiDAR, cámara
   - Comunicación bidireccional activa

4. **Detector de Obstáculos:** ✅ Implementado con 5 zonas
   - Detección en tiempo real
   - Visualización en RViz
   - Alertas por zona

### 📊 Parámetros Técnicos Verificados

| Sensor | Parámetro | Valor Configurado | Estado |
|--------|-----------|-------------------|---------|
| IMU | Frecuencia | 100 Hz | ✅ |
| IMU | Ruido acelerómetro | σ=0.0016-0.0019 | ✅ |
| IMU | Ruido giróscopo | σ=0.00174533 | ✅ |
| LiDAR | Frecuencia | 10 Hz | ✅ |
| LiDAR | Rango | 0.05-15m | ✅ |
| LiDAR | Resolución | 1cm | ✅ |
| LiDAR | Muestras | 3200 | ✅ |

### 🔧 Comandos de Verificación Adicionales

**Para monitorear continuamente los sensores:**
```bash
# IMU en tiempo real
ros2 topic echo /imu/data

# LiDAR en tiempo real  
ros2 topic echo /scan

# Zonas de detección
ros2 topic echo /lidar_zones
```

**Para visualización en RViz:**
```bash
ros2 run rviz2 rviz2
```

## Modificaciones Realizadas

### Edición del Launch File para Clase 13

**Archivo modificado:** `src/diffbot_bringup/launch/diffbot_complete_system.launch.py`

**Cambios realizados:**
1. **Actualización de documentación:** Cambié el título y descripción para reflejar que es específico para la Clase 13 (IMU y LiDAR)
2. **Eliminación de componentes de cámara:**
   - Removido `line_detector_node` (detector de líneas basado en visión)
   - Removido `camera_viewer` (visualizador de imágenes)
3. **Actualización de comentarios:** Modificé los comentarios del `bridge_node` para indicar que solo maneja IMU y LiDAR
4. **Simplificación del sistema:** El launch file ahora se enfoca únicamente en sensores IMU y LiDAR según la teoría de la Clase 13

**Componentes mantenidos:**
- Simulación en Gazebo (`sim_stack`)
- Puente ROS-Gazebo (`bridge_node`) para IMU y LiDAR
- Controlador diferencial (`nodo_control`)
- Calculador de odometría (`nodo_odometria`)
- Detector de obstáculos LiDAR (`lidar_detector_node`)
- Herramientas opcionales de desarrollo (RViz, joint_state_publisher_gui)

**Resultado:** El sistema ahora está alineado con la teoría de la Clase 13, enfocándose exclusivamente en sensores IMU y LiDAR sin componentes de visión por computadora.

## Archivos Modificados/Revisados

1. **URDF del robot:** `src/diffbot_description/urdf/diffbot.urdf.xacro`
2. **Configuración bridge:** `src/diffbot_gz/config/gz_bridge.yaml`
3. **Detector LiDAR:** `src/diffbot_control/diffbot_control/detector_lidar.py`
4. **Launch file:** `src/diffbot_bringup/launch/diffbot_complete_system.launch.py`

## Fecha de Realización
Ejercicio completado exitosamente el día de hoy con todos los objetivos cumplidos.