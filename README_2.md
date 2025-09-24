# DiffBot - Comandos para LiDAR y Cámara

Este documento contiene todos los comandos necesarios para utilizar el LiDAR y la cámara en el proyecto DiffBot.

## Compilación del Proyecto

Antes de ejecutar cualquier comando, asegúrate de compilar el workspace:

```bash
# Desde el directorio raíz del workspace
cd /home/ale/tp-1
rm -rf {build,install,log} && colcon build --symlink-install && source install/setup.bash
```

## Comandos para LiDAR

### 1. Lanzar Simulación Completa con LiDAR

```bash
# Lanza Gazebo con el robot y todos los sensores
ros2 launch diffbot_bringup completo.launch.py
```

### 2. Ejecutar Detector de Obstáculos LiDAR

```bash
# Ejecuta el nodo de detección de obstáculos por zonas
ros2 run diffbot_control detector_lidar
```

### 3. Visualizar Datos del LiDAR

```bash
# Ver datos crudos del LiDAR
ros2 topic echo /scan

# Visualizar en RViz
ros2 run rviz2 rviz2
# Agregar display tipo LaserScan y configurar topic /scan
```

### 4. Configurar Parámetros del Detector LiDAR

```bash
# Ejecutar con parámetros personalizados
ros2 run diffbot_control detector_lidar --ros-args -p distance_threshold:=0.5 -p zone_angle:=30.0
```

### 5. Monitorear Zonas de Detección

```bash
# Ver marcadores de visualización de zonas
ros2 topic echo /lidar_zones_markers
```

## Comandos para Cámara

### 1. Visualizar Imagen de la Cámara

```bash
# Ver stream de la cámara
ros2 run rqt_image_view rqt_image_view
# Seleccionar topic /camera en la interfaz

# O usando comando directo
ros2 topic echo /camera
```

### 2. Ejecutar Detector de Líneas

```bash
# Ejecuta el nodo de seguimiento de líneas
ros2 run diffbot_control line_detector
```

### 3. Configurar Parámetros del Detector de Líneas

```bash
# Ejecutar con parámetros personalizados para HSV
ros2 run diffbot_control line_detector --ros-args \
  -p hsv_lower_h:=0 -p hsv_lower_s:=0 -p hsv_lower_v:=0 \
  -p hsv_upper_h:=180 -p hsv_upper_s:=255 -p hsv_upper_v:=50 \
  -p linear_speed:=0.2 -p angular_gain:=1.0
```

### 4. Guardar Imágenes

```bash
# Guardar imagen actual
ros2 run image_view image_saver --ros-args --remap image:=/camera
```

## Comandos de Control y Navegación

### 1. Control Manual del Robot

```bash
# Teleoperación con teclado
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 2. Enviar Comandos de Velocidad

```bash
# Comando directo de velocidad
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
```

### 3. Monitorear Odometría

```bash
# Ver datos de odometría
ros2 topic echo /odom

# Ver transformaciones
ros2 run tf2_tools view_frames
```

## Comandos de Diagnóstico

### 1. Verificar Nodos Activos

```bash
# Listar todos los nodos
ros2 node list

# Información de un nodo específico
ros2 node info /lidar_zone_detector
ros2 node info /line_detector
```

### 2. Verificar Topics

```bash
# Listar todos los topics
ros2 topic list

# Información de un topic específico
ros2 topic info /scan
ros2 topic info /camera
```

### 3. Verificar Frecuencia de Publicación

```bash
# Frecuencia del LiDAR
ros2 topic hz /scan

# Frecuencia de la cámara
ros2 topic hz /camera
```

## Configuración de Mundos y Escenarios

### 1. Mundo con Obstáculos

```bash
# Lanzar con mundo de obstáculos
ros2 launch diffbot_gz gzbo2.launch.py
```

### 2. Mundo Vacío

```bash
# Lanzar con mundo vacío
ros2 launch diffbot_gz gzbo.launch.py
```

### 3. Lanzar con RViz para Visualización

```bash
# Lanzar con herramientas de testing (incluye RViz)
ros2 launch diffbot_bringup completo.launch.py testing:=true
```

## Comandos de Grabación y Reproducción

### 1. Grabar Datos de Sensores

```bash
# Grabar datos del LiDAR y cámara
ros2 bag record /scan /camera /cmd_vel /odom
```

### 2. Reproducir Datos Grabados

```bash
# Reproducir bag file
ros2 bag play <nombre_del_bag>
```

## Ejercicios Específicos

### 1. Ejecutar Ejercicios de Cinemática

```bash
# Ejercicio 5 - Cinemática directa
ros2 run diffbot_description cinematica5

# Ejercicio 6 - Cinemática inversa
ros2 run diffbot_description cinematica6

# Ejercicio 7 - Análisis de trayectorias
ros2 run diffbot_description cinematica7
```

### 2. Ejecutar Ejercicios de Control

```bash
# Ejercicio 8 - Control diferencial
ros2 run diffbot_control ejercicio8

# Ejercicio 9 - Cálculo de odometría
ros2 run diffbot_control ejercicio9
```

## Solución de Problemas Comunes

### 1. Si no se detectan datos del LiDAR:

```bash
# Verificar que el bridge esté funcionando
ros2 topic list | grep scan

# Reiniciar el bridge
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

### 2. Si no se ve la imagen de la cámara:

```bash
# Verificar topic de cámara
ros2 topic list | grep camera

# Verificar configuración del bridge
ros2 param get /ros_gz_bridge config_file
```

### 3. Si el robot no se mueve:

```bash
# Verificar controladores
ros2 control list_controllers

# Cargar controladores si es necesario
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller velocity_controller_l
ros2 control load_controller velocity_controller_r
```

## Parámetros Configurables

### Detector LiDAR:
- `distance_threshold`: Distancia mínima de detección (default: 0.8m)
- `zone_angle`: Ángulo de cada zona en grados (default: 36°)
- `scan_topic`: Topic del LiDAR (default: "/scan")

### Detector de Líneas:
- `camera_topic`: Topic de la cámara (default: "/camera")
- `cmd_vel_topic`: Topic de comandos de velocidad (default: "/cmd_vel")
- `linear_speed`: Velocidad lineal (default: 0.1 m/s)
- `angular_gain`: Ganancia angular (default: 0.5)
- `hsv_lower_*` y `hsv_upper_*`: Rangos HSV para detección de color

---

**Nota**: Asegúrate de tener sourced el workspace (`source install/setup.bash`) antes de ejecutar cualquier comando.