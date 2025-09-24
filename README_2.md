# DiffBot - Sistema Completo de Control y Navegación

Este documento contiene todos los comandos necesarios para utilizar el sistema completo
del robot DiffBot, incluyendo LiDAR, cámara, control diferencial y navegación autónoma.

## Archivos Principales del Sistema

- **diffbot_complete_system.launch.py**: Lanzamiento completo del sistema
- **differential_drive_controller.py**: Controlador de tracción diferencial
- **odometry_calculator.py**: Calculador de odometría del robot
- **line_detector.py**: Detector de líneas basado en visión
- **detector_lidar.py**: Detector de obstáculos con LiDAR

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
# Lanza el sistema completo con Gazebo, sensores y controladores
ros2 launch diffbot_bringup diffbot_complete_system.launch.py
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

### 2. Ejecutar Detector de Líneas (Algoritmo Mejorado)

```bash
# Ejecuta el nodo de seguimiento de líneas con algoritmo mejorado para curvas
ros2 run diffbot_control line_detector
```

**Características del Algoritmo Mejorado:**
- **Detección de orientación**: Analiza la orientación de la línea usando momentos de segundo orden
- **Control híbrido**: Combina error de posición y orientación de la línea
- **Mejor rendimiento en curvas**: Especialmente optimizado para curvas menores a 45°
- **Búsqueda inteligente**: Usa la orientación conocida para recuperar líneas perdidas

### 3. Configurar Parámetros del Detector de Líneas

```bash
# Ejecutar con parámetros básicos
ros2 run diffbot_control line_detector --ros-args \
  -p hsv_lower_h:=0 -p hsv_lower_s:=0 -p hsv_lower_v:=0 \
  -p hsv_upper_h:=180 -p hsv_upper_s:=255 -p hsv_upper_v:=50 \
  -p linear_speed:=0.18 -p angular_gain:=0.8
```

```bash
# Ejecutar con parámetros avanzados para curvas
ros2 run diffbot_control line_detector --ros-args \
  -p curve_sensitivity:=2.0 -p orientation_weight:=0.5 \
  -p search_angular_speed:=0.4 -p min_line_area:=80
```

**Parámetros Nuevos:**
- `curve_sensitivity`: Sensibilidad para detectar orientación (1.0-3.0, default: 2.0)
- `orientation_weight`: Peso de orientación vs posición (0.0-1.0, default: 0.5)
- `search_angular_speed`: Velocidad de búsqueda cuando se pierde línea (default: 0.4)
- `min_line_area`: Área mínima para detectar líneas finas (default: 80)

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
# Lanzar con herramientas de desarrollo (incluye RViz y GUI de articulaciones)
ros2 launch diffbot_bringup diffbot_complete_system.launch.py testing:=true
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

### 1. Ejecutar Controladores del Sistema

```bash
# Controlador de tracción diferencial
ros2 run diffbot_control differential_drive_controller

# Calculador de odometría
ros2 run diffbot_control odometry_calculator

# Controlador de navegación a punto específico
ros2 run diffbot_control go_to_pose_controller
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
# Verificar controladores activos
ros2 control list_controllers

# Verificar que el controlador diferencial esté funcionando
ros2 node info /diff_drive_controller

# Cargar controladores si es necesario
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller velocity_controller_l
ros2 control load_controller velocity_controller_r
```

## Parámetros Configurables

### Controlador de Tracción Diferencial:
- `wheel_radius`: Radio de las ruedas en metros (default: 0.035)
- `wheel_separation`: Separación entre ruedas en metros (default: 0.135)

### Calculador de Odometría:
- `wheel_r`: Radio de las ruedas en metros (default: 0.035)
- `wheel_sep`: Separación entre ruedas en metros (default: 0.135)
- `left_wheel_joint`: Nombre del joint de la rueda izquierda
- `right_wheel_joint`: Nombre del joint de la rueda derecha
- `publish_tf`: Publicar transformaciones TF (default: true)

### Detector de Líneas (Algoritmo Mejorado):
- `hsv_lower_h/s/v`: Valores mínimos HSV para detección de color
- `hsv_upper_h/s/v`: Valores máximos HSV para detección de color
- `linear_speed`: Velocidad lineal base (default: 0.15) - Optimizada para mejor control
- `angular_gain`: Ganancia para corrección angular (default: 1.2) - Aumentada para respuesta más rápida
- `curve_sensitivity`: Sensibilidad para detectar orientación (default: 2.5) - Mejorada para curvas
- `orientation_weight`: Peso de orientación vs posición (default: 0.6) - Mayor peso a orientación
- `search_angular_speed`: Velocidad de búsqueda (default: 0.5) - Aumentada para recuperación más rápida
- `min_line_area`: Área mínima para líneas válidas (default: 60) - Reducida para líneas más finas

## Resolución de Problemas Específicos

### 4. Si el robot va al costado en lugar de seguir la línea:

**Problema corregido**: El algoritmo de cálculo del error de posición estaba invertido, causando que el robot se alejara de la línea en lugar de seguirla.

**Solución aplicada**: Se corrigió la fórmula del error de posición de `1.0 - (2.0 * cx / width)` a `(2.0 * cx / width) - 1.0`.

### 5. Si el robot no sigue bien las curvas menores a 45°:

```bash
# Aumentar sensibilidad para curvas suaves
ros2 param set /line_detector curve_sensitivity 2.5

# Aumentar peso de orientación
ros2 param set /line_detector orientation_weight 0.6

# Reducir velocidad para mejor control
ros2 param set /line_detector linear_speed 0.15
```

### 6. Si el robot pierde la línea frecuentemente:

```bash
# Reducir área mínima para líneas más finas
ros2 param set /line_detector min_line_area 60

# Aumentar velocidad de búsqueda
ros2 param set /line_detector search_angular_speed 0.5

# Verificar configuración HSV
ros2 param get /line_detector hsv_lower_v
ros2 param get /line_detector hsv_upper_v
```

### 7. Para optimizar rendimiento en diferentes tipos de líneas:

```bash
# Para líneas gruesas y curvas pronunciadas
ros2 param set /line_detector orientation_weight 0.3
ros2 param set /line_detector curve_sensitivity 1.5

# Para líneas finas y curvas suaves
ros2 param set /line_detector orientation_weight 0.7
ros2 param set /line_detector curve_sensitivity 2.5
ros2 param set /line_detector min_line_area 50
```

### 8. Si el robot no puede seguir curvas de 90° o curvas cerradas:

El sistema incluye un **algoritmo de búsqueda conservador** que se activa cuando se pierde la línea:

- **Fase 0**: Busca en dirección opuesta con intensidad progresiva (20 frames)
- **Fase 1**: Continúa en dirección del último movimiento conocido (0.8x velocidad, 20 frames)
- **Búsqueda suave**: Evita movimientos bruscos que interfieran con curvas graduales
- **Intensidad progresiva**: Aumenta 10% gradualmente (máximo 1.8x)
- **Conservador**: Prioriza estabilidad sobre velocidad de búsqueda

Para curvas muy cerradas, ajustar:
```bash
# Aumentar velocidad de búsqueda
ros2 param set /line_detector search_angular_speed 0.6

# Reducir área mínima para detectar líneas más finas
ros2 param set /line_detector min_line_area 40

# Reducir velocidad lineal para mayor precisión
ros2 param set /line_detector linear_speed 0.12
```

### Detector LiDAR:
- `distance_threshold`: Distancia mínima de detección (default: 0.5m)
- `zone_angle`: Ángulo de cada zona en grados (default: 30°)
- `scan_topic`: Topic del LiDAR (default: "/scan")

### Detector de Líneas:
- `camera_topic`: Topic de la cámara (default: "/camera")
- `cmd_vel_topic`: Topic de comandos de velocidad (default: "/cmd_vel")
- `linear_speed`: Velocidad lineal base (default: 0.2 m/s)
- `angular_gain`: Ganancia para corrección angular (default: 0.5)
- `hsv_lower_h/s/v`: Valores mínimos HSV para detección de color
- `hsv_upper_h/s/v`: Valores máximos HSV para detección de color

---

**Nota**: Asegúrate de tener sourced el workspace (`source install/setup.bash`) antes de ejecutar cualquier comando.