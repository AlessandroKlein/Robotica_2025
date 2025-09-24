# Sistema de Calibración de Odometría para Diffbot

Este documento describe cómo usar el sistema completo de calibración de odometría implementado para el robot Diffbot.

## Archivos Implementados

### 1. Nodos de Calibración
- **`square_trajectory.py`**: Ejecuta trayectorias cuadradas para recolección de datos
- **`calibration_data_collector.py`**: Automatiza la recolección de múltiples trayectorias
- **`calibration_calculator.py`**: Calcula los parámetros de calibración

### 2. Nodos Modificados
- **`differential_drive_controller.py`**: Control diferencial con corrección de calibración
- **`odometry_calculator.py`**: Cálculo de odometría con parámetros calibrados

### 3. Archivos de Configuración
- **`diffbot.urdf.xacro`**: Robot URDF con plugin OdometryPublisher de Gazebo
- **`gz_bridge.yaml`**: Configuración del puente para odometría de Gazebo

### 4. Launch Files
- **`gzodom_calibrated.launch.py`**: Launch principal con soporte para calibración
- **`example_calibrated_params.launch.py`**: Ejemplo con parámetros de calibración

## Proceso de Calibración Completo

### Paso 1: Preparar el Entorno
```bash
# Compilar el workspace
cd /home/ale/tp-1
colcon build
source install/setup.bash
```

### Paso 2: Lanzar la Simulación
```bash
# Lanzar Gazebo con el sistema de odometría
ros2 launch diffbot_gz gzodom_calibrated.launch.py
```

### Paso 3: Recolectar Datos de Calibración
```bash
# En una nueva terminal
cd /home/ale/tp-1
source install/setup.bash

# Ejecutar el recolector automático de datos
ros2 run diffbot_control calibration_data_collector

# O ejecutar manualmente trayectorias individuales:
# ros2 run diffbot_control square_trajectory --ros-args -p direction:=cw -p side_length:=4.0
# ros2 run diffbot_control square_trajectory --ros-args -p direction:=ccw -p side_length:=4.0
```

### Paso 4: Calcular Parámetros de Calibración
```bash
# Calcular los parámetros de calibración
ros2 run diffbot_control calibration_calculator

# Esto generará un archivo calibration_parameters.json con:
# - c_L: Coeficiente de corrección rueda izquierda
# - c_R: Coeficiente de corrección rueda derecha  
# - b_actual: Separación corregida entre ruedas
# - alpha, beta: Parámetros de error sistemático
# - E_b, E_d: Errores de separación y diámetro
```

### Paso 5: Aplicar la Calibración

#### Opción A: Usar el Launch File de Ejemplo
1. Editar `example_calibrated_params.launch.py`
2. Reemplazar los valores de ejemplo con los calculados:
```python
example_c_L = "TU_VALOR_c_L"      # Del archivo calibration_parameters.json
example_c_R = "TU_VALOR_c_R"      # Del archivo calibration_parameters.json  
example_b_actual = "TU_VALOR_b_actual" # Del archivo calibration_parameters.json
```
3. Lanzar el sistema calibrado:
```bash
ros2 launch diffbot_gz example_calibrated_params.launch.py
```

#### Opción B: Pasar Parámetros Directamente
```bash
ros2 launch diffbot_gz gzodom_calibrated.launch.py \
    c_L:=TU_VALOR_c_L \
    c_R:=TU_VALOR_c_R \
    b_actual:=TU_VALOR_b_actual
```

## Verificación de la Calibración

### Comparar Odometría Antes y Después
```bash
# Terminal 1: Lanzar sistema calibrado
ros2 launch diffbot_gz gzodom_calibrated.launch.py c_L:=TU_c_L c_R:=TU_c_R b_actual:=TU_b_actual

# Terminal 2: Ejecutar trayectoria de verificación
ros2 run diffbot_control square_trajectory --ros-args -p direction:=cw -p side_length:=4.0

# Terminal 3: Monitorear tópicos
ros2 topic echo /odom          # Odometría del robot (calibrada)
ros2 topic echo /gazebo/odometry # Odometría de Gazebo (referencia)
```

### Métricas de Evaluación
El sistema calcula automáticamente:
- **εx, εy**: Errores de posición en X e Y
- **Error de retorno**: Distancia entre posición inicial y final
- **Mejora de precisión**: Comparación antes/después de calibración

## Parámetros del Sistema

### Parámetros de Calibración
- **`c_L`**: Coeficiente de corrección para rueda izquierda (default: 1.0)
- **`c_R`**: Coeficiente de corrección para rueda derecha (default: 1.0)
- **`b_actual`**: Separación corregida entre ruedas en metros (default: 0.135)

### Parámetros del Robot
- **`wheel_radius`**: Radio de las ruedas en metros (default: 0.035)
- **`wheel_separation`**: Separación nominal entre ruedas en metros (default: 0.135)

### Parámetros de Trayectoria
- **`side_length`**: Longitud del lado del cuadrado en metros (default: 4.0)
- **`linear_velocity`**: Velocidad lineal en m/s (default: 0.2)
- **`angular_velocity`**: Velocidad angular en rad/s (default: 0.5)

## Estructura de Datos

### Archivo calibration_data.json
```json
{
  "cw_runs": [
    {
      "initial_robot_pos": {"x": 0.0, "y": 0.0, "theta": 0.0},
      "final_robot_pos": {"x": 0.1, "y": 0.05, "theta": 0.02},
      "initial_gazebo_pos": {"x": 0.0, "y": 0.0, "theta": 0.0},
      "final_gazebo_pos": {"x": 0.0, "y": 0.0, "theta": 0.0},
      "error_x": 0.1,
      "error_y": 0.05
    }
  ],
  "ccw_runs": [...]
}
```

### Archivo calibration_parameters.json
```json
{
  "c_L": 0.98,
  "c_R": 1.02,
  "b_actual": 0.138,
  "alpha": 0.025,
  "beta": 0.015,
  "E_b": 0.003,
  "E_d": 0.0007
}
```

## Troubleshooting

### Problemas Comunes
1. **Error "No odometry data received"**: Verificar que los tópicos `/odom` y `/gazebo/odometry` estén publicando
2. **Robot no se mueve**: Verificar que los controladores estén activos con `ros2 control list_controllers`
3. **Datos de calibración inconsistentes**: Asegurar que el robot complete las trayectorias sin obstáculos

### Comandos de Diagnóstico
```bash
# Verificar tópicos activos
ros2 topic list | grep -E "(odom|cmd_vel)"

# Verificar controladores
ros2 control list_controllers

# Verificar parámetros del nodo
ros2 param list /differential_drive_controller
ros2 param list /odometry_calculator
```

## Notas Importantes

1. **Tiempo de Estabilización**: Los controladores necesitan tiempo para inicializarse. El launch file incluye temporizadores apropiados.

2. **Precisión de Calibración**: La calidad de la calibración depende de:
   - Múltiples ejecuciones de trayectorias
   - Condiciones consistentes de simulación
   - Ausencia de perturbaciones externas

3. **Validación**: Siempre validar los parámetros calculados ejecutando trayectorias de verificación antes de usar en aplicaciones críticas.

4. **Persistencia**: Los parámetros de calibración deben guardarse y aplicarse en cada sesión de trabajo.

## Referencias

- Laboratorio basado en `/home/ale/tp-1/Teoria/Clase 16/lab.md`
- Teoría de odometría en `/home/ale/tp-1/Teoria/`
- Documentación de calibración en `/home/ale/tp-1/Teoria/Clase 16/`