# Cinemática de un robot diferencial con ROS2

**Guía de trabajos prácticos 2025 - Entregable 1**

Este repositorio contiene los paquetes y configuraciones necesarios para los ejercicios de cinemática de un robot diferencial utilizando ROS2.

-----

## 🚀 Puesta en Marcha

Para trabajar con los paquetes de este proyecto, sigue los siguientes pasos:

### 🛠️ Construcción del Proyecto

Primero, asegúrate de tener todas las dependencias necesarias:

```bash
rosdep install -i --from-path src -y
```

Para construir todos los paquetes del proyecto, puedes usar los siguientes comandos. Es recomendable limpiar las versiones anteriores para evitar conflictos:

```bash
# Limpia y construye todos los paquetes en modo debug (útil para desarrollo)
rm -rf {build,install,log} && colcon build --symlink-install && source install/setup.bash

# O, limpia y construye todos los paquetes en modo release (optimizado para rendimiento)
rm -rf {build,install,log} && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash
```

Si necesitas reconstruir un paquete específico, usa el siguiente formato:

```bash
# Reconstruir el paquete 'diffbot_description'
rm -rf build/diffbot_description install/diffbot_description && colcon build --packages-select diffbot_description && source install/setup.bash

# Reconstruir el paquete 'diffbot_gz'
rm -rf build/diffbot_gz install/diffbot_gz && colcon build --packages-select diffbot_gz && source install/setup.bash

# Reconstruir el paquete 'diffbot_control'
rm -rf build/diffbot_control install/diffbot_control && colcon build --packages-select diffbot_control && source install/setup.bash

# Reconstruir el paquete 'diffbot_bringup'
rm -rf build/diffbot_bringup install/diffbot_bringup && colcon build --packages-select diffbot_bringup && source install/setup.bash
```

-----

## 📋 Ejecución de los Ejercicios

A continuación, se detallan los comandos para ejecutar los distintos ejercicios propuestos. Recuerda **siempre ejecutar `source install/setup.bash` en cada nueva terminal** antes de lanzar cualquier comando de ROS2 para asegurar que el entorno esté configurado correctamente.

### Ejercicio 2: Visualización del Robot en RViz

Este ejercicio permite visualizar el modelo del robot en RViz, con o sin la interfaz gráfica.

  * **Lanzar con GUI y RViz (modo de prueba):**
    ```bash
    source install/setup.bash && ros2 launch diffbot_description description.launch.py testing:=True
    ```
  * **Lanzar sin GUI y RViz (modo normal/producción):**
    ```bash
    ros2 launch diffbot_description description.launch.py testing:=False
    ```
      * O simplemente:
        ```bash
        ros2 launch diffbot_description description.launch.py
        ```

### Ejercicio 3: Simulación en Gazebo

Este comando inicia la simulación del robot en Gazebo.

```bash
source install/setup.bash && ros2 launch diffbot_gz gazebo.launch.py
```

### Ejercicio 4: Control Básico del Robot en Gazebo

```bash
source install/setup.bash && ros2 launch diffbot_gz gzbt.launch.py
```

-----

## Uso Ejercicios 5 - 6 - 7

### Ejecución de Scripts de Simulación

  * Para ejecutar los scripts `ej5.py`, `ej6.py` o `ej7.py`:
    ```bash
    python3 ~/tp-1/ej5.py
    ```
    Estos scripts generan gráficos de trayectorias y muestran cálculos cinemáticos.

### Lanzamiento de la Simulación en Gazebo

1.  Lanza el sistema completo con visualización:

    ```bash
    ros2 launch diffbot_bringup display_robot_system.launch.py
    ```

    Esto inicia Gazebo, RViz, y los nodos de control.

2.  Para simulaciones específicas, usa los archivos de lanzamiento en `diffbot_gz/launch/`:

    ```bash
    ros2 launch diffbot_gz gazebo.launch.py
    ```

-----

### Ejercicio 8: Publicación de Comandos de Velocidad

```bash
# Lanza el nodo que maneja los comandos de velocidad
source install/setup.bash && ros2 launch diffbot_gz gztwist.launch.py

# Publica un comando de velocidad lineal (0.5 m/s) y angular (0.2 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" --once

# Detiene el robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Monitorea los comandos enviados a los controladores de las ruedas
ros2 topic echo /left_wheel_cmd
ros2 topic echo /right_wheel_cmd

# Configura parámetros del nodo cmd_vel_listener
ros2 param set /cmd_vel_listener invert_polarity true
ros2 param set /cmd_vel_listener swap_wheels true
```

### Ejercicio 9: Visualización de Odometría en RViz

```bash
# Lanza la simulación con el publicador de odometría
source install/setup.bash && ros2 launch diffbot_gz gzodom.launch.py

# Muestra los datos de odometría
ros2 topic echo /odom

# Verifica la transformación entre 'odom' y 'base_link'
ros2 run tf2_ros tf2_echo odom base_link

# Publica un comando de velocidad para mover el robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}" --once

# Abre RViz2 para visualizar la odometría
rviz2
```

### Ejercicio 10: Sistema de Robot Completo

Este comando lanza un sistema completo con múltiples nodos relacionados con el robot.

```bash
source install/setup.bash && ros2 launch diffbot_bringup robot_system.launch.py

# Lista los nodos activos
ros2 node list

# Lista los tópicos disponibles
ros2 topic list

# Publica un comando de velocidad
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" --once

# Muestra los datos de odometría
ros2 topic echo /odom
```

-----

### Ejemplo Ejercicio 6-7: Secuencia de Movimientos

Este ejemplo muestra cómo publicar una secuencia de comandos de velocidad para realizar un patrón de movimiento.

```bash
# Mueve el robot hacia adelante y gira a la izquierda
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2885, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.154}}" --once

# Mueve el robot hacia adelante
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3665, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Mueve el robot hacia adelante y gira a la derecha
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2885, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.154}}" --once

# Mueve el robot hacia adelante
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3665, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Detiene el robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### Ejercicio 11: Odometría con Visualización Avanzada

Similar al Ejercicio 9, pero puede implicar configuraciones más avanzadas o escenarios específicos del ejercicio.

```bash
# Lanza la simulación con el publicador de odometría
source install/setup.bash && ros2 launch diffbot_gz gzodom.launch.py

# Muestra los datos de odometría
ros2 topic echo /odom

# Verifica la transformación entre 'odom' y 'base_link'
ros2 run tf2_ros tf2_echo odom base_link

# Publica un comando de velocidad para mover el robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Abre RViz2 para visualizar la odometría
rviz2
```

### Ejercicio 12: Demostración del Sistema de Visualización

```bash
source install/setup.bash && ros2 launch diffbot_bringup display_robot_system.launch.py

# Publica un comando de velocidad
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" --once

# Muestra los datos de odometría
ros2 topic echo /odom
```

-----

## ⚙️ Generación del URDF

Si necesitas generar el archivo URDF a partir del XACRO, puedes usar el siguiente comando:

```bash
cd ~/tp-1 # Asegúrate de estar en el directorio raíz de tu proyecto
ros2 run xacro xacro --inorder src/diffbot_description/urdf/diffbot.urdf.xacro > generated_robot.urdf
```

-----

## ⚠️ Nota sobre la Versión de ROS2

Este proyecto ha sido desarrollado y probado con **ROS2 Jazzy**. Asegúrate de que tu entorno ROS2 sea compatible para evitar posibles problemas.

-----

## 📄 Fuentes y Licencias

Algunos archivos y fragmentos de código de este repositorio han sido extraídos o adaptados de las páginas oficiales de **ROS2** y **Gazebo**, así como de la guía de la asignatura disponible en [https://acapovilla.github.io/robotica-2025/](https://acapovilla.github.io/robotica-2025/). La mayoría de estos ejemplos y códigos están sujetos a licencias de código abierto, lo que permite su uso y modificación sin fines de lucro.
