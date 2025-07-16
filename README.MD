### 🔧 **Construcción del paquete**
```bash
    rosdep install -i --from-path src -y
```

## BORRO VERSIONES ANTERIORES Y CONSTRUYO EL PAQUETE
```bash
    rm -rf {build,install,log} && colcon build --symlink-install && source install/setup.bash
```
```bash
    rm -rf build/robot_description install/robot_description && colcon build --packages-select robot_description && source install/setup.bash

    rm -rf build/gz_robot install/gz_robot  && colcon build --packages-select gz_robot  && source install/setup.bash
    
    rm -rf build/control_robot install/control_robot  && colcon build --packages-select control_robot  && source install/setup.bash
```

## Carga los controladores definidos (si están configurados en el launch), Rviz (Ejecicio 2)
    - Para lanzar con la GUI y RViz (modo de prueba):
    ```bash
        source install/setup.bash && ros2 launch robot_description description.launch.py testing:=True
    ```
    - Para lanzar sin la GUI y RViz (modo normal/producción):
    ```bash
        ros2 launch robot_description description.launch.py testing:=False
    ```
        - O simplemente:
        ```bash
            ros2 launch robot_description description.launch.py
        ```

## Carga los controladores definidos (si están configurados en el launch), Gzbo (Ejercicio 3)
```bash
    source install/setup.bash && ros2 launch gz_robot gazebo.launch.py
    ros2 topic pub /velocity_controller_left/commands std_msgs/msg/Float64MultiArray "data: [2.0]"
    ros2 topic pub /velocity_controller_right/commands std_msgs/msg/Float64MultiArray "data: [2.0]"
```

## (Ejercicio 4)
```bash
    source install/setup.bash && ros2 launch gz_robot gzbt.launch.py
```

## (Ejercicio 8)
### O el comando para tu archivo de lanzamiento si lo creaste
```bash
    source install/setup.bash && ros2 run gz_robot twist_controller
```
### 
```bash
    source install/setup.bash && ros2 launch gz_robot robot_control.launch.py
```
### 
```bash
    source install/setup.bash && ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
```
___

## Ver la respuesta del controllers
```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers "{}"
```

## Listado de topic
```bash
ros2 topic list
    ros2 topic echo /clock
```

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x:0.1, y: 0.1}, angular: {z:0.3}}"
```

## Libreria de ros2 jazzy