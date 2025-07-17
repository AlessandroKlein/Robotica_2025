### 🔧 **Construcción del paquete**
```bash
    rosdep install -i --from-path src -y
```

## BORRO VERSIONES ANTERIORES Y CONSTRUYO EL PAQUETE
```bash
    rm -rf {build,install,log} && colcon build --symlink-install && source install/setup.bash

    rm -rf {build,install,log} && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash
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
```

## (Ejercicio 4)
```bash
    source install/setup.bash && ros2 launch gz_robot gzbt.launch.py
```

```
* gz_robot/gztwist.launch.py
* control_robot/config/controllers.yaml
* control_robot/cmd_vel_listener.py
* control_robot/odometry_publisher.py
* robot_description/urdf/diffbot.urdf.xacro
* robot_description/rviz/diffbot.rviz
```

## (Ejercicio 8)
### O el comando para tu archivo de lanzamiento si lo creaste
```bash
    source install/setup.bash && ros2 launch gz_robot gztwist.launch.py

    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" --once

    ros2 topic echo /velocity_controller_left/commands
    ros2 topic echo /velocity_controller_right/commands
```
## (Ejercicio 9)
```bash
    source install/setup.bash && ros2 launch gz_robot gzodom.launch.py
    ros2 topic echo /odom
    ros2 run tf2_ros tf2_echo odom base_link
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

    rviz2
```

## (Ejercicio 10)
```bash
    source install/setup.bash && ros2 launch robot_bringup robot_system.launch.py

    ros2 node list
    ros2 topic list
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" --once
    ros2 topic echo /odom
```
### Ejemplo ejercicio 6
```bash
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2885, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.154}}" --once
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3665, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2885, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.154}}" --once
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3665, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

## (Ejercicio 11)
```bash
    source install/setup.bash && ros2 launch gz_robot gzodom.launch.py
    ros2 topic echo /odom
    ros2 run tf2_ros tf2_echo odom base_link
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

    rviz2
```

## (Ejercicio 12)
```bash
    source install/setup.bash && ros2 launch robot_bringup display_robot_system.launch.py
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" --once
    ros2 topic echo /odom
```
___

## Libreria de ros2 jazzy