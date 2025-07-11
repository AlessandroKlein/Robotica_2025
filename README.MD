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

## Este comando inserta el robot en la simulación de Gazebo
```bash
    source install/setup.bash && ros2 run ros_gz_sim create -entity robot -topic robot_description
```

## Carga los controladores definidos (si están configurados en el launch), Rviz
```bash
    source install/setup.bash && ros2 launch robot_description description.launch.py
```

## Carga los controladores definidos (si están configurados en el launch), Gzbo
```bash
    source install/setup.bash && ros2 launch gz_robot gazebo.launch.py
```