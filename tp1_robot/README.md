```bash
rm -rf {build,install,log} && colcon build --packages-select tp1_robot && source install/setup.bash 
```
```bash
ros2 launch tp1_robot gazebo.launch.py
```
# V2
```bash
ros2 launch tp1_robot diffbot_sim.launch.py
```
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```
___
# Instalar dependencias
```bash
rosdep install --from-paths src --ignore-src -r -y
```

# Construir
```bash
colcon build --packages-select tp1_robot
```

# Source
```bash
source install/setup.bash
```

ros2 launch tp1_robot diffbot_sim.launch.py

# Lanzar visualización
```bash
ros2 launch tp1_robot description.launch.py testing:=true
```

# Lanzar sin GUI
```bash
ros2 launch tp1_robot description.launch.py testing:=false
```

# Ejecutar teleop
```bash
ros2 run tp1_robot teleop_twist_keyboard_node
```

# Ver estado de juntas
```bash
ros2 topic echo /joint_states
```

# Ver transformaciones
```bash
ros2 run tf2_ros tf2_monitor
```

---
# Lanzar todo
```bash
ros2 launch tp1_robot bringup.launch.py testing:=true
```

# Ver estado de juntas
```bash
ros2 topic echo /joint_states
```

# Ver transformaciones
```bash
ros2 run tf2_ros tf2_monitor
```

# Ver gráfico de TF
```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

# Ver movimiento del robot
```bash
ros2 topic echo /cmd_vel
```

# Ver odometría
```bash
ros2 topic echo /odom
```


# Gazebo
```bash
ros2 run ros_gz_sim create -topic robot_description
```

# V2
```bash
ros2 launch tp1_robot diffbot_sim.launch.py
```
# Xacro
```bash
ros2 run xacro xacro $(ros2 pkg prefix tp1_robot)/share/tp1_robot/urdf/diffbot.xacro
```
# Ver datos
```bash
ps -a
```