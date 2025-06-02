```
rm -rf build/tp1_robot install/tp1_robot && colcon build --packages-select tp1_robot && source install/setup.bash 
```

# Instalar dependencias
```
rosdep install --from-paths src --ignore-src -r -y
```

# Construir
```
colcon build --packages-select tp1_robot
```

# Source
```
source install/setup.bash
```

# Lanzar visualización
```
ros2 launch tp1_robot description.launch.py testing:=true
```

# Lanzar sin GUI
```
ros2 launch tp1_robot description.launch.py testing:=false
```

# Ejecutar teleop
```
ros2 run tp1_robot teleop_twist_keyboard_node
```

# Ver estado de juntas
```
ros2 topic echo /joint_states
```

# Ver transformaciones
```
ros2 run tf2_ros tf2_monitor
```

---
# Lanzar todo
```
ros2 launch tp1_robot bringup.launch.py testing:=true
```

# Ver estado de juntas
```
ros2 topic echo /joint_states
```

# Ver transformaciones
```
ros2 run tf2_ros tf2_monitor
```

# Ver gráfico de TF
```
ros2 run rqt_tf_tree rqt_tf_tree
```

# Ver movimiento del robot
```
ros2 topic echo /cmd_vel
```

# Ver odometría
```
ros2 topic echo /odom
```