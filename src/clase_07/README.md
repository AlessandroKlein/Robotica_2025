## Compilar

```markdown
colcon build --packages-select clase_07 --symlink-install
source install/setup.bash
```
___

## Ejecutar

```markdown
ros2 run clase_07 static_tf_broadcaster  # verás logs periódicos
ros2 run clase_07 dynamic_tf_broadcaster # verás logs del movimiento
```