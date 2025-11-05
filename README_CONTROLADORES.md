# Controladores para Robot Diferencial

Este documento describe los controladores implementados para las Clases 17 y 18, incluyendo instrucciones para su ejecución y configuración.

## Clase 17: Controlador Point Follower

El controlador `point_follower` implementa un controlador cinemático que guía al robot hacia un punto objetivo utilizando la ley de control basada en coordenadas polares.

### Ejecución

Para ejecutar el controlador point follower:

```bash
ros2 launch diffbot_bringup go_to_point.launch.py goal_x:=1.0 goal_y:=1.0
```

### Parámetros configurables

| Parámetro | Descripción | Valor por defecto |
|-----------|-------------|-------------------|
| `goal_x` | Coordenada X del punto objetivo | 1.0 |
| `goal_y` | Coordenada Y del punto objetivo | 1.0 |
| `k_rho` | Ganancia para la distancia al objetivo | 0.5 |
| `k_alpha` | Ganancia para el ángulo hacia el objetivo | 1.0 |
| `k_beta` | Ganancia para el ángulo de orientación final | -0.1 |
| `v_max` | Velocidad lineal máxima (m/s) | 0.22 |
| `w_max` | Velocidad angular máxima (rad/s) | 2.84 |
| `epsilon_tol` | Tolerancia para considerar que se alcanzó el objetivo (m) | 0.05 |
| `control_frequency` | Frecuencia de ejecución del controlador (Hz) | 10.0 |

Ejemplo con parámetros personalizados:

```bash
ros2 launch diffbot_bringup go_to_point.launch.py goal_x:=2.0 goal_y:=3.0 k_rho:=0.8 k_alpha:=1.5 v_max:=0.3
```

## Clase 18: Controlador con Campos Potenciales

El controlador `potential_field_controller` implementa un controlador basado en campos potenciales artificiales que permite al robot navegar hacia un punto objetivo mientras evita obstáculos detectados por el LIDAR.

## Clase 19: Controlador Bug2 con Máquina de Estados

El controlador `bug2_controller` implementa el algoritmo Bug2 con máquina de estados finito que combina navegación hacia un punto objetivo con seguimiento de paredes para evadir obstáculos.

### Ejecución

Para ejecutar el controlador con campos potenciales:

```bash
ros2 launch diffbot_bringup potential_field_controller.launch.py goal_x:=1.0 goal_y:=1.0
```

### Ejecución

Para ejecutar el controlador Bug2:

```bash
ros2 launch diffbot_bringup bug2_controller.launch.py goal_x:=3.0 goal_y:=3.0
```

### Parámetros configurables

| Parámetro | Descripción | Valor por defecto |
|-----------|-------------|-------------------|
| `goal_x` | Coordenada X del punto objetivo | 1.0 |
| `goal_y` | Coordenada Y del punto objetivo | 1.0 |
| `k_att` | Ganancia para la fuerza de atracción | 0.5 |
| `k_rep` | Ganancia para la fuerza de repulsión | 1.0 |
| `d_influence` | Distancia de influencia para la repulsión (m) | 0.8 |
| `v_max` | Velocidad lineal máxima (m/s) | 0.22 |
| `w_max` | Velocidad angular máxima (rad/s) | 2.84 |
| `epsilon_tol` | Tolerancia para considerar que se alcanzó el objetivo (m) | 0.1 |
| `control_frequency` | Frecuencia de ejecución del controlador (Hz) | 10.0 |

Ejemplo con parámetros personalizados:

```bash
ros2 launch diffbot_bringup potential_field_controller.launch.py goal_x:=2.0 goal_y:=3.0 k_att:=0.7 k_rep:=1.5 d_influence:=1.0
```

### Parámetros configurables (Bug2)

| Parámetro | Descripción | Valor por defecto |
|-----------|-------------|-------------------|
| `goal_x` | Coordenada X del punto objetivo | 3.0 |
| `goal_y` | Coordenada Y del punto objetivo | 3.0 |
| `k_rho` | Ganancia para la distancia al objetivo | 0.5 |
| `k_alpha` | Ganancia para el ángulo hacia el objetivo | 1.0 |
| `k_beta` | Ganancia para el ángulo de orientación final | -0.1 |
| `v_max` | Velocidad lineal máxima (m/s) | 0.22 |
| `w_max` | Velocidad angular máxima (rad/s) | 2.84 |
| `epsilon_tol` | Tolerancia para considerar que se alcanzó el objetivo (m) | 0.05 |
| `epsilon_theta` | Tolerancia angular para orientación (rad) | 0.1 |
| `obstacle_distance` | Distancia mínima para detectar obstáculos (m) | 0.5 |
| `control_frequency` | Frecuencia de ejecución del controlador (Hz) | 10.0 |

Ejemplo con parámetros personalizados:

```bash
ros2 launch diffbot_bringup bug2_controller.launch.py goal_x:=4.0 goal_y:=2.0 obstacle_distance:=0.6 k_rho:=0.8
```

## Compilación

Después de cualquier modificación, es necesario compilar el proyecto:

```bash
cd /home/ale/tp-1
colcon build
source install/setup.bash
```

## Notas adicionales

- El controlador point follower es adecuado para entornos sin obstáculos
- El controlador con campos potenciales es más apropiado para entornos con obstáculos
- El controlador Bug2 es ideal para navegación en entornos complejos con múltiples obstáculos
- Todos los controladores utilizan la odometría para conocer la posición actual del robot
- Los controladores con evasión de obstáculos utilizan además los datos del LIDAR para detectar obstáculos
- El algoritmo Bug2 utiliza una máquina de estados finito para alternar entre navegación directa y seguimiento de paredes