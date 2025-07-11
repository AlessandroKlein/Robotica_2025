# 🤖 Repositorio de Proyecto ROS 2 - `tp1_robot`

---

## 📦 Estructura del Paquete

```
└── tp1_robot/
    ├── launch/         # Archivos .launch.py para iniciar simulación, visualización y control
    ├── urdf/           # Definición del robot en formato XACRO
    ├── config/         # Configuración de controladores (YAML)
    ├── rviz/           # Configuraciones predefinidas de RViz
    ├── meshes/         # Modelos 3D (STL) usados en el robot
    └── src/            # Código fuente de nodos personalizados
```

---

## ⚙️ Comandos Principales

### 🔧 **Construcción del paquete**

rosdep install -i --from-path src -y

```bash
rm -rf {build,install,log} && colcon build --packages-select tp1_robot && source install/setup.bash

ros2 run ros_gz_sim create -entity robot -topic robot_description
ros2 launch tp1_robot gzbo.launch.py

```
> Limpia, construye y carga las variables de entorno del paquete `tp1_robot`.

---

### 🏃‍♂️ **Ejecutar el robot en simulación (Gazebo)**

```bash
ros2 launch tp1_robot diffbot_sim.launch.py
```

```bash
ros2 launch tp1_robot gazebo.launch.py
```

> Inicia Gazebo con el robot cargado, publica su descripción y establece comunicación con ROS 2 usando `ros_gz_bridge`.

---

### 🌐 **Lanzar solo la descripción del robot (RViz)**

```bash
ros2 launch tp1_robot description.launch.py
```
> Carga el modelo URDF del robot, publica el estado de las articulaciones y abre RViz para visualizar el modelo.

---

### 🕹️ **Teleoperación con teclado**

```bash
ros2 run tp1_robot teleop_twist_keyboard_node
```
> Permite mover el robot usando las teclas del teclado (similar a `teleop_twist_keyboard` pero personalizado).

---

### 📈 **Ver estado de las articulaciones**

```bash
ros2 topic echo /joint_states
```
> Muestra los valores actuales de posición, velocidad y esfuerzo de las articulaciones del robot.

---

### 🗺️ **Ver información de odometría**

```bash
ros2 topic echo /odom
```
> Muestra la estimación de la posición y orientación del robot en el marco de referencia `odom`.

---

### 🔄 **Monitor de transformaciones TF**

```bash
ros2 run tf2_ros tf2_monitor
```
> Muestra todas las transformaciones espaciales activas en el sistema (útil para debuggear TF).

---

### 📊 **Visualizar árbol de transformaciones (RQT)**

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```
> Interfaz gráfica para ver el árbol completo de transformaciones (`tf`) en tiempo real.

---

### 🛠️ **Listar controladores activos**

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {}
```
> Muestra los controladores activos gestionados por `controller_manager`.


---

## 📝 Notas Adicionales

- Asegúrate de haber corrido `source install/setup.bash` después de construir el proyecto.
- Si trabajas con Gazebo, verifica que los plugins estén correctamente incluidos en el URDF.
- El paquete ya tiene soporte para ROS 2 Control y transformaciones TF.

---



# Errores

## 5-6-7 Se debe realizar a mano o codigo aparte con script

Se puede realizar en nodo

    - No realizar con nodo al trayectoria del robot 


## Actualizar urdf parte Gazebo

    - Eliminar parametros fisicos rueda
    - Eliminar oddom
    - Parametros antiguos de Gazebo
    - Eliminar diferencia drive

## Separar paquetes y nodos

## Launch

    - Ejercicio N°2
        - Parametro testing reagregar
        - Eliminar Ros_ez_brig (Solo dejar clock)

    - Ejercicio N°4
        - Falta joingroupvelovitycontroler
        -Actualizar y eliminar controladores 

    - Ejercicio N°8
        - Solo crear un nodo (Esta todo las configuraciones solo controlar que funcione)
        - Mal los calculos del 6 y 8. Y mal programada

    - Ejercicio N°5
        - Eliminar y mira licencias
        - Describir comandos que necesita para hacerlo
        - teleop (Marcar de donde se saca)

    - Ejecicio N°9
        - Odom (Mas o menos estaba)
        - 