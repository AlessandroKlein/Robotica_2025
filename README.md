# Wiki del Proyecto `tp1_robot` (Robotica_2025)

---

## **Descripción General**
El proyecto `tp1_robot` es un paquete ROS 2 diseñado para simular y controlar un robot móvil diferencial (DiffBot) utilizando herramientas estándar de ROS 2. Incluye modelos 3D, controladores, configuraciones de sensores, y scripts para teleoperación. Está estructurado para integrarse con herramientas como **RViz**, **Gazebo**, y controladores de movimiento.

---

## **Características Principales**
- **Modelo 3D del robot**: Incluye mallas STL para visualización en RViz/Gazebo.
- **Controladores ROS 2**: Configuración de controladores de velocidad para ruedas.
- **Teleoperación**: Nodo para controlar el robot mediante teclado.
- **Visualización**: Archivos de configuración RViz para visualizar el robot en 3D.
- **Lanzadores (Launch)**: Scripts para iniciar la simulación o visualización.
- **Validación**: Tests de calidad de código (PEP257, Flake8, Copyright).

---

## **Estructura del Proyecto**

```
tp1_robot/
├── diffbot.rviz                # Configuración de RViz
├── package.xml                 # Metadatos del paquete ROS 2
├── setup.cfg                   # Configuración de instalación
├── setup.py                    # Script de instalación
├── config/                     # Configuración de controladores
│   └── diffbot_controllers.yaml
├── launch/                     # Scripts de lanzamiento
│   └── description.launch.py
├── meshes/                     # Mallas 3D (STL)
│   ├── caster_base.stl
│   ├── caster_wheel.stl
│   └── ... (otros archivos STL)
├── resource/                   # Recursos adicionales
│   └── tp1_robot
├── test/                       # Tests de calidad
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── tp1_robot/                  # Código fuente Python
│   ├── __init__.py
│   └── teleop_twist_keyboard_node.py
└── urdf/                       # Modelo URDF y macros
    ├── diffbot.urdf.xacro
    ├── inertial_macros.xacro
    └── motor.xacro
```

---

## **Dependencias**
### **Compilación**
- `rclpy`: Biblioteca principal de ROS 2 Python.
- `std_msgs`, `geometry_msgs`, `nav_msgs`: Tipos de mensajes estándar.
- `robot_state_publisher`, `joint_state_publisher_gui`: Para publicar el estado del robot.
- `xacro`: Para procesar macros en archivos URDF.

### **Ejecución**
- `rclpy`, `std_msgs`, `geometry_msgs`, `nav_msgs`.
- `python3-numpy`, `transforms3d`: Librerías Python para cálculos matemáticos.
- `launch`, `launch_ros`: Para scripts de lanzamiento.

---

## **Instalación**
1. **Clonar el repositorio**:
   ```bash
   git clone https://github.com/AlessandroKlein/Robotica_2025.git
   cd Robotica_2025
   ```

2. **Construir el paquete**:
   ```bash
   colcon build --packages-select tp1_robot
   source install/setup.bash
   ```

3. **Verificar instalación**:
   ```bash
   ros2 pkg list | grep tp1_robot
   ```

---

## **Uso**
### **Visualización en RViz**
1. **Iniciar RViz y el modelo del robot**:
   ```bash
   ros2 launch tp1_robot description.launch.py
   ```
   Esto cargará el modelo URDF, el `robot_state_publisher`, y abrirá RViz con la configuración predefinida (`diffbot.rviz`).

2. **Alternativa: Modo prueba**:
   ```bash
   ros2 launch tp1_robot description.launch.py testing:=true
   ```
   Se activará `joint_state_publisher_gui` para ajustar manualmente las posiciones de las articulaciones.

---

### **Teleoperación con Teclado**
1. **Ejecutar el nodo de teleoperación**:
   ```bash
   ros2 run tp1_robot teleop_twist_keyboard_node
   ```
   Este nodo publica mensajes en el topic `/cmd_vel` para controlar el movimiento del robot.

2. **Comandos disponibles**:
   - `w/s`: Avanzar/retroceder.
   - `a/d`: Girar izquierda/derecha.
   - `q/e`: Ajustar velocidad lineal.
   - `z/c`: Ajustar velocidad angular.

---

### **Controladores**
El archivo `config/diffbot_controllers.yaml` define:
- Un **controlador de velocidad** para cada rueda (`left_velocity_controller`, `right_velocity_controller`).
- Un **JointStateBroadcaster** para publicar estados de articulaciones.

Ejemplo de configuración:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    use_sim_time: true
    left_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController
      joints:
        - left_wheel_joint
```

---

## **Archivos Clave**
### **URDF y Xacro**
- `urdf/diffbot.urdf.xacro`: Modelo principal del robot en formato Xacro.
- `urdf/inertial_macros.xacro`: Macros para definir propiedades inerciales.
- `urdf/motor.xacro`: Componentes específicos del motor.

### **Lanzadores**
- `launch/description.launch.py`: Script que:
  - Convierte Xacro a URDF.
  - Inicia `robot_state_publisher`.
  - Condicionalmente lanza `joint_state_publisher_gui` y RViz.

### **Mallas 3D**
Las mallas STL en `meshes/` se usan para:
- Visualización en RViz.
- Colisiones en simuladores como Gazebo.

---

## **Desarrollo**
### **Validación de Código**
Ejecutar tests de calidad:
```bash
colcon test --packages-select tp1_robot
colcon test-result
```

### **Estructura de Tests**
- `test/test_copyright.py`: Verifica licencias en archivos.
- `test/test_flake8.py`: Valida estilo de código (PEP8).
- `test/test_pep257.py`: Verifica docstrings.

---

## **Licencia**
El paquete usa una licencia no especificada (pendiente de definir en `package.xml`). Actualmente, la entrada en `package.xml` es:
```xml
<license>TODO: License declaration</license>
```

---

## **Contribuyendo**
1. Fork del repositorio.
2. Crea una rama: `git checkout -b feature/nueva-funcionalidad`.
3. Realiza cambios y commitea: `git commit -m 'Agrega nueva funcionalidad'`.
4. Sube la rama: `git push origin feature/nueva-funcionalidad`.
5. Abre un Pull Request.

---

## **Contacto**
- **Autor**: Alessandro Klein
- **Email**: ale@todo.todo
- **Repositorio**: [https://github.com/AlessandroKlein/Robotica_2025](https://github.com/AlessandroKlein/Robotica_2025)

---

## **Ejemplos de Comandos Útiles**
- **Convertir Xacro a URDF**:
  ```bash
  xacro /path/to/diffbot.urdf.xacro > /tmp/diffbot.urdf
  ```

- **Publicar transformaciones estáticas**:
  ```bash
  ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link
  ```

- **Monitorear TF**:
  ```bash
  ros2 run tf2_ros tf2_monitor
  ```

---

¡Contribuye y mejora este proyecto para simulaciones robóticas avanzadas! 🤖
