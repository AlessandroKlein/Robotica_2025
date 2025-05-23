[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/AlessandroKlein/Robotica_2025)

# Guía de Comandos ROS2

## Configuración del Proyecto

### Crear un Nuevo Proyecto
```bash
cd ./src && ros2 pkg create --build-type ament_python <nombre_paquete> && cd ..
```
- Crea un nuevo paquete Python para ROS2. Reemplaza `<nombre_paquete>` (ej: `clase_04`).

### Instalar Dependencias
```bash
rosdep install -i --from-path src -y
```
- Instala todas las dependencias listadas en los archivos `package.xml`.

### Compilar el Proyecto
```bash
colcon build  # Compila todos los paquetes
colcon build --packages-select <nombre_paquete>  # Compila solo un paquete
```
- Usa `--packages-select` para compilar paquetes específicos (ej: `clase_06`).

### Agregar Paquete al Entorno ROS2
```bash
source install/setup.bash
```
- Actualiza el entorno para reconocer el paquete compilado.

---

## Gestión de Paquetes

### Listar Ejecutables de un Paquete
```bash
ros2 pkg executables | grep <nombre_paquete>
```
- Muestra los ejecutables disponibles en `<nombre_paquete>` (ej: `clase_04`).

---

## Ejecución y Depuración

### Ejecutar un Nodo
```bash
ros2 run <nombre_paquete> <nombre_ejecutable>
```
- Ejemplo: `ros2 run clase_04 mi_nodo`.

### Listar Nodos Activos
```bash
ros2 node list
```
- Muestra todos los nodos en ejecución.

### Inspeccionar un Nodo
```bash
ros2 node info <nombre_nodo>
```
- Muestra detalles del nodo: publicadores, suscriptores, etc.

---

## Gestión de Topics

### Listar Topics Activos
```bash
ros2 topic list
```
- Muestra todos los topics disponibles.

### Obtener Metadatos de un Topic
```bash
ros2 topic info <nombre_topic>
```
- Muestra tipo de mensaje, publicadores y suscriptores.

### Monitorear Datos de un Topic
```bash
ros2 topic echo <nombre_topic>
```
- Muestra en tiempo real los datos publicados en el topic.

---

## Registros y Control de Mensajes

### Niveles de Severidad de Logs
```python
# Niveles de prioridad (orden ascendente):
self.get_logger().fatal('Error crítico!')
self.get_logger().error('Error detectado')
self.get_logger().warn('Advertencia')
self.get_logger().info('Información general')
self.get_logger().debug('Depuración')
```

### Opciones Avanzadas de Logging
```python
# Log único (sin repeticiones)
self.get_logger().warn('Advertica única', once=True)

# Omitir primer mensaje
self.get_logger().info('Mensaje omitido', skip_first=True)

# Limitar frecuencia (1 mensaje/segundo)
self.get_logger().debug(f'Dato: {valor}', throttle_duration_sec=1)
```

---

## Archivos de Lanzamiento (Launch)

### Crear Directorio para Launch Files
```bash
mkdir -p src/<nombre_paquete>/launch  # Ejemplo: src/clase_06/launch
```

### Crear Nuevo Archivo de Lanzamiento
```bash
touch src/<nombre_paquete>/launch/<nombre_archivo>.launch.py
```
- Reemplaza `<nombre_paquete>` y `<nombre_archivo>` (ej: `clase_06` y `mi_lanzamiento`).
```

Este formato organiza los comandos en secciones lógicas,
mantiene los ejemplos claros y usa placeholders (`< >`) para personalización.
Ideal para documentación técnica en español.
