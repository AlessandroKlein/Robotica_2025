## Ejercicio 9

#!/usr/bin/env python3

import rclpy # Importa la biblioteca de cliente de ROS2 para Python.
from rclpy.node import Node # Importa la clase Node, la base para crear nodos.
from nav_msgs.msg import Odometry # Importa el mensaje Odometry para publicar los datos de odometría.
from std_msgs.msg import Float64 # Importa el mensaje Float64 para recibir las velocidades de las ruedas.
import math # Importa la biblioteca de matemáticas para funciones como cos y sin.
import transforms3d  # Importa la biblioteca para convertir ángulos de Euler a cuaterniones. Necesario: pip install transforms3d

class OdometryNode(Node): # Define la clase del nodo de odometría.
    def __init__(self): # Constructor de la clase.
        super().__init__('odometry_node') # Llama al constructor de la clase base y nombra el nodo.
        self.declare_parameter('wheel_separation', 0.1) # Declara el parámetro para la separación de las ruedas.
        self.declare_parameter('wheel_radius', 0.035) # Declara el parámetro para el radio de las ruedas.

        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value # Obtiene y almacena el valor de la separación de las ruedas.
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value # Obtiene y almacena el valor del radio de las ruedas.

        self.left_velocity = 0.0 # Inicializa la velocidad de la rueda izquierda.
        self.right_velocity = 0.0 # Inicializa la velocidad de la rueda derecha.
        self.x = 0.0 # Inicializa la posición x del robot.
        self.y = 0.0 # Inicializa la posición y del robot.
        self.theta = 0.0 # Inicializa la orientación (ángulo theta) del robot.
        self.last_time = self.get_clock().now() # Guarda el tiempo actual para calcular el intervalo de tiempo (dt).

        # Suscripción a comandos de velocidad de las ruedas
        self.create_subscription(Float64, 'left_wheel_cmd', self.left_vel_callback, 10) # Se suscribe al tópico de velocidad de la rueda izquierda.
        self.create_subscription(Float64, 'right_wheel_cmd', self.right_vel_callback, 10) # Se suscribe al tópico de velocidad de la rueda derecha.

        # Publicador de odometría
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10) # Crea un publicador para el mensaje de odometría.

    def left_vel_callback(self, msg): # Callback para cuando se recibe un mensaje de velocidad de la rueda izquierda.
        self.left_velocity = msg.data # Actualiza la velocidad de la rueda izquierda.

    def right_vel_callback(self, msg): # Callback para cuando se recibe un mensaje de velocidad de la rueda derecha.
        self.right_velocity = msg.data # Actualiza la velocidad de la rueda derecha.

    def update_odometry(self): # Método para calcular y actualizar la odometría del robot.
        current_time = self.get_clock().now() # Obtiene el tiempo actual.
        dt = (current_time - self.last_time).nanoseconds / 1e9 # Calcula el tiempo transcurrido (dt) desde la última actualización.
        self.last_time = current_time # Actualiza el último tiempo de ejecución.

        v = self.wheel_radius * (self.right_velocity + self.left_velocity) / 2 # Calcula la velocidad lineal del robot.
        w = (self.wheel_radius * (self.right_velocity - self.left_velocity)) / self.wheel_separation # Calcula la velocidad angular del robot.

        delta_x = v * math.cos(self.theta) * dt # Calcula el cambio en la posición x.
        delta_y = v * math.sin(self.theta) * dt # Calcula el cambio en la posición y.
        delta_theta = w * dt # Calcula el cambio en la orientación.

        self.x += delta_x # Actualiza la posición x.
        self.y += delta_y # Actualiza la posición y.
        self.theta += delta_theta # Actualiza la orientación.

        self.publish_odometry(v, w) # Llama al método para publicar la odometría.

    def publish_odometry(self, linear_velocity, angular_velocity): # Método para construir y publicar el mensaje de Odometría.
        odom = Odometry() # Crea una instancia del mensaje Odometry.
        odom.header.stamp = self.get_clock().now().to_msg() # Establece la marca de tiempo del mensaje.
        odom.header.frame_id = 'odom' # Establece el marco de referencia padre ('odom').
        odom.child_frame_id = 'base_link' # Establece el marco de referencia hijo ('base_link').

        # Posición
        odom.pose.pose.position.x = self.x # Establece la posición x en el mensaje.
        odom.pose.pose.position.y = self.y # Establece la posición y en el mensaje.
        odom.pose.pose.position.z = 0.0 # La posición z es 0 para un robot 2D.
        quat = transforms3d.euler.euler2quat(0, 0, self.theta) # Convierte el ángulo de Euler (theta) a un cuaternión.
        odom.pose.pose.orientation.x = quat[1] # Establece el componente x del cuaternión.
        odom.pose.pose.orientation.y = quat[2] # Establece el componente y del cuaternión.
        odom.pose.pose.orientation.z = quat[3] # Establece el componente z del cuaternión.
        odom.pose.pose.orientation.w = quat[0] # Establece el componente w del cuaternión.

        # Velocidad
        odom.twist.twist.linear.x = linear_velocity # Establece la velocidad lineal en el mensaje.
        odom.twist.twist.angular.z = angular_velocity # Establece la velocidad angular en el mensaje.

        self.odom_pub.publish(odom) # Publica el mensaje de odometría completo.

def main(): # Función principal.
    rclpy.init() # Inicializa rclpy.
    node = OdometryNode() # Crea una instancia del nodo de odometría.
    rate = node.create_rate(50)  # Crea un objeto para controlar la frecuencia del bucle a 50 Hz.
    try: # Inicia un bloque try para manejar interrupciones.
        while rclpy.ok(): # Bucle principal que se ejecuta mientras ROS esté activo.
            node.update_odometry() # Llama al método para actualizar la odometría.
            rclpy.spin_once(node, timeout_sec=0.01) # Procesa los eventos pendientes de ROS.
    except KeyboardInterrupt: # Captura la interrupción por teclado (Ctrl+C).
        pass # No hace nada, simplemente sale del bucle.
    finally: # Bloque que se ejecuta siempre al final.
        node.destroy_node() # Destruye el nodo explícitamente.
        rclpy.shutdown() # Cierra rclpy.

if __name__ == '__main__': # Punto de entrada del script.
    main() # Llama a la función principal.