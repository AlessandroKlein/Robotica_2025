## Ejercicio 7

#!/usr/bin/env python3

import rclpy # Importa la biblioteca de cliente de ROS2 para Python.
from rclpy.node import Node # Importa la clase Node, que es la base para crear todos los nodos.
from geometry_msgs.msg import Twist # Importa el tipo de mensaje Twist, usado para comandos de velocidad lineal y angular.
from std_msgs.msg import Float64 # Importa el tipo de mensaje Float64, para publicar valores de punto flotante de 64 bits.

class InverseKinematicsNode(Node): # Define la clase del nodo, que hereda de la clase Node de rclpy.
    def __init__(self): # Define el constructor de la clase.
        super().__init__('inverse_kinematics_node') # Llama al constructor de la clase base (Node) y le da un nombre al nodo.
        self.declare_parameter('wheel_separation', 0.110) # Declara un parámetro para la distancia entre las ruedas, con un valor por defecto.
        self.declare_parameter('wheel_radius', 0.035) # Declara un parámetro para el radio de las ruedas, con un valor por defecto.

        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value # Obtiene el valor del parámetro 'wheel_separation' y lo almacena.
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value # Obtiene el valor del parámetro 'wheel_radius' y lo almacena.

        self.left_pub = self.create_publisher(Float64, 'left_wheel_cmd', 10) # Crea un publicador para enviar la velocidad de la rueda izquierda.
        self.right_pub = self.create_publisher(Float64, 'right_wheel_cmd', 10) # Crea un publicador para enviar la velocidad de la rueda derecha.
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10) # Crea un suscriptor para recibir los comandos de velocidad del tópico 'cmd_vel'.

    def twist_callback(self, msg): # Define el método que se llama cada vez que se recibe un mensaje en 'cmd_vel'.
        v = msg.linear.x # Extrae la velocidad lineal en el eje x del mensaje.
        w = msg.angular.z # Extrae la velocidad angular en el eje z del mensaje.

        v_left = (v - (w * self.wheel_separation / 2)) / self.wheel_radius # Calcula la velocidad angular necesaria para la rueda izquierda.
        v_right = (v + (w * self.wheel_separation / 2)) / self.wheel_radius # Calcula la velocidad angular necesaria para la rueda derecha.

        self.left_pub.publish(Float64(data=v_left)) # Publica la velocidad calculada para la rueda izquierda.
        self.right_pub.publish(Float64(data=v_right)) # Publica la velocidad calculada para la rueda derecha.

def main(): # Define la función principal del programa.
    rclpy.init() # Inicializa el sistema de cliente de ROS2 para Python.
    node = InverseKinematicsNode() # Crea una instancia del nodo de cinemática inversa.
    rclpy.spin(node) # Mantiene el nodo vivo, esperando y manejando callbacks.
    rclpy.shutdown() # Libera los recursos utilizados por rclpy.

if __name__ == '__main__': # Punto de entrada del script de Python.
    main() # Llama a la función principal para ejecutar el nodo.