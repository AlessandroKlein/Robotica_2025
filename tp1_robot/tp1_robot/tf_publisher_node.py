#!/usr/bin/env python3

import rclpy # Importa la biblioteca de cliente de ROS2 para Python.
from rclpy.node import Node # Importa la clase Node, la base para crear nodos.
from nav_msgs.msg import Odometry # Importa el mensaje Odometry, que contiene la pose y velocidad del robot.
from geometry_msgs.msg import TransformStamped # Importa el mensaje para definir una transformación entre dos sistemas de coordenadas.
from tf2_ros import TransformBroadcaster # Importa la clase para publicar transformaciones TF2.

class TFPublisherNode(Node): # Define la clase del nodo que publicará las transformaciones.
    def __init__(self): # Constructor de la clase.
        super().__init__('tf_publisher_node') # Llama al constructor de la clase base y nombra el nodo.
        self.tf_broadcaster = TransformBroadcaster(self) # Crea una instancia del publicador de transformaciones.

        # Suscripción a odometría
        self.subscription = self.create_subscription( # Crea una suscripción.
            Odometry, # Tipo del mensaje a recibir (Odometry).
            'odom', # Nombre del tópico al que se suscribe.
            self.odom_callback, # Función que se llamará al recibir un mensaje.
            10 # Tamaño de la cola de mensajes.
        )

    def odom_callback(self, msg): # Callback que se ejecuta cada vez que se recibe un mensaje de odometría.
        transform = TransformStamped() # Crea una instancia del mensaje de transformación.
        transform.header.stamp = self.get_clock().now().to_msg() # Establece la marca de tiempo de la transformación.
        transform.header.frame_id = "odom" # Establece el marco de coordenadas padre (origen).
        transform.child_frame_id = "base_link" # Establece el marco de coordenadas hijo (destino).

        # Posición
        transform.transform.translation.x = msg.pose.pose.position.x # Copia la posición x desde el mensaje de odometría.
        transform.transform.translation.y = msg.pose.pose.position.y # Copia la posición y desde el mensaje de odometría.
        transform.transform.translation.z = msg.pose.pose.position.z # Copia la posición z desde el mensaje de odometría.

        # Orientación
        transform.transform.rotation = msg.pose.pose.orientation # Copia la orientación (cuaternión) desde el mensaje de odometría.

        # Publicar transformación
        self.tf_broadcaster.sendTransform(transform) # Envía la transformación para que sea publicada en el tópico /tf.

def main(): # Función principal.
    rclpy.init() # Inicializa rclpy.
    node = TFPublisherNode() # Crea una instancia del nodo publicador de TF.
    try: # Inicia un bloque try para manejar la interrupción por teclado.
        rclpy.spin(node) # Mantiene el nodo activo, procesando callbacks.
    except KeyboardInterrupt: # Captura la excepción de interrupción (Ctrl+C).
        pass # No hace nada, solo sale del bucle de spin.
    finally: # Bloque que se ejecuta siempre al final.
        node.destroy_node() # Destruye el nodo explícitamente.
        rclpy.shutdown() # Cierra rclpy y libera los recursos.

if __name__ == '__main__': # Punto de entrada del script de Python.
    main() # Llama a la función principal.