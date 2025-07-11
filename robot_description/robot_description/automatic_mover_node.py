## Ejercicio 6  -  8


# robot_description/automatic_mover_node.py
import rclpy # Importa la biblioteca de cliente de ROS2 para Python.
from rclpy.node import Node # Importa la clase Node, la base para crear nodos.
from geometry_msgs.msg import Twist # Importa el mensaje Twist para comandos de velocidad.
import time # Importa la biblioteca de tiempo de Python (aunque no se usa directamente a favor del reloj de ROS).

class AutomaticMover(Node): # Define la clase del nodo para el movimiento automático.
    def __init__(self): # Constructor de la clase.
        super().__init__('automatic_mover') # Llama al constructor de la clase base y nombra el nodo.
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # Crea un publicador para enviar comandos de velocidad al tópico 'cmd_vel'.

    def move_straight(self, duration=5.0, linear_speed=0.2): # Define un método para que el robot se mueva en línea recta.
        msg = Twist() # Crea una nueva instancia del mensaje Twist.
        msg.linear.x = linear_speed # Establece la velocidad lineal en el eje x.
        self._publish_for_duration(msg, duration) # Llama al método para publicar el mensaje durante un tiempo determinado.

    def turn(self, duration=3.0, angular_speed=0.5): # Define un método para que el robot gire sobre su propio eje.
        msg = Twist() # Crea una nueva instancia del mensaje Twist.
        msg.angular.z = angular_speed # Establece la velocidad angular en el eje z.
        self._publish_for_duration(msg, duration) # Llama al método para publicar el mensaje durante un tiempo determinado.

    def stop(self): # Define un método para detener el robot.
        msg = Twist() # Crea un mensaje Twist vacío (velocidades a cero).
        self.publisher_.publish(msg) # Publica el mensaje de detención.

    def _publish_for_duration(self, msg, duration): # Método privado para publicar un mensaje repetidamente durante una duración específica.
        start_time = self.get_clock().now() # Obtiene el tiempo actual del reloj de ROS.
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < duration: # Bucle que se ejecuta durante el tiempo especificado.
            self.publisher_.publish(msg) # Publica el mensaje de velocidad.
            rclpy.spin_once(self, timeout_sec=0.1) # Permite que ROS procese otros eventos, como callbacks, durante un breve período.

def main(): # Define la función principal del programa.
    rclpy.init() # Inicializa el sistema de cliente de ROS2.
    mover = AutomaticMover() # Crea una instancia del nodo de movimiento automático.
    try: # Inicia un bloque try para manejar posibles excepciones.
        mover.turn(duration=2.0) # Llama al método para girar durante 2 segundos.
        mover.move_straight(duration=3.0) # Llama al método para moverse recto durante 3 segundos.
        mover.turn(duration=2.0) # Llama al método para girar durante 2 segundos.
        mover.move_straight(duration=3.0) # Llama al método para moverse recto de nuevo durante 3 segundos.
        mover.stop() # Detiene el robot.
    except KeyboardInterrupt: # Captura la excepción que ocurre cuando el usuario presiona Ctrl+C.
        pass # No hace nada en caso de interrupción del teclado, simplemente sale del bloque try.
    finally: # Bloque de código que se ejecuta siempre, haya o no una excepción.
        mover.stop() # Se asegura de que el robot esté detenido antes de terminar.
        rclpy.shutdown() # Libera los recursos utilizados por rclpy.

if __name__ == '__main__': # Punto de entrada del script de Python.
    main() # Llama a la función principal para ejecutar el nodo.