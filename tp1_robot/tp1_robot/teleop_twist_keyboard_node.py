## Ejercicio 5

import rclpy # Importa la biblioteca de cliente de ROS2 para Python.
from rclpy.node import Node # Importa la clase Node, la base para crear todos los nodos.
from geometry_msgs.msg import Twist # Importa el mensaje Twist para comandos de velocidad.
from std_msgs.msg import Float64 # Importa el mensaje Float64 para las velocidades de las ruedas.
import sys # Proporciona acceso a variables y funciones mantenidas por el intérprete de Python.
import select # Permite esperar a que ocurran eventos de E/S en múltiples flujos de entrada/salida.
import termios # Define una interfaz para el control de E/S de la tty (terminal).
import tty # Contiene funciones para poner la tty en modo cbreak y raw.

class DiffBotController(Node): # Define una clase para el controlador del robot diferencial (cinemática inversa).
    def __init__(self): # Constructor de la clase.
        super().__init__('diffbot_controller') # Llama al constructor de la clase base y nombra el nodo.
        self.declare_parameter('wheel_separation', 0.08) # Declara un parámetro para la separación de las ruedas.
        self.declare_parameter('wheel_radius', 0.035) # Declara un parámetro para el radio de las ruedas.

        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value # Obtiene el valor de la separación de las ruedas.
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value # Obtiene el valor del radio de las ruedas.

        self.left_pub = self.create_publisher(Float64, 'left_wheel_cmd', 10) # Crea un publicador para la velocidad de la rueda izquierda.
        self.right_pub = self.create_publisher(Float64, 'right_wheel_cmd', 10) # Crea un publicador para la velocidad de la rueda derecha.
        self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10) # Se suscribe al tópico 'cmd_vel' para recibir comandos de velocidad.

    def twist_callback(self, msg): # Callback que se ejecuta al recibir un mensaje en 'cmd_vel'.
        v = msg.linear.x # Extrae la velocidad lineal.
        w = msg.angular.z # Extrae la velocidad angular.

        v_left = (v - (w * self.wheel_separation / 2)) / self.wheel_radius # Calcula la velocidad de la rueda izquierda.
        v_right = (v + (w * self.wheel_separation / 2)) / self.wheel_radius # Calcula la velocidad de la rueda derecha.

        self.left_pub.publish(Float64(data=v_left)) # Publica la velocidad de la rueda izquierda.
        self.right_pub.publish(Float64(data=v_right)) # Publica la velocidad de la rueda derecha.

class TeleopNode(Node): # Define la clase para el nodo de teleoperación.
    def __init__(self): # Constructor de la clase.
        super().__init__('teleop_twist_keyboard') # Llama al constructor de la clase base y nombra el nodo.
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # Crea un publicador para enviar mensajes Twist al tópico 'cmd_vel'.

    def send_velocity(self, linear, angular): # Método para enviar un comando de velocidad.
        msg = Twist() # Crea una instancia del mensaje Twist.
        msg.linear.x = linear # Establece la velocidad lineal.
        msg.angular.z = angular # Establece la velocidad angular.
        self.publisher_.publish(msg) # Publica el mensaje.

def getKey(): # Función para obtener una tecla presionada del teclado.
    tty.setraw(sys.stdin.fileno()) # Configura la terminal en modo "raw" para leer caracteres individuales.
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1) # Espera hasta 0.1 segundos para que haya datos en la entrada estándar.
    if rlist: # Si se presionó una tecla.
        key = sys.stdin.read(1) # Lee un solo carácter.
    else: # Si no se presionó ninguna tecla.
        key = '' # Retorna una cadena vacía.
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) # Restaura la configuración original de la terminal.
    return key # Devuelve la tecla presionada.

msg = """
Reading from the keyboard!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i : forward
, : backward
j : left
l : right
k : stop
CTRL-C to quit
""" # Mensaje de ayuda que se muestra al usuario.

moveBindings = { # Diccionario que mapea las teclas a los movimientos (velocidad lineal, velocidad angular).
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'm': (-1, 1),
    ',': (-1, 0),
    '.': (-1, -1),
}

settings = termios.tcgetattr(sys.stdin) # Guarda la configuración actual de la terminal para restaurarla después.

def main(): # Función principal.
    rclpy.init() # Inicializa rclpy.
    controller = DiffBotController() # Crea una instancia del nodo controlador (cinemática inversa).
    teleop = TeleopNode() # Crea una instancia del nodo de teleoperación.

    print(msg) # Imprime el mensaje de ayuda.
    try: # Inicia un bloque try para manejar excepciones.
        while True: # Bucle infinito para leer continuamente el teclado.
            key = getKey() # Obtiene la tecla presionada.
            if key in moveBindings: # Si la tecla está en el diccionario de movimientos.
                x = moveBindings[key][0] # Obtiene el componente de velocidad lineal.
                th = moveBindings[key][1] # Obtiene el componente de velocidad angular.
                teleop.send_velocity(x * 0.5, th * 0.5) # Envía el comando de velocidad (reducido a la mitad).
            elif key == '\x03': # Si la tecla es CTRL-C.
                break # Rompe el bucle para salir del programa.
            rclpy.spin_once(controller, timeout_sec=0.1) # Procesa los callbacks del nodo controlador.
    except Exception as e: # Captura cualquier otra excepción.
        print(e) # Imprime la excepción.
    finally: # Bloque que se ejecuta siempre al final.
        teleop.send_velocity(0.0, 0.0) # Envía una velocidad de cero para detener el robot.
        rclpy.shutdown() # Cierra rclpy.

if __name__ == '__main__': # Punto de entrada del script.
    main() # Llama a la función principal.