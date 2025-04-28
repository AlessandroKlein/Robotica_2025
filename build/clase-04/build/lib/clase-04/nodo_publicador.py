import rclpy

# Importar el tipo de mensaje String
from std_msgs.msg import String

def main(args=None):
    # 1. Inicialización
    rclpy.init(args=args)

    # 2. Creación de nodo
    # nodo = ...

    # 2.1 Creación de publisher
    # pub = ...

    # 2.2 Creación de mensaje
    msg = String()

    # 2.3 Programación de función de callback
    def timer_callback():
        # Completar el campo 'data' del mensaje 
        msg.data = 'Mensaje de prueba'

        # Publicar el mensaje
        pub.publish(msg)

    # 2.4 Creación del timer
    # ...

    # 3. Procesamiento de mensajes y callback
    rclpy.spin(nodo)

    # 4. Finalización 
    rclpy.shutdown()


if __name__ == '__main__':
    main()