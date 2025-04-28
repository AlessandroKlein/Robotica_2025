import rclpy

# Importar el tipo de mensaje String
from std_msgs.msg import String

def main(args=None):
    # 1. Inicialización
    rclpy.init(args=args)

    # 2. Creación de nodo
    nodo = rclpy.create_node('suscriptor')

    # 2.1 Programación de función de callback
    def sub_callback(msg):
        # Mostrar el mensaje en consola
        nodo.get_logger().info('Recibí: "%s"' % msg.data)   #Mensaje de Info
        #nodo.get_logger().warn('Recibí: "%s"' % msg.data) # Para notificacion
        #nodo.get_logger().error('Recibí: "%s"' % msg.data) # Para errores
        #nodo.get_logger().debug('Recibí: "%s"' % msg.data) # Menos severos Para mensaje de depuracion

    # 2.2 Creación de suscriptor
    sub = nodo.create_subscription(String, 'chat', sub_callback, 10)

    # 3. Procesamiento de mensajes y callback
    rclpy.spin(nodo)

    # 4. Finalización 
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
