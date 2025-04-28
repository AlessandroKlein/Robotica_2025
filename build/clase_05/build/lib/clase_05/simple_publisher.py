import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        
        # Crear un publisher
        self.publisher_ = self.create_publisher(String, 'mensaje_texto', 10)
        
        # Crear un timer para enviar mensajes periódicamente
        timer_period = 2.0  # segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Nodo publisher iniciado.')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hola desde el publisher!'
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Mensaje publicado: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    publisher = SimplePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
