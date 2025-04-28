import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')

        # Crear un subscriber
        self.subscription = self.create_subscription(
            String,
            'mensaje_texto',
            self.listener_callback,
            10)

        self.get_logger().info('Nodo subscriber iniciado.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Mensaje recibido: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    subscriber = SimpleSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
