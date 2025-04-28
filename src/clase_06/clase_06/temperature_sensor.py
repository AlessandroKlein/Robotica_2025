import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import random

class TemperatureSensor(Node):
    """
    Nodo de ROS2 que simula un sensor de temperatura.
    Publica periódicamente valores de temperatura generados aleatoriamente alrededor de una base configurable.
    """

    def __init__(self):
        # Inicializar el nodo con el nombre 'temperature_sensor'
        super().__init__('temperature_sensor')

        # Declarar parámetros que permiten configurar el comportamiento del sensor
        self.declare_parameter('base_temperature', 25.0)  # Temperatura base sobre la cual se aplicarán variaciones
        self.declare_parameter('max_variation', 5.0)       # Máxima variación aleatoria permitida
        self.declare_parameter('publish_rate', 1)          # Frecuencia de publicación (mensajes por segundo)

        # Obtener los valores de los parámetros declarados
        self.base_temperature = self.get_parameter('base_temperature').get_parameter_value().double_value
        self.max_variation = self.get_parameter('max_variation').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value

        # Crear un publicador que enviará mensajes de tipo Temperature en el topic 'temperature'
        self.publisher_ = self.create_publisher(Temperature, 'temperature', 10)

        # Configurar un temporizador para ejecutar una función periódicamente según la frecuencia indicada
        timer_period = 1.0 / self.publish_rate  # Calcular el período en segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Sensor de temperatura iniciado: '
            f'Base = {self.base_temperature}°C, '
            f'Variación máxima = ±{self.max_variation}°C, '
            f'Frecuencia = {self.publish_rate} Hz'
        )

    def timer_callback(self):
        """
        Función ejecutada cada vez que el temporizador se activa.
        Genera un nuevo valor de temperatura y lo publica en el topic.
        """
        # Generar un valor de temperatura simulando una variación aleatoria
        temperature_value = self.base_temperature + random.uniform(-self.max_variation, self.max_variation)

        # Crear un mensaje de tipo Temperature
        msg = Temperature()
        msg.temperature = temperature_value

        # Completar el encabezado del mensaje con la marca de tiempo actual y un identificador de marco
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_frame'

        # Publicar el mensaje en el topic
        self.publisher_.publish(msg)

        # Registrar la temperatura generada para depuración o monitoreo
        self.get_logger().info(f'Temperatura generada: {temperature_value:.2f} °C')

def main(args=None):
    """
    Función principal que inicializa el sistema ROS2, crea el nodo de sensor de temperatura
    y mantiene el programa activo hasta su finalización.
    """
    rclpy.init(args=args)
    sensor = TemperatureSensor()
    rclpy.spin(sensor)  # Mantener el nodo en ejecución para seguir publicando datos

    # Finalizar adecuadamente liberando los recursos
    sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
