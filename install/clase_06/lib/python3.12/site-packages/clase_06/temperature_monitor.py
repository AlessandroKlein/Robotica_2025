import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature

class TemperatureMonitor(Node):
    """
    Nodo de ROS2 que supervisa la temperatura recibida de un topic y genera alertas si se supera un umbral definido.
    Permite configurar la unidad de temperatura (Celsius o Fahrenheit) y el umbral de alarma mediante parámetros.
    """

    def __init__(self):
        # Inicializar el nodo con el nombre 'temperature_monitor'
        super().__init__('temperature_monitor')

        # Declarar parámetros configurables
        self.declare_parameter('alarm_threshold', 30.0)   # Umbral de alarma por defecto: 30°C
        self.declare_parameter('temperature_unit', 'C')   # Unidad de temperatura por defecto: Celsius ('C')

        # Obtener los valores de los parámetros
        self.alarm_threshold = self.get_parameter('alarm_threshold').get_parameter_value().double_value
        self.temperature_unit = self.get_parameter('temperature_unit').get_parameter_value().string_value

        # Crear una suscripción al topic 'temperature' (tipo sensor_msgs/Temperature)
        self.subscription = self.create_subscription(
            Temperature,             # Tipo de mensaje esperado
            'temperature',            # Nombre del topic
            self.listener_callback,   # Función de callback que maneja los mensajes recibidos
            10                        # Tamaño de la cola (queue size)
        )

        self.get_logger().info(
            f'Monitor de temperatura iniciado. Umbral de alarma: {self.alarm_threshold}°{self.temperature_unit.upper()}'
        )

    def listener_callback(self, msg):
        """
        Función que se ejecuta cada vez que llega un nuevo mensaje de temperatura.
        Compara la temperatura con el umbral y genera mensajes de alerta o informativos.
        """
        temperature = msg.temperature

        # Si se especificó la unidad Fahrenheit, convertir desde Celsius
        if self.temperature_unit.upper() == 'F':
            temperature = temperature * 9.0 / 5.0 + 32.0

        # Comparar la temperatura contra el umbral de alarma
        if temperature > self.alarm_threshold:
            self.get_logger().warn(
                f'¡Alarma! Temperatura crítica: {temperature:.2f}°{self.temperature_unit.upper()}'
            )
        else:
            self.get_logger().info(
                f'Temperatura normal: {temperature:.2f}°{self.temperature_unit.upper()}'
            )

def main(args=None):
    """
    Función principal que inicializa el entorno de ROS2, crea el nodo de monitoreo y mantiene el proceso activo.
    """
    rclpy.init(args=args)
    monitor = TemperatureMonitor()
    rclpy.spin(monitor)  # Mantener el nodo en ejecución escuchando mensajes

    # Cuando se detiene el nodo, liberar recursos correctamente
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
