#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import cv_bridge
import numpy as np

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        
        # Declarar parámetros configurables
        self.declare_parameter('hsv_lower_h', 0)
        self.declare_parameter('hsv_lower_s', 0)
        self.declare_parameter('hsv_lower_v', 0)
        self.declare_parameter('hsv_upper_h', 180)
        self.declare_parameter('hsv_upper_s', 255)
        self.declare_parameter('hsv_upper_v', 50)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('search_angular_speed', 0.3)  # Velocidad angular para búsqueda
        self.declare_parameter('min_line_area', 100)  # Área mínima para considerar línea válida
        
        # Leer parámetros
        self.hsv_lower_h = self.get_parameter('hsv_lower_h').get_parameter_value().integer_value
        self.hsv_lower_s = self.get_parameter('hsv_lower_s').get_parameter_value().integer_value
        self.hsv_lower_v = self.get_parameter('hsv_lower_v').get_parameter_value().integer_value
        self.hsv_upper_h = self.get_parameter('hsv_upper_h').get_parameter_value().integer_value
        self.hsv_upper_s = self.get_parameter('hsv_upper_s').get_parameter_value().integer_value
        self.hsv_upper_v = self.get_parameter('hsv_upper_v').get_parameter_value().integer_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.min_line_area = self.get_parameter('min_line_area').get_parameter_value().integer_value
        
        # Variables de estado para manejo de pérdida de línea
        self.last_known_direction = 0.0  # Última dirección conocida de la línea
        self.line_lost_counter = 0  # Contador de frames sin línea
        self.max_lost_frames = 5  # Máximo de frames antes de activar búsqueda
        
        # Crear el puente CV
        self.bridge = cv_bridge.CvBridge()
        
        # Subscriptor a la cámara
        self.sub = self.create_subscription(
            Image, 
            'camera', 
            self.sub_callback, 
            10
        )
        
        # Publisher para comandos de velocidad
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10
        )
        
        # Modo headless - sin ventana de visualización
        self.headless_mode = True
        
        self.get_logger().info('Line Detector iniciado con parámetros configurables')
        self.get_logger().info(f'HSV Lower: [{self.hsv_lower_h}, {self.hsv_lower_s}, {self.hsv_lower_v}]')
        self.get_logger().info(f'HSV Upper: [{self.hsv_upper_h}, {self.hsv_upper_s}, {self.hsv_upper_v}]')
        self.get_logger().info(f'Linear Speed: {self.linear_speed}, Angular Gain: {self.angular_gain}')

    def sub_callback(self, msg: Image):
        try:
            # 0. Captura desde ROS2
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 1. Preprocesamiento - reducir resolución
            image = cv2.resize(image, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_CUBIC)
            height, width = image.shape[:2]
            
            # 2. Detección del camino - conversión a HSV y máscara
            image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Definir rango para detectar línea usando parámetros configurables
            lower = np.array([self.hsv_lower_h, self.hsv_lower_s, self.hsv_lower_v])
            upper = np.array([self.hsv_upper_h, self.hsv_upper_s, self.hsv_upper_v])
            
            # Aplicar máscara
            mask = cv2.inRange(image_hsv, lower, upper)
            
            # 3. Estimación del curso - encontrar centroide
            M = cv2.moments(mask)
            
            if M['m00'] > self.min_line_area:
                # Línea detectada - resetear contador de pérdida
                self.line_lost_counter = 0
                
                # Calcular centroide
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                # Dibujar centroide en la imagen
                cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
                
                # Control de velocidad angular usando parámetros configurables
                # e = (W/2) - cx
                # alpha = e / (W/2) = 1 - (2*cx/W)
                alpha = 1.0 - (2.0 * cx / width)
                angular_z = alpha * self.angular_gain
                
                # Guardar última dirección conocida
                self.last_known_direction = angular_z
                
                # Crear mensaje Twist
                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = angular_z
                
                # Publicar comando
                self.cmd_vel_pub.publish(twist)
                
                # Log de información (solo cada 10 frames para reducir spam)
                if self.line_lost_counter % 10 == 0:
                    self.get_logger().debug(
                        f'Línea detectada - Centroide: ({cx}, {cy}), Alpha: {alpha:.3f}, Angular: {angular_z:.3f}'
                    )
            else:
                # No se detectó línea - incrementar contador
                self.line_lost_counter += 1
                
                if self.line_lost_counter <= self.max_lost_frames:
                    # Continuar en la última dirección conocida con velocidad reducida
                    twist = Twist()
                    twist.linear.x = self.linear_speed * 0.5  # Reducir velocidad lineal
                    twist.angular.z = self.last_known_direction * 0.8  # Mantener dirección con factor de amortiguación
                    
                    self.cmd_vel_pub.publish(twist)
                    
                    if self.line_lost_counter == 1:  # Solo mostrar warning una vez
                        self.get_logger().warn(f'Línea perdida - continuando en última dirección conocida (frame {self.line_lost_counter}/{self.max_lost_frames})')
                else:
                    # Activar modo de búsqueda - girar en la dirección de la última línea conocida
                    twist = Twist()
                    twist.linear.x = 0.1  # Velocidad lineal muy baja
                    # Girar en la dirección de la última línea conocida, pero amplificado
                    search_direction = 1.0 if self.last_known_direction >= 0 else -1.0
                    twist.angular.z = search_direction * self.search_angular_speed
                    
                    self.cmd_vel_pub.publish(twist)
                    
                    if self.line_lost_counter % 20 == 0:  # Log cada 20 frames para evitar spam
                        self.get_logger().warn(f'Modo búsqueda activo - buscando línea (frame {self.line_lost_counter})')
            
            # En modo headless, solo procesamos sin mostrar
            if not self.headless_mode:
                # Combinar imagen original y máscara para visualización
                mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                combined = np.hstack((image, mask_colored))
                
                cv2.imshow("Robot_camera", combined)
                cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error en procesamiento de imagen: {str(e)}')

    def destroy_node(self):
        if not self.headless_mode:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()