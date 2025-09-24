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
        
        # Leer parámetros
        self.hsv_lower_h = self.get_parameter('hsv_lower_h').get_parameter_value().integer_value
        self.hsv_lower_s = self.get_parameter('hsv_lower_s').get_parameter_value().integer_value
        self.hsv_lower_v = self.get_parameter('hsv_lower_v').get_parameter_value().integer_value
        self.hsv_upper_h = self.get_parameter('hsv_upper_h').get_parameter_value().integer_value
        self.hsv_upper_s = self.get_parameter('hsv_upper_s').get_parameter_value().integer_value
        self.hsv_upper_v = self.get_parameter('hsv_upper_v').get_parameter_value().integer_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
        
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
            
            if M['m00'] > 0:
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
                
                # Crear mensaje Twist
                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = angular_z
                
                # Publicar comando
                self.cmd_vel_pub.publish(twist)
                
                # Log de información
                self.get_logger().info(
                    f'Centroide: ({cx}, {cy}), Alpha: {alpha:.3f}, Angular: {angular_z:.3f}'
                )
            else:
                # No se detectó línea, detener el robot
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
                self.get_logger().warn('No se detectó línea')
            
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