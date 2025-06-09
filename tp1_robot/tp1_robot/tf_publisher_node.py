#!/usr/bin/env python3
# tp1_robot/tf_publisher_node.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import transforms3d as tf3d

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher_node')
        
        # Parámetros configurables
        self.declare_parameter('publish_tf', True)  # Activar/desactivar TF
        self.declare_parameter('odom_topic', '/odom')  # Fuente de odometría
        self.declare_parameter('frame_id', 'odom')  # Frame padre
        self.declare_parameter('child_frame_id', 'base_link')  # Frame hijo
        
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        
        # Validar parámetros
        if not self.publish_tf:
            self.get_logger().info('Publicación de TF desactivada')
            return
        
        # Suscripción a odometría
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        
        # Inicializar transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info(f'Publicando transformación {self.frame_id} -> {self.child_frame_id}')
        self.get_logger().info(f'Fuente de odometría: {self.odom_topic}')

    def odom_callback(self, msg):
        try:
            # Extraer datos de odometría
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z  # Debería ser 0.0
            
            # Extraer orientación (quaternion)
            quat = msg.pose.pose.orientation
            quat_list = [quat.x, quat.y, quat.z, quat.w]
            
        except Exception as e:
            self.get_logger().error(f'Error al procesar mensaje de odometría: {str(e)}')
            return
        
        # Crear transformación
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.child_frame_id
        
        # Posición
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z  # 0.0 en robots 2D
        
        # Orientación
        transform.transform.rotation.x = quat.x
        transform.transform.rotation.y = quat.y
        transform.transform.rotation.z = quat.z
        transform.transform.rotation.w = quat.w
        
        # Publicar transformación
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TFPublisher()
    
    try:
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Detener el nodo
        tf_publisher.get_logger().info('Nodo de transformaciones TF detenido')
        tf_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()