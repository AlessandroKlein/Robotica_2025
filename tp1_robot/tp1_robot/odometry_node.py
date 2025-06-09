#!/usr/bin/env python3
# tp1_robot/odometry_node.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import transforms3d as tf3d

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Parámetros configurables
        self.declare_parameter('wheel_radius', 0.035)  # m (de diffbot.xacro)
        self.declare_parameter('wheel_separation', 0.23)  # m (de diffbot.xacro)
        self.declare_parameter('publish_tf', True)  # Publicar transformación TF
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # Validar parámetros
        if self.wheel_radius <= 0 or self.wheel_separation <= 0:
            self.get_logger().error('Parámetros inválidos: radio o separación de ruedas <= 0')
            raise ValueError('Parámetros de ruedas deben ser positivos')
        
        # Variables de estado
        self.last_left_pos = 0.0
        self.last_right_pos = 0.0
        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Suscripción a /joint_states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publicador de odometría
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Inicializar transform broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Nodo de odometría iniciado')
        self.get_logger().info(f'Radio de rueda: {self.wheel_radius} m, Separación: {self.wheel_separation} m')

    def joint_state_callback(self, msg):
        try:
            # Extraer posiciones de ruedas
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
            
            left_pos = msg.position[left_idx]
            right_pos = msg.position[right_idx]
            
        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Error al extraer posiciones de ruedas: {str(e)}')
            return
        
        # Calcular tiempo transcurrido
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return  # Evitar divisiones por cero
            
        # Calcular velocidades lineales y angulares
        delta_left = left_pos - self.last_left_pos
        delta_right = right_pos - self.last_right_pos
        
        # Velocidades lineales de las ruedas
        v_left = delta_left / dt
        v_right = delta_right / dt
        
        # Velocidad lineal y angular del robot
        v = (self.wheel_radius * (v_left + v_right)) / 2
        w = (self.wheel_radius * (v_right - v_left)) / self.wheel_separation
        
        # Actualizar posición (integración simple)
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        
        # Actualizar valores anteriores
        self.last_left_pos = left_pos
        self.last_right_pos = right_pos
        self.last_time = current_time
        
        # Publicar mensaje de odometría
        self.publish_odometry(current_time)
        
        # Publicar transformación TF si está activado
        if self.publish_tf:
            self.broadcast_transform(current_time)

    def publish_odometry(self, timestamp):
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Posición
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientación (euler a quaternion)
        quat = tf3d.euler.euler2quat(0, 0, self.theta)
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        odom.pose.pose.orientation.w = quat[0]
        
        # Velocidad (lineal y angular)
        odom.twist.twist.linear.x = self.wheel_radius * (self.last_right_pos - self.last_left_pos) / 2
        odom.twist.twist.angular.z = self.wheel_radius * (self.last_right_pos - self.last_left_pos) / self.wheel_separation
        
        # Covarianzas (valores por defecto, pueden ajustarse)
        odom.pose.covariance = [0.0]*36
        odom.twist.covariance = [0.0]*36
        
        self.odom_pub.publish(odom)

    def broadcast_transform(self, timestamp):
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        quat = tf3d.euler.euler2quat(0, 0, self.theta)
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]
        t.transform.rotation.w = quat[0]
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdometryNode()
    
    try:
        rclpy.spin(odom_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Detener el robot al salir
        odom_node.get_logger().info('Nodo de odometría detenido')
        odom_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()