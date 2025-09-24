#!/usr/bin/env python3
"""
Calculador de Odometría para Robot Diferencial DiffBot

Este nodo implementa el cálculo de odometría para un robot diferencial basado en
las posiciones angulares de las ruedas. Utiliza la cinemática directa para estimar
la posición y orientación del robot en el espacio.

Autor: Sistema de Navegación DiffBot
Fecha: 2025
Versión: 1.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class DiffbotOdometry(Node):
    """
    Calculador de odometría para robot diferencial.
    
    Este nodo calcula la odometría del robot basándose en las posiciones angulares
    de las ruedas obtenidas del topic joint_states. Implementa la cinemática directa
    del robot diferencial y opcionalmente publica transformaciones TF.
    
    Parámetros:
        wheel_r (float): Radio de las ruedas en metros (default: 0.035)
        wheel_sep (float): Separación entre ruedas en metros (default: 0.135)
        left_wheel_joint (str): Nombre del joint de la rueda izquierda
        right_wheel_joint (str): Nombre del joint de la rueda derecha
        publish_tf (bool): Si publicar transformaciones TF (default: True)
    
    Topics:
        Suscripciones:
            /joint_states (sensor_msgs/JointState): Estados de las articulaciones
        
        Publicaciones:
            /odom (nav_msgs/Odometry): Información de odometría
            /tf (tf2_msgs/TFMessage): Transformaciones (si publish_tf=True)
    """
    def __init__(self):
        super().__init__('diffbot_odometry_node')

        # Parámetros
        self.declare_parameter('wheel_r', 0.035)
        self.declare_parameter('wheel_sep', 0.135)
        self.declare_parameter('left_wheel_joint', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint', 'right_wheel_joint')
        self.declare_parameter('publish_tf', True)  # nuevo parámetro ACTIVA / DESACTIVA TF
        
        # Parámetros de calibración de odometría
        self.declare_parameter('c_L', 1.0)  # Coeficiente de corrección rueda izquierda
        self.declare_parameter('c_R', 1.0)  # Coeficiente de corrección rueda derecha
        self.declare_parameter('b_actual', 0.135)  # Separación corregida entre ruedas

        self.wheel_r = self.get_parameter('wheel_r').get_parameter_value().double_value
        self.wheel_sep = self.get_parameter('wheel_sep').get_parameter_value().double_value
        self.left_wheel_name = self.get_parameter('left_wheel_joint').get_parameter_value().string_value
        self.right_wheel_name = self.get_parameter('right_wheel_joint').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        
        # Parámetros de calibración
        self.c_L = self.get_parameter('c_L').get_parameter_value().double_value
        self.c_R = self.get_parameter('c_R').get_parameter_value().double_value
        self.b_actual = self.get_parameter('b_actual').get_parameter_value().double_value

        # Estado inicial
        self.lwheel_ang_old = 0.0
        self.rwheel_ang_old = 0.0
        self.x_k = 0.0
        self.y_k = 0.0
        self.w_k = 0.0

        # Publisher y suscriptor
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(JointState, 'joint_states', self.sub_callback, 10)

        # Broadcaster de TF (solo si se activa)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"Nodo de odometría iniciado con parámetros:")
        self.get_logger().info(f"  Radio de rueda: {self.wheel_r} m")
        self.get_logger().info(f"  Separación nominal: {self.wheel_sep} m")
        self.get_logger().info(f"  Separación corregida: {self.b_actual} m")
        self.get_logger().info(f"  Rueda izquierda: {self.left_wheel_name}")
        self.get_logger().info(f"  Rueda derecha: {self.right_wheel_name}")
        self.get_logger().info(f"  Publicar TF: {self.publish_tf}")
        self.get_logger().info(f"  Calibración: c_L={self.c_L:.6f}, c_R={self.c_R:.6f}")

    def sub_callback(self, msg: JointState):
        """
        Callback que procesa los estados de las articulaciones y calcula la odometría.
        
        Implementa la cinemática directa del robot diferencial:
        1. Calcula los incrementos lineales de cada rueda
        2. Aplica la cinemática directa para obtener desplazamiento y rotación
        3. Actualiza la pose del robot
        4. Publica la odometría y opcionalmente las transformaciones TF
        
        Ecuaciones utilizadas:
        - dl_k = (theta_l_k - theta_l_k-1) * r
        - dr_k = (theta_r_k - theta_r_k-1) * r
        - dA_k = (dr_k + dl_k) / 2
        - Dw_k = (dr_k - dl_k) / L
        - x_k = x_k-1 + dA_k * cos(w_k-1)
        - y_k = y_k-1 + dA_k * sin(w_k-1)
        - w_k = w_k-1 + Dw_k
        
        Args:
            msg (sensor_msgs.msg.JointState): Mensaje con estados de articulaciones
        """
        # Extraer ángulos de las ruedas
        lwheel_ang, rwheel_ang = 0.0, 0.0
        for name, position in zip(msg.name, msg.position):
            if name == self.left_wheel_name:
                lwheel_ang = position
            elif name == self.right_wheel_name:
                rwheel_ang = position

        # Incrementos lineales de cada rueda con corrección de calibración
        dl_k = (lwheel_ang - self.lwheel_ang_old) * self.wheel_r * self.c_L
        dr_k = (rwheel_ang - self.rwheel_ang_old) * self.wheel_r * self.c_R

        # Cinemática directa diferencial con separación corregida
        dA_k = (dr_k + dl_k) / 2.0
        Dw_k = (dr_k - dl_k) / self.b_actual

        # Pose nueva (usar w_k anterior para x,y)
        x_k_new = self.x_k + dA_k * np.cos(self.w_k)
        y_k_new = self.y_k + dA_k * np.sin(self.w_k)
        w_k_new = self.w_k + Dw_k

        # Publicar TF si está activado
        if self.publish_tf:
            self.send_tf(x_k_new, y_k_new, w_k_new)

        # Publicar Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = x_k_new
        odom_msg.pose.pose.position.y = y_k_new
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = np.sin(w_k_new / 2.0)  # solo rotación en Z
        odom_msg.pose.pose.orientation.w = np.cos(w_k_new / 2.0)

        # Velocidades (opcional, se pueden calcular si es necesario)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.pub_odom.publish(odom_msg)

        # Log de debug
        self.get_logger().debug(f"Pose: x={x_k_new:.3f}, y={y_k_new:.3f}, theta={w_k_new:.3f}")

        # Actualizar estado
        self.lwheel_ang_old = lwheel_ang
        self.rwheel_ang_old = rwheel_ang
        self.x_k = x_k_new
        self.y_k = y_k_new
        self.w_k = w_k_new

    def send_tf(self, x, y, theta):
        """
        Publica la transformación entre los frames 'odom' y 'base_link'.
        
        Esta transformación permite a otros nodos conocer la posición del robot
        en el frame de odometría, facilitando la navegación y localización.
        
        Args:
            x (float): Posición X del robot en metros
            y (float): Posición Y del robot en metros
            theta (float): Orientación del robot en radianes
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(theta / 2.0)
        t.transform.rotation.w = np.cos(theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    """
    Función principal del nodo calculador de odometría.
    
    Inicializa el nodo ROS2, crea una instancia del calculador de odometría
    y mantiene el nodo activo hasta recibir una señal de interrupción.
    
    Args:
        args: Argumentos de línea de comandos (opcional)
    """
    # Inicialización del sistema ROS2
    rclpy.init(args=args)
    
    # Creación del nodo calculador de odometría
    node = DiffbotOdometry()
    
    try:
        # Mantener el nodo activo procesando callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Manejo de interrupción por teclado (Ctrl+C)
        pass
    finally:
        # Limpieza y finalización
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()