import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np

class DiffbotOdometry(Node):
    def __init__(self):
        super().__init__('diffbot_odometry_node')

        # Parámetro de radio y separación de ruedas
        self.declare_parameter('wheel_separation', 0.135)
        self.declare_parameter('wheel_radius', 0.07/2)

        # Parámetro para el nombre de las juntas
        self.declare_parameter('left_wheel_name', 'front_left_wheel_joint')
        self.declare_parameter('right_wheel_name', 'front_right_wheel_joint')

        self.wheel_sep = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.left_wheel_name = self.get_parameter('left_wheel_name').get_parameter_value().string_value
        self.right_wheel_name = self.get_parameter('right_wheel_name').get_parameter_value().string_value

        # Crear el suscriptor al topic joint_states
        self.sub = self.create_subscription(JointState, 'joint_states', self.sub_callback, 10)

        # Crear el publisher de la odometría
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)

        # Variables de estado de la odometría
        self.x_k = 0.0
        self.y_k = 0.0
        self.w_k = 0.0
        
        # Posiciones angulares anteriores de las ruedas
        self.lwheel_ang_old = 0.0
        self.rwheel_ang_old = 0.0
        self.first_callback = True

    def sub_callback(self, msg: JointState):
        # Chequear si el primer elemento es el joint
        # de la rueda izquierda o derecha
        lwheel_ang, rwheel_ang = 0.0, 0.0
        for name, position in zip(msg.name, msg.position):
            if name == self.left_wheel_name:
                lwheel_ang = position
            if name == self.right_wheel_name:
                rwheel_ang = position
        
        # Saltar el primer callback para inicializar posiciones anteriores
        if self.first_callback:
            self.lwheel_ang_old = lwheel_ang
            self.rwheel_ang_old = rwheel_ang
            self.first_callback = False
            return
        
        # Cálculo de la distancia recorrida
        dl_k = (lwheel_ang - self.lwheel_ang_old) * self.wheel_r
        dr_k = (rwheel_ang - self.rwheel_ang_old) * self.wheel_r
        
        dA_k = (dr_k + dl_k) / 2
        Dw_k = (dr_k - dl_k) / self.wheel_sep

        # Cálculo de la odometría (pose)
        x_k_new = self.x_k + dA_k * np.cos(self.w_k)
        y_k_new = self.y_k + dA_k * np.sin(self.w_k)
        w_k_new = self.w_k + Dw_k

        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = x_k_new
        odom_msg.pose.pose.position.y = y_k_new
        
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = np.sin(w_k_new/2)
        odom_msg.pose.pose.orientation.w = np.cos(w_k_new/2)
        
        # Publicar
        self.pub_odom.publish(odom_msg)

        # Actualizar valores
        self.lwheel_ang_old = lwheel_ang
        self.rwheel_ang_old = rwheel_ang
        self.x_k = x_k_new
        self.y_k = y_k_new
        self.w_k = w_k_new

def main(args=None):
    rclpy.init(args=args)
    diffbot_odometry = DiffbotOdometry()
    try:
        rclpy.spin(diffbot_odometry)
    except KeyboardInterrupt:
        pass
    finally:
        diffbot_odometry.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()