import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class DiffbotOdometry(Node):
    def __init__(self):
        super().__init__('diffbot_odometry_node')

        # Parámetros
        self.declare_parameter('wheel_r', 0.035)
        self.declare_parameter('wheel_sep', 0.135)
        self.declare_parameter('left_wheel_joint', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint', 'right_wheel_joint')
        
        # Parámetro para activar la publicación de las transformaciones
        self.declare_parameter('publish_tf', True)

        self.wheel_r = self.get_parameter('wheel_r').get_parameter_value().double_value
        self.wheel_sep = self.get_parameter('wheel_sep').get_parameter_value().double_value
        self.left_wheel_name = self.get_parameter('left_wheel_joint').get_parameter_value().string_value
        self.right_wheel_name = self.get_parameter('right_wheel_joint').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

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

    def sub_callback(self, msg: JointState):
        # Extraer ángulos de las ruedas
        lwheel_ang, rwheel_ang = 0.0, 0.0
        for name, position in zip(msg.name, msg.position):
            if name == self.left_wheel_name:
                lwheel_ang = position
            elif name == self.right_wheel_name:
                rwheel_ang = position

        # Incrementos lineales de cada rueda
        dl_k = (lwheel_ang - self.lwheel_ang_old) * self.wheel_r
        dr_k = (rwheel_ang - self.rwheel_ang_old) * self.wheel_r

        # Cinemática directa diferencial
        dA_k = (dr_k + dl_k) / 2.0
        Dw_k = (dr_k - dl_k) / self.wheel_sep

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
        odom_msg.pose.pose.orientation.z = np.sin(w_k_new / 2.0)  # ✅ solo rotación en Z
        odom_msg.pose.pose.orientation.w = np.cos(w_k_new / 2.0)

        self.pub_odom.publish(odom_msg)

        # Actualizar estado
        self.lwheel_ang_old = lwheel_ang
        self.rwheel_ang_old = rwheel_ang
        self.x_k = x_k_new
        self.y_k = y_k_new
        self.w_k = w_k_new
    
    def send_tf(self, x, y, theta):
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
    rclpy.init(args=args)
    nodo = DiffbotOdometry()
    try:
        rclpy.spin(nodo)
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()