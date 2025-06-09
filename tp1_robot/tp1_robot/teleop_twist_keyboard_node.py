#!/usr/bin/env python3
# tp1_robot/teleop_twist_keyboard_node.py

import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class KeyPoller:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.new = termios.tcgetattr(self.fd)
        self.old = termios.tcgetattr(self.fd)
        self.new[3] = self.new[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new)
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old)

    def poll(self, timeout=None):
        import select
        drdy, _, _ = select.select([sys.stdin], [], [], timeout)
        if drdy:
            return sys.stdin.read(1)
        else:
            return None

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        
        # Parámetros configurables
        self.declare_parameter('linear_speed', 0.5)   # m/s
        self.declare_parameter('angular_speed', 1.0)  # rad/s
        self.declare_parameter('acceleration_steps', 5)  # Pasos para acelerar
        self.declare_parameter('deadman_timeout', 0.5)  # Segundos antes de detener
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.acceleration_steps = max(1, self.get_parameter('acceleration_steps').value)
        self.deadman_timeout = self.get_parameter('deadman_timeout').value
        
        # Estados de movimiento
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_key_time = self.get_clock().now()
        
        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.left_cmd_pub = self.create_publisher(Float64MultiArray, '/left_velocity_controller/commands', 10)
        self.right_cmd_pub = self.create_publisher(Float64MultiArray, '/right_velocity_controller/commands', 10)
        
        # Radio de rueda y separación (de diffbot.xacro)
        self.wheel_radius = 0.035  # m
        self.wheel_separation = 0.23  # m
        
        # Temporizador para actualización suave
        self.timer = self.create_timer(0.1, self.update_movement)
        
        self.get_logger().info('Nodo de teleoperación iniciado')
        self.get_logger().info('Use las teclas WASD para controlar el robot')
        self.get_logger().info('Presione Q/E para ajustar velocidad')
        self.get_logger().info('Presione CTRL+C para salir')

    def update_movement(self):
        now = self.get_clock().now()
        
        # Verificar timeout del deadman switch
        if (now - self.last_key_time).nanoseconds / 1e9 > self.deadman_timeout:
            self.target_linear = 0.0
            self.target_angular = 0.0
        
        # Aceleración gradual
        if self.current_linear < self.target_linear:
            self.current_linear = min(self.current_linear + self.linear_speed/self.acceleration_steps, self.target_linear)
        elif self.current_linear > self.target_linear:
            self.current_linear = max(self.current_linear - self.linear_speed/self.acceleration_steps, self.target_linear)
            
        if self.current_angular < self.target_angular:
            self.current_angular = min(self.current_angular + self.angular_speed/self.acceleration_steps, self.target_angular)
        elif self.current_angular > self.target_angular:
            self.current_angular = max(self.current_angular - self.angular_speed/self.acceleration_steps, self.target_angular)
        
        # Publicar comando en /cmd_vel
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.cmd_vel_pub.publish(twist)
        
        # Convertir a comandos de ruedas
        left_vel, right_vel = self.calculate_wheel_velocities(twist)
        self.publish_wheel_commands(left_vel, right_vel)

    def calculate_wheel_velocities(self, twist):
        # Calcular velocidades de ruedas (cinemática directa)
        left_vel = (twist.linear.x - (twist.angular.z * self.wheel_separation / 2)) / self.wheel_radius
        right_vel = (twist.linear.x + (twist.angular.z * self.wheel_separation / 2)) / self.wheel_radius
        return left_vel, right_vel

    def publish_wheel_commands(self, left_vel, right_vel):
        left_cmd = Float64MultiArray()
        right_cmd = Float64MultiArray()
        
        left_cmd.data = [left_vel]
        right_cmd.data = [right_vel]
        
        self.left_cmd_pub.publish(left_cmd)
        self.right_cmd_pub.publish(right_cmd)

    def key_loop(self):
        with KeyPoller() as key_poller:
            while rclpy.ok():
                key = key_poller.poll(timeout=0.1)
                if key is None:
                    continue
                
                self.last_key_time = self.get_clock().now()
                
                if key == 'w':
                    self.target_linear = min(self.linear_speed, self.target_linear + self.linear_speed/self.acceleration_steps)
                elif key == 's':
                    self.target_linear = max(-self.linear_speed, self.target_linear - self.linear_speed/self.acceleration_steps)
                elif key == 'a':
                    self.target_angular = min(self.angular_speed, self.target_angular + self.angular_speed/self.acceleration_steps)
                elif key == 'd':
                    self.target_angular = max(-self.angular_speed, self.target_angular - self.angular_speed/self.acceleration_steps)
                elif key == 'q':
                    self.linear_speed = max(0.1, self.linear_speed - 0.1)
                    self.angular_speed = max(0.1, self.angular_speed - 0.1)
                    self.get_logger().info(f'Velocidad ajustada: {self.linear_speed:.1f} m/s, {self.angular_speed:.1f} rad/s')
                elif key == 'e':
                    self.linear_speed = min(2.0, self.linear_speed + 0.1)
                    self.angular_speed = min(3.0, self.angular_speed + 0.1)
                    self.get_logger().info(f'Velocidad ajustada: {self.linear_speed:.1f} m/s, {self.angular_speed:.1f} rad/s')
                elif key == '\x03':  # CTRL+C
                    break

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopTwistKeyboard()
    
    # Crear hilo para lectura de teclado
    key_thread = threading.Thread(target=teleop_node.key_loop)
    key_thread.daemon = True
    key_thread.start()
    
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Detener el robot
        zero_twist = Twist()
        teleop_node.cmd_vel_pub.publish(zero_twist)
        teleop_node.publish_wheel_commands(0.0, 0.0)
        teleop_node.get_logger().info('Robot detenido')
        teleop_node.destroy_node()
        rclpy.shutdown()
        key_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()