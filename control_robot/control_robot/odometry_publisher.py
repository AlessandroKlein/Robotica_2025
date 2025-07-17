import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
from rclpy.time import Time
from rclpy.duration import Duration

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Declare and get parameters for robot dimensions and joint names
        self.declare_parameter('wheel_radius', 0.07) # Default value from URDF
        self.declare_parameter('wheel_separation', 0.135) # Default value from URDF
        self.declare_parameter('left_wheel_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint_name', 'right_wheel_joint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.b = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.left_wheel_joint_name = self.get_parameter('left_wheel_joint_name').get_parameter_value().string_value
        self.right_wheel_joint_name = self.get_parameter('right_wheel_joint_name').get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value

        self.get_logger().info(f'OdometryPublisher initialized with r={self.r}, b={self.b}')
        self.get_logger().info(f'Joints: Left="{self.left_wheel_joint_name}", Right="{self.right_wheel_joint_name}"')
        self.get_logger().info(f'Frames: Odom="{self.odom_frame_id}", Base="{self.base_frame_id}"')


        # Subscribers
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states', # Topic publicado por JointStateBroadcaster
            self.joint_state_callback,
            10
        )
        self.joint_state_subscription # Prevent unused variable warning

        # Publishers
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None
        
        # Store last wheel velocities to detect changes
        self.last_left_wheel_velocity = 0.0
        self.last_right_wheel_velocity = 0.0
        self.first_joint_state_received = False

    def joint_state_callback(self, msg):
        current_time = self.get_clock().now()

        left_wheel_velocity = 0.0
        right_wheel_velocity = 0.0
        
        # Find velocities for the specific joints
        try:
            left_index = msg.name.index(self.left_wheel_joint_name)
            left_wheel_velocity = msg.velocity[left_index]
        except ValueError:
            # self.get_logger().warn(f'Left wheel joint "{self.left_wheel_joint_name}" not found in JointState message.')
            pass # Keep previous velocity if not found or warn once
        
        try:
            right_index = msg.name.index(self.right_wheel_joint_name)
            right_wheel_velocity = msg.velocity[right_index]
        except ValueError:
            # self.get_logger().warn(f'Right wheel joint "{self.right_wheel_joint_name}" not found in JointState message.')
            pass # Keep previous velocity if not found or warn once

        # Initialize last_time on the first callback
        if not self.first_joint_state_received:
            self.last_time = current_time
            self.last_left_wheel_velocity = left_wheel_velocity
            self.last_right_wheel_velocity = right_wheel_velocity
            self.first_joint_state_received = True
            return # Skip odometry calculation for the very first message

        dt_ns = (current_time - self.last_time).nanoseconds
        dt = dt_ns / 1e9 # Convert nanoseconds to seconds

        if dt <= 0.0: # Avoid division by zero or negative time
            return

        # Calculate average velocities for better accuracy over the interval
        avg_left_wheel_velocity = (left_wheel_velocity + self.last_left_wheel_velocity) / 2.0
        avg_right_wheel_velocity = (right_wheel_velocity + self.last_right_wheel_velocity) / 2.0

        # Kinematic model
        v = self.r * (avg_right_wheel_velocity + avg_left_wheel_velocity) / 2.0
        omega = self.r * (avg_right_wheel_velocity - avg_left_wheel_velocity) / self.b

        # Update pose
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to be between -pi and pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        quat_x = 0.0
        quat_y = 0.0
        quat_z = math.sin(self.theta / 2.0)
        quat_w = math.cos(self.theta / 2.0)
        
        odom_msg.pose.pose.orientation.x = quat_x
        odom_msg.pose.pose.orientation.y = quat_y
        odom_msg.pose.pose.orientation.z = quat_z
        odom_msg.pose.pose.orientation.w = quat_w

        # Set velocities
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = omega

        self.odom_publisher.publish(odom_msg)

        # Publish TF transform (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat_x
        t.transform.rotation.y = quat_y
        t.transform.rotation.z = quat_z
        t.transform.rotation.w = quat_w
        self.tf_broadcaster.sendTransform(t)

        # Update last state
        self.last_time = current_time
        self.last_left_wheel_velocity = left_wheel_velocity
        self.last_right_wheel_velocity = right_wheel_velocity

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()