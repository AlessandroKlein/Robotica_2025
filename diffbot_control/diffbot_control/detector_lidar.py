import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarZoneDetector(Node):
    def __init__(self):
        super().__init__('lidar_zone_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

    def scan_callback(self, msg):
        zones = {
            'L': [],
            'FL': [],
            'F': [],
            'FR': [],
            'R': []
        }

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if math.isinf(distance) or math.isnan(distance):
                continue

            # Clasificación por zona
            if 2.09 <= angle <= 2.62:
                zones['L'].append(distance)
            elif 1.57 <= angle < 2.09:
                zones['FL'].append(distance)
            elif 1.04 <= angle < 1.57:
                zones['F'].append(distance)
            elif 0.52 <= angle < 1.04:
                zones['FR'].append(distance)
            elif 0.00 <= angle < 0.52:
                zones['R'].append(distance)

        # Evaluación por zona
        for zone, distances in zones.items():
            if distances:
                min_dist = min(distances)
                self.get_logger().info(f'{zone}: distancia mínima = {min_dist:.2f} m')
                if min_dist < 0.5:
                    self.get_logger().warn(f' Obstáculo en zona {zone} a {min_dist:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = LidarZoneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()