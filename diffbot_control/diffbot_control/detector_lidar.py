import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math

class LidarZoneDetector(Node):
    def __init__(self):
        super().__init__('lidar_zone_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Publisher para marcadores de visualización
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/lidar_zones',
            10)
        
        # Crear marcadores de zonas
        self.create_zone_markers()

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
        
        # Publicar marcadores de zonas
        self.publish_zone_markers()
    
    def create_zone_markers(self):
        """Crear marcadores para visualizar las zonas de detección"""
        self.zone_markers = MarkerArray()
        
        # Definir zonas con sus ángulos y colores
        zones_config = {
            'L': {'angles': (2.09, 2.62), 'color': (1.0, 0.0, 0.0, 0.3)},  # Rojo
            'FL': {'angles': (1.57, 2.09), 'color': (1.0, 0.5, 0.0, 0.3)}, # Naranja
            'F': {'angles': (1.04, 1.57), 'color': (1.0, 1.0, 0.0, 0.3)},  # Amarillo
            'FR': {'angles': (0.52, 1.04), 'color': (0.0, 1.0, 0.0, 0.3)}, # Verde
            'R': {'angles': (0.00, 0.52), 'color': (0.0, 0.0, 1.0, 0.3)}   # Azul
        }
        
        for i, (zone_name, config) in enumerate(zones_config.items()):
            marker = Marker()
            marker.header.frame_id = "lidar_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "lidar_zones"
            marker.id = i
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            
            # Configurar color
            marker.color = ColorRGBA(
                r=config['color'][0],
                g=config['color'][1], 
                b=config['color'][2],
                a=config['color'][3]
            )
            
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            
            # Crear triángulos para representar la zona
            angle_start, angle_end = config['angles']
            radius = 3.0  # Radio de visualización
            
            # Punto central
            center = Point(x=0.0, y=0.0, z=0.0)
            
            # Crear múltiples triángulos para formar el sector
            num_segments = 10
            angle_step = (angle_end - angle_start) / num_segments
            
            for j in range(num_segments):
                a1 = angle_start + j * angle_step
                a2 = angle_start + (j + 1) * angle_step
                
                # Tres puntos del triángulo
                p1 = Point(x=0.0, y=0.0, z=0.0)
                p2 = Point(x=radius * math.cos(a1), y=radius * math.sin(a1), z=0.0)
                p3 = Point(x=radius * math.cos(a2), y=radius * math.sin(a2), z=0.0)
                
                marker.points.extend([p1, p2, p3])
                
                # Colores para cada vértice
                for _ in range(3):
                    marker.colors.append(marker.color)
            
            self.zone_markers.markers.append(marker)
    
    def publish_zone_markers(self):
        """Publicar marcadores de zonas"""
        for marker in self.zone_markers.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        
        self.marker_pub.publish(self.zone_markers)

def main(args=None):
    rclpy.init(args=args)
    node = LidarZoneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()