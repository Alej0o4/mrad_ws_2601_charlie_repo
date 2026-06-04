#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ScanInverter(Node):
    def __init__(self):
        super().__init__("scan_inverter")

        # QoS: Los sensores físicos suelen usar "Best Effort" en lugar de "Reliable"
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber: Escucha los datos crudos del hardware
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.callback,
            qos_profile
        )

        # Publisher: Publica los datos corregidos para tu dist_finder
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan_filtered',     
            qos_profile
        )
        self.get_logger().info("Scan Inverter Node Initialized. Aplicando offset de 180°.")

    def callback(self, msg):
        inverted_msg = LaserScan()
        
        # 1. Copiar todos los metadatos exactamente igual
        inverted_msg.header = msg.header
        inverted_msg.angle_min = msg.angle_min
        inverted_msg.angle_max = msg.angle_max
        inverted_msg.angle_increment = msg.angle_increment
        inverted_msg.time_increment = msg.time_increment
        inverted_msg.scan_time = msg.scan_time
        inverted_msg.range_min = msg.range_min
        inverted_msg.range_max = msg.range_max

        # 2. Lógica de inversión (Offset de 180 grados)
        n_points = len(msg.ranges)
        half = n_points // 2

        # Desplazamos el arreglo partiendo por la mitad
        # Lo que estaba atrás (segunda mitad) pasa adelante, y viceversa.
        if n_points > 0:
            inverted_msg.ranges = msg.ranges[half:] + msg.ranges[:half]
        
        # Si el LiDAR físico provee intensidades, las rotamos también
        if len(msg.intensities) > 0:
            inverted_msg.intensities = msg.intensities[half:] + msg.intensities[:half]

        # 3. Publicar el mensaje corregido
        self.publisher.publish(inverted_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanInverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()