#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class LineSegment(Node):
    """
    Line segment extraction node.
    Subscribes:
        - /scan (sensor_msgs/LaserScan)
    Publishes:
      - /extracted_lines visualization (visualization_msgs/MarkerArray)
    """

    def __init__(self):
        super().__init__('line_segment_extraction_node')
        # ---- Parameters ----
        self.declare_parameter('epsilon', 0.05)  # Distancia máxima para considerar puntos como parte de la misma línea
        self.declare_parameter('p_min', 6)   # Número mínimo de puntos para formar una línea
        self.declare_parameter('line_width', 0.02)  # Ancho de las líneas para visualización
        self.declare_parameter('l_min', 0.1)  # Longitud mínima de una línea para ser considerada válida

        # Carga inicial de parámetros
        self.epsilon = self.get_parameter('epsilon').value
        self.p_min = self.get_parameter('p_min').value
        self.line_width = self.get_parameter('line_width').value
        self.l_min = self.get_parameter('l_min').value

        # Qos para suscripciones y publicaciones
        qos_sensor = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(depth=10)

        # Suscripción al Lidar
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_sensor)

        # Publicación de líneas extraídas
        self.lines_pub = self.create_publisher(MarkerArray, '/extracted_lines', qos_reliable)

        self.get_logger().info("Line Segment Extraction Node Ready")
        self.get_logger().info(f"Parameters: epsilon={self.epsilon:.2f}, p_min={self.p_min}, line_width={self.line_width:.2f}, l_min={self.l_min:.2f}")


    ### ===== Callbacks ===== ###
    def scan_callback(self, msg):
        pass
    

    ### ===== Funciones para extracción de líneas ===== ###
    def detection_seed_segments(self, points):
        pass

    def region_growing(self, points, seed_segments):
        pass
    
    def overlap_region_processing(self, segments):
        pass


    ### ===== Funciones para geometría ===== ###
    def distance_point_to_point(self, p1, p2):
        pass

    def distance_point_to_line(self, point, line_start, line_end):
        pass
    
    def predict_point_on_line(self, point, line_start, line_end):
        pass

    def polar_to_cartesian(self, ranges, angle_min, angle_increment):
        pass

    ### ===== Funciones para visualización ===== ###
    def publish_extracted_lines(self, lines):
        pass

    

def main():
    rclpy.init()
    node = LineSegment()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

