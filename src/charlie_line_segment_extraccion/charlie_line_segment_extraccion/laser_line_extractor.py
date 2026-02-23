#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np

class LineSegmentExtractor:
    def __init__(self):
        # Valores inicializados en 0, se sobrescriben inmediatamente por los parámetros
        self.L_min = 0.0     
        self.P_min = 0       
        self.S_num = 0       
        self.epsilon = 0.0  
        self.delta = 0.0
        self.min_range = 0.0
        self.max_range = 0.0

    def fit_line_orthogonal(self, points):
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid
        try:
            _, _, Vt = np.linalg.svd(centered_points)
            normal = Vt[-1] 
            return normal[0], normal[1], -np.dot(normal, centroid)
        except np.linalg.LinAlgError:
            return 1.0, 0.0, -centroid[0] 

    def extract_lines(self, ranges, angle_min, angle_inc):
        ranges_arr = np.array(ranges)
        
        # Filtrar valores usando los parámetros
        valid_mask = np.isfinite(ranges_arr) & (ranges_arr >= self.min_range) & (ranges_arr <= self.max_range)
        valid_indices = np.where(valid_mask)[0]
        
        if len(valid_indices) < self.P_min:
            return []

        bearings = angle_min + valid_indices * angle_inc
        r = ranges_arr[valid_indices]
        
        x = r * np.cos(bearings)
        y = r * np.sin(bearings)
        points = np.column_stack((x, y))
        
        N_p = len(points)
        extracted_lines = []
        used_points = np.zeros(N_p, dtype=bool)

        i = 0
        while i < N_p - self.P_min:
            if used_points[i]:
                i += 1
                continue
                
            j = i + self.S_num
            if j >= N_p:
                break
                
            seed_pts = points[i:j]
            a, b, c = self.fit_line_orthogonal(seed_pts)
            denom_dist = np.sqrt(a**2 + b**2 + 1e-6)
            
            test_pts = points[i:j]
            test_bearings = bearings[i:j]
            
            dists = np.abs(a * test_pts[:, 0] + b * test_pts[:, 1] + c) / denom_dist
            if np.any(dists > self.epsilon):
                i += 1
                continue
                
            denoms = a * np.cos(test_bearings) + b * np.sin(test_bearings)
            valid_denoms = np.abs(denoms) > 1e-6
            
            if not np.all(valid_denoms):
                i += 1
                continue
                
            x_preds = -c * np.cos(test_bearings) / denoms
            y_preds = -c * np.sin(test_bearings) / denoms
            
            dist_preds = np.sqrt((test_pts[:, 0] - x_preds)**2 + (test_pts[:, 1] - y_preds)**2)
            
            if np.any(dist_preds > self.delta):
                i += 1
                continue
            
            P_b = i  
            P_f = j  
            
            while P_f < N_p and not used_points[P_f]:
                dist_to_line = abs(a * points[P_f, 0] + b * points[P_f, 1] + c) / denom_dist
                if dist_to_line < self.epsilon:
                    P_f += 1
                    if (P_f - P_b) % 3 == 0:
                        a, b, c = self.fit_line_orthogonal(points[P_b:P_f])
                        denom_dist = np.sqrt(a**2 + b**2 + 1e-6)
                else:
                    break
            
            line_pts = points[P_b:P_f]
            if len(line_pts) >= self.P_min:
                pt_start = line_pts[0]
                pt_end = line_pts[-1]
                L_l = np.sqrt((pt_end[0] - pt_start[0])**2 + (pt_end[1] - pt_start[1])**2)
                
                if L_l >= self.L_min:
                    denom_proj = a**2 + b**2
                    if denom_proj > 1e-6:
                        pts_to_proj = np.array([pt_start, pt_end])
                        xs = (b**2 * pts_to_proj[:, 0] - a * b * pts_to_proj[:, 1] - a * c) / denom_proj
                        ys = (a**2 * pts_to_proj[:, 1] - a * b * pts_to_proj[:, 0] - b * c) / denom_proj
                        
                        extracted_lines.append(([xs[0], ys[0]], [xs[1], ys[1]]))
                    used_points[P_b:P_f] = True
            
            i = P_f 
                
        return extracted_lines


class LaserLineExtractorNode(Node):
    def __init__(self):
        super().__init__('laser_line_extractor')
        self.extractor = LineSegmentExtractor()
        
        # --- DECLARACIÓN DE PARÁMETROS (Permite cargarlos desde YAML) ---
        self.declare_parameter('L_min', 0.4)
        self.declare_parameter('P_min', 8)
        self.declare_parameter('S_num', 5)
        self.declare_parameter('epsilon', 0.05)
        self.declare_parameter('delta', 0.15)
        self.declare_parameter('min_range', 0.2)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('process_interval', 0.2)
        self.declare_parameter('marker_lifetime', 10)

        # Variables de control interno
        self.global_marker_id = 0
        self.last_process_time_ns = 0

        # Suscriptor y Publicador
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.scan_callback,
            qos_profile_sensor_data) 
            
        self.marker_pub = self.create_publisher(MarkerArray, '/extracted_lines_markers', 10)
        self.get_logger().info('✅ Nodo Extractor de Líneas Iniciado (Controlado por Launch/YAML)')

    def scan_callback(self, msg):
        # 1. Leer parámetros dinámicamente
        process_interval = self.get_parameter('process_interval').value
        marker_lifetime = self.get_parameter('marker_lifetime').value
        
        self.extractor.L_min = self.get_parameter('L_min').value
        self.extractor.P_min = self.get_parameter('P_min').value
        self.extractor.S_num = self.get_parameter('S_num').value
        self.extractor.epsilon = self.get_parameter('epsilon').value
        self.extractor.delta = self.get_parameter('delta').value
        self.extractor.min_range = self.get_parameter('min_range').value
        self.extractor.max_range = self.get_parameter('max_range').value

        # 2. Control de Tasa (Throttle) usando reloj interno de ROS 2 (Nanosegundos)
        current_time_ns = self.get_clock().now().nanoseconds
        process_interval_ns = int(process_interval * 1e9) # Convertir segundos a nanosegundos
        
        if (current_time_ns - self.last_process_time_ns) < process_interval_ns:
            return
        self.last_process_time_ns = current_time_ns

        # 3. Extraer Líneas
        lines = self.extractor.extract_lines(msg.ranges, msg.angle_min, msg.angle_increment)
        
        if not lines:
            return 

        # 4. Publicar en RViz
        marker_array = MarkerArray()
        
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "map_lines"
        
        marker.id = self.global_marker_id
        self.global_marker_id += 1
        
        marker.type = Marker.LINE_LIST 
        marker.action = Marker.ADD
        
        marker.pose.orientation.w = 1.0 
        marker.lifetime.sec = int(marker_lifetime) 
        marker.scale.x = 0.08 
        
        marker.color.r = 1.0  
        marker.color.a = 1.0  
        
        for p1, p2 in lines:
            p_start = Point(x=float(p1[0]), y=float(p1[1]), z=0.0)
            p_end = Point(x=float(p2[0]), y=float(p2[1]), z=0.0)
            marker.points.extend([p_start, p_end])
            
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LaserLineExtractorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()