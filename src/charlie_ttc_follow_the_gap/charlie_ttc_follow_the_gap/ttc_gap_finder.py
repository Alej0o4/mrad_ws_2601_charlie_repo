#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped 
from nav_msgs.msg import Odometry 
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

class TtcGapFinder(Node):
    def __init__(self):
        super().__init__('ttc_gap_finder')

        # --- PARÁMETROS (Ajustados para Robot Diferencial) ---
        self.declare_parameter('robot_width', 0.44)
        self.declare_parameter('ttc_min', 0.8)         # ¡Corregido! 
        self.declare_parameter('fov_angle', 1.57)      # 90 grados a cada lado (Sin puntos ciegos)
        self.declare_parameter('safety_margin', 0.15)  # Margen seguro base
        self.declare_parameter('debug_mode', True) 

        # Leer valores
        self.fov_angle = self.get_parameter('fov_angle').value
        self.width = self.get_parameter('robot_width').value
        self.ttc_min = self.get_parameter('ttc_min').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        self.current_speed = 0.0
        self.ranges = None
        self.angles = None

        qos_sensor = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(depth=10)

        # Suscripciones
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_sensor)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_reliable)

        # Publicación Principal
        self.angle_pub = self.create_publisher(TwistStamped, '/gap_angle', qos_reliable)

        # --- PUBLICADORES DE DEBUG ---
        if self.debug_mode:
            self.proc_scan_pub = self.create_publisher(LaserScan, '/proc_scan', qos_reliable)
            self.marker_pub = self.create_publisher(MarkerArray, '/debug_markers', qos_reliable)

        self.get_logger().info(f"TTC Gap Finder (Ultimate Edition) Ready. Debug Mode: {self.debug_mode}")

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        if self.angles is None or len(self.angles) != len(ranges):
            self.angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
            
        ranges = np.nan_to_num(ranges, posinf=msg.range_max)

        # 2. Recorte FOV
        fov_mask = np.abs(self.angles) <= self.fov_angle
        proc_ranges = ranges[fov_mask].copy() 
        proc_angles = self.angles[fov_mask].copy()

        # 3. Filtro TTC
        if self.current_speed > 0.1:
            closing_speeds = self.current_speed * np.cos(proc_angles)
            closing_speeds[closing_speeds <= 0] = 0.001
            ttc = proc_ranges / closing_speeds
            unsafe_mask = ttc < self.ttc_min
            proc_ranges[unsafe_mask] = 0.0
        
        # 4. Burbuja de Seguridad Dinámica (¡Corregida!)
        valid_indices = np.where(proc_ranges > 0.05)[0]
        
        if len(valid_indices) > 0:
            min_dist_idx = valid_indices[np.argmin(proc_ranges[valid_indices])]
            min_dist = proc_ranges[min_dist_idx]
            
            if min_dist < 0.5:
                # Límite mínimo duro de 3cm para no apagar el hueco
                dynamic_margin = max(0.03, self.safety_margin * (min_dist / 0.5))
            else:
                dynamic_margin = self.safety_margin

            safety_radius = (self.width / 2.0) + dynamic_margin 
            bubble_angle = np.arctan2(safety_radius, min_dist)
            angle_inc = msg.angle_increment
            bubble_indices = int(bubble_angle / angle_inc)
            
            start = max(0, min_dist_idx - bubble_indices)
            end = min(len(proc_ranges), min_dist_idx + bubble_indices)
            proc_ranges[start:end] = 0.0

        # 5. Encontrar el MEJOR Gap (Combinación de Ancho y Profundo)
        gap_start, gap_end = self.find_best_gap(proc_ranges)
        
        # ---------------------------------------------------------
        # 6. CALCULAR OBJETIVO (Afinado para Ackermann)
        # ---------------------------------------------------------
        gap_center = (gap_start + (gap_end - 1)) // 2
        gap_width = gap_end - gap_start
        
        best_idx = gap_center
        best_score = -1.0

        for i in range(gap_start, gap_end):
            # Saturamos a 4.0m (Ackermann necesita mirar un poco más lejos que un diferencial)
            depth = min(proc_ranges[i], 4.0)
            
            # Distancia normalizada
            dist_from_center = abs(i - gap_center) / (gap_width / 2.0 + 1e-5)
            
            # Penalización LATERAL reducida dramáticamente (del 70% al 15%)
            # Permite al Ackermann comprometerse a giros cerrados sin "arrepentirse" y oscilar
            score = depth * (1.0 - 0.35 * dist_from_center)
            
            if score > best_score:
                best_score = score
                best_idx = i
                
        steering_angle = proc_angles[best_idx]

        # ---------------------------------------------------------
        # 7. INSTINTO DE SUPERVIVENCIA LATERAL (Repulsión)
        # ---------------------------------------------------------
        # Promedio de los 10 rayos más a la derecha y a la izquierda
        right_clearance = np.mean(proc_ranges[:10]) 
        left_clearance = np.mean(proc_ranges[-10:]) 
        
        # Si gira hacia la izquierda pero roza a la derecha
        if steering_angle > 0 and right_clearance < 0.4:
            steering_angle *= 1.5
        # Si gira hacia la derecha pero roza a la izquierda
        elif steering_angle < 0 and left_clearance < 0.4:
            steering_angle *= 1.5

        # 8. Publicar
        out_msg = TwistStamped()
        out_msg.header = msg.header
        out_msg.twist.angular.z = float(steering_angle)
        self.angle_pub.publish(out_msg)

        # --- LOGICA DE VISUALIZACIÓN ---
        if self.debug_mode:
            self.publish_debug_scan(proc_ranges, msg)
            safe_end_idx = max(0, gap_end - 1)
            self.publish_debug_markers(steering_angle, proc_angles[gap_start], proc_angles[safe_end_idx], msg.header)

    # --- LA NUEVA FUNCIÓN DE BÚSQUEDA ---
    def find_best_gap(self, ranges):
        mask = ranges > 0.05
        padded_mask = np.concatenate(([False], mask, [False]))
        diff = np.diff(padded_mask.astype(int))
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0]
        
        if len(starts) == 0: return 0, len(ranges)-1
        
        best_score = -1.0
        best_start = 0
        best_end = len(ranges)-1
        
        for s, e in zip(starts, ends):
            gap_width = e - s
            if gap_width < 5:  # Ignorar ruido
                continue
                
            gap_ranges = ranges[s:e]
            avg_depth = np.mean(gap_ranges)
            
            # Fusión: 40% ancho, 60% profundidad
            score = (gap_width * 0.7) + (avg_depth * 2.0)
            
            if score > best_score:
                best_score = score
                best_start = s
                best_end = e
                
        return best_start, best_end

    # --- FUNCIONES DE DEBUG ---
    def publish_debug_scan(self, proc_ranges, original_msg):
        debug_msg = LaserScan()
        debug_msg.header = original_msg.header
        debug_msg.angle_min = -self.fov_angle 
        debug_msg.angle_max = self.fov_angle
        debug_msg.angle_increment = original_msg.angle_increment
        debug_msg.time_increment = original_msg.time_increment
        debug_msg.scan_time = original_msg.scan_time
        debug_msg.range_min = original_msg.range_min
        debug_msg.range_max = original_msg.range_max
        debug_msg.ranges = proc_ranges.tolist()
        self.proc_scan_pub.publish(debug_msg)

    def publish_debug_markers(self, target_angle, gap_start_angle, gap_end_angle, header):
        marker_array = MarkerArray()
        
        arrow = Marker()
        arrow.header = header
        arrow.ns = "steering_goal"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = 0.05
        arrow.scale.y = 0.1
        arrow.scale.z = 0.1
        arrow.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        
        start_pt = Point(x=0.0, y=0.0, z=0.0)
        end_pt = Point(x=2.0 * np.cos(target_angle), y=2.0 * np.sin(target_angle), z=0.0)
        arrow.points = [start_pt, end_pt]
        
        gap_lines = Marker()
        gap_lines.header = header
        gap_lines.ns = "gap_boundaries"
        gap_lines.id = 1
        gap_lines.type = Marker.LINE_LIST
        gap_lines.action = Marker.ADD
        gap_lines.scale.x = 0.01
        gap_lines.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        p1 = Point(x=3.0 * np.cos(gap_start_angle), y=3.0 * np.sin(gap_start_angle), z=0.0)
        p2 = Point(x=3.0 * np.cos(gap_end_angle), y=3.0 * np.sin(gap_end_angle), z=0.0)
        
        gap_lines.points = [start_pt, p1, start_pt, p2]

        marker_array.markers = [arrow, gap_lines]
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = TtcGapFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()