import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped 
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

class FsmGapFinder(Node):
    def __init__(self):
        super().__init__('fsm_gap_finder')

        # --- PARÁMETROS BÁSICOS ---
        self.declare_parameter('robot_width', 0.3)
        self.declare_parameter('fov_angle', np.radians(60))
        self.declare_parameter('safety_margin', 0.15) 
        self.declare_parameter('horizon_dist', 4.0)
        self.declare_parameter('debug_mode', True) 

        # --- [NUEVO] PARÁMETROS AVANZADOS ---
        # Penalización por cambio brusco de dirección (Histéresis)
        self.declare_parameter('hysteresis_penalty', 3.0)
        # 0.0 = Solo mira la profundidad, 1.0 = Solo mira el centro del hueco
        self.declare_parameter('center_bias', 0.4) 

        self.width = self.get_parameter('robot_width').value
        self.fov_angle = self.get_parameter('fov_angle').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.horizon_dist = self.get_parameter('horizon_dist').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.hysteresis_penalty = self.get_parameter('hysteresis_penalty').value
        self.center_bias = self.get_parameter('center_bias').value
        
        self.angles = None
        # [NUEVO] Memoria de la última dirección elegida
        self.last_target_angle = 0.0

        qos_sensor = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(depth=10)

        # Suscripción
        self.scan_sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_callback, qos_sensor)
        
        # Publicación a la FSM
        self.gap_pub = self.create_publisher(TwistStamped, '/gap_data', qos_reliable)

        # Publicadores de Debug
        if self.debug_mode:
            self.proc_scan_pub = self.create_publisher(LaserScan, '/proc_scan', qos_reliable)
            self.marker_pub = self.create_publisher(MarkerArray, '/debug_markers', qos_reliable)

        self.get_logger().info("FSM Gap Finder Ready (Advanced Scoring & Hybrid Targeting).")

    # ==========================================================
    # ORQUESTADOR PRINCIPAL (Callback)
    # ==========================================================
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        if self.angles is None or len(self.angles) != len(ranges):
            self.angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # 1. Preprocesamiento (Filtros y FOV)
        proc_ranges, proc_angles = self._preprocess_scan(ranges)

        # 2. Extensión de Disparidad (Inflar obstáculos)
        proc_ranges = self._apply_disparity_extender(proc_ranges, msg.angle_increment)

        # 3. Encontrar el mejor hueco (con sistema de puntuación e histéresis)
        # [NUEVO] Pasamos los ángulos para poder calcular la penalización direccional
        target_idx, gap_start, gap_end = self._score_and_find_gap(proc_ranges, proc_angles)

        # 4. Extracción de datos para el controlador FSM
        target_angle = proc_angles[target_idx]
        target_depth = proc_ranges[target_idx]
        valid_ranges = proc_ranges[proc_ranges > 0.1]
        closest_obstacle = np.min(valid_ranges) if len(valid_ranges) > 0 else self.horizon_dist

        # [NUEVO] Guardamos el ángulo elegido para recordarlo en el próximo ciclo
        self.last_target_angle = target_angle

        # 5. Publicar a la Máquina de Estados
        self._publish_fsm_data(msg.header, target_angle, closest_obstacle, target_depth)

        # 6. Visualización RViz
        if self.debug_mode:
            self._publish_debug_scan(proc_ranges, msg)
            safe_end_idx = max(0, gap_end - 1)
            self._publish_debug_markers(target_angle, proc_angles[gap_start], proc_angles[safe_end_idx], msg.header)

    # ==========================================================
    # MÉTODOS PRIVADOS (Lógica de Negocio)
    # ==========================================================
    
    def _preprocess_scan(self, ranges):
        clean_ranges = np.nan_to_num(ranges, posinf=self.horizon_dist, neginf=0.0)
        clean_ranges = np.clip(clean_ranges, 0.0, self.horizon_dist)

        fov_mask = np.abs(self.angles) <= self.fov_angle
        proc_ranges = clean_ranges[fov_mask].copy() 
        proc_angles = self.angles[fov_mask].copy()
        
        return proc_ranges, proc_angles

    def _apply_disparity_extender(self, ranges, angle_increment):
        threshold = 0.2
        
        # 1. Creamos una copia inmutable para leer la verdad absoluta del LiDAR
        # Así evitamos que burbujas anteriores alteren los cálculos siguientes.
        original_ranges = ranges.copy()
        
        # Calculamos los saltos usando la copia inmutable
        diffs = np.diff(original_ranges)
        disparity_indices = np.where(np.abs(diffs) > threshold)[0]
        
        for idx in disparity_indices:
            # Leemos las distancias desde el arreglo original intacto
            if original_ranges[idx] < original_ranges[idx+1]:
                closer_idx = idx
                farther_idx = idx + 1
            else:
                closer_idx = idx + 1
                farther_idx = idx
                
            closer_dist = original_ranges[closer_idx]
            
            # Cálculo del tamaño de la burbuja (Se mantiene igual)
            margin = self.safety_margin * 1.5 if closer_dist < 0.5 else self.safety_margin
            safety_radius = (self.width / 2.0) + margin 
            
            bubble_angle = np.arctan2(safety_radius, max(closer_dist, 0.1))
            bubble_indices = int(bubble_angle / angle_increment)
            
            if closer_idx == idx: # Obstáculo a la izquierda, inflar hacia la derecha
                start = farther_idx
                end = min(len(ranges), farther_idx + bubble_indices)
                
                # 2. LA MAGIA: np.minimum
                # Comparamos lo que ya hay en el arreglo (ranges) con nuestra nueva burbuja (closer_dist).
                # Solo se sobrescribe si la nueva burbuja es MÁS CERCANA que lo que ya había.
                ranges[start:end] = np.minimum(ranges[start:end], closer_dist)
                
            else: # Obstáculo a la derecha, inflar hacia la izquierda
                start = max(0, farther_idx - bubble_indices + 1)
                end = farther_idx + 1
                
                # 2. LA MAGIA: np.minimum evita borrar obstáculos cercanos
                ranges[start:end] = np.minimum(ranges[start:end], closer_dist)

        # Zonas demasiado cercanas se consideran lava letal
        ranges[ranges < 0.2] = 0.0
        return ranges

    def _score_and_find_gap(self, ranges, angles):
        """Evalúa huecos usando ancho, profundidad y MEMORIA direccional."""
        mask = ranges > 0.05
        padded_mask = np.concatenate(([False], mask, [False]))
        diff = np.diff(padded_mask.astype(int))
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0]
        
        if len(starts) == 0: 
            return len(ranges) // 2, 0, len(ranges) - 1
            
        best_score = -float('inf')
        best_gap_idx = 0
        
        # --- PUNTUACIÓN CON HISTÉRESIS ---
        for i in range(len(starts)):
            width = ends[i] - starts[i]
            gap_rays = ranges[starts[i]:ends[i]]
            depth = np.mean(gap_rays)
            
            # [NUEVO] Calcular el ángulo medio de este hueco
            mid_idx = int((starts[i] + ends[i]) / 2)
            gap_mid_angle = angles[mid_idx]
            
            # [NUEVO] Calcular penalización por cambio de dirección
            # Cuanto más se aleje de nuestro último objetivo, más le restamos
            angle_diff = abs(gap_mid_angle - self.last_target_angle)
            penalty = angle_diff * self.hysteresis_penalty
            
            # Puntuación final: Profundidad + Ancho - Penalización por volantazo
            score = (depth * 2.0) + (width * 1.0) - penalty
            
            if score > best_score:
                best_score = score
                best_gap_idx = i
                
        gap_start = starts[best_gap_idx]
        gap_end = ends[best_gap_idx]
        
        # --- [NUEVO] SELECCIÓN DE OBJETIVO HÍBRIDO ---
        # 1. Índice de la Máxima Profundidad
        gap_ranges = ranges[gap_start:gap_end]
        max_depth = np.max(gap_ranges)
        deepest_indices = np.where(gap_ranges >= max_depth - 0.1)[0]
        depth_idx_in_gap = int(np.mean(deepest_indices))
        absolute_depth_idx = gap_start + depth_idx_in_gap
        
        # 2. Índice del Centro Geométrico del hueco
        absolute_center_idx = int((gap_start + gap_end) / 2)
        
        # 3. Mezcla ponderada (Blend)
        best_target_idx = int((self.center_bias * absolute_center_idx) + ((1.0 - self.center_bias) * absolute_depth_idx))
        
        # Aseguramos que el índice no se salga de los límites del hueco por seguridad
        best_target_idx = max(gap_start, min(best_target_idx, gap_end - 1))
        
        return best_target_idx, gap_start, gap_end

    def _publish_fsm_data(self, header, angle, closest_obstacle, depth):
        data_msg = TwistStamped()
        data_msg.header = header
        data_msg.twist.angular.z = float(angle)
        data_msg.twist.linear.x = float(closest_obstacle) 
        data_msg.twist.linear.y = float(depth)
        self.gap_pub.publish(data_msg)

    # ==========================================================
    # MÉTODOS DE VISUALIZACIÓN (RViz)
    # ==========================================================

    def _publish_debug_scan(self, proc_ranges, original_msg):
        debug_msg = LaserScan()
        debug_msg.header = original_msg.header
        
        debug_msg.angle_min = -self.fov_angle + np.pi
        debug_msg.angle_max = self.fov_angle + np.pi
        debug_msg.angle_increment = original_msg.angle_increment
        debug_msg.time_increment = original_msg.time_increment
        debug_msg.scan_time = original_msg.scan_time
        debug_msg.range_min = original_msg.range_min
        debug_msg.range_max = original_msg.range_max
        
        debug_msg.ranges = proc_ranges.tolist()
        self.proc_scan_pub.publish(debug_msg)

    def _publish_debug_markers(self, target_angle, gap_start_angle, gap_end_angle, header):
        marker_array = MarkerArray()
        
        vis_target = target_angle + np.pi
        vis_start = gap_start_angle + np.pi
        vis_end = gap_end_angle + np.pi
        
        # Marcador 1: Flecha de Dirección (Verde)
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
        end_pt = Point(x=2.0 * np.cos(vis_target), y=2.0 * np.sin(vis_target), z=0.0)
        arrow.points = [start_pt, end_pt]
        
        # Marcador 2: Límites del Gap (Líneas Rojas)
        gap_lines = Marker()
        gap_lines.header = header
        gap_lines.ns = "gap_boundaries"
        gap_lines.id = 1
        gap_lines.type = Marker.LINE_LIST
        gap_lines.action = Marker.ADD
        gap_lines.scale.x = 0.01
        gap_lines.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0) 
        
        p1 = Point(x=3.0 * np.cos(vis_start), y=3.0 * np.sin(vis_start), z=0.0)
        p2 = Point(x=3.0 * np.cos(vis_end), y=3.0 * np.sin(vis_end), z=0.0)
        gap_lines.points = [start_pt, p1, start_pt, p2]

        marker_array.markers = [arrow, gap_lines]
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = FsmGapFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()