import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped 
from nav_msgs.msg import Odometry 
from visualization_msgs.msg import Marker, MarkerArray # <--- NUEVO
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

class TtcGapFinder(Node):
    def __init__(self):
        super().__init__('ttc_gap_finder')

        # --- PARÁMETROS ---
        self.declare_parameter('robot_width', 0.44)
        self.declare_parameter('ttc_min', 0.7)
        self.declare_parameter('fov_angle', np.radians(90))
        self.declare_parameter('safety_margin', 0.15) 
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
        self.scan_sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_callback, qos_sensor)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_reliable)

        # Publicación Principal
        self.angle_pub = self.create_publisher(TwistStamped, '/gap_angle', qos_reliable)

        # --- PUBLICADORES DE DEBUG ---
        if self.debug_mode:
            # Publica el scan modificado (con burbujas y recortes)
            self.proc_scan_pub = self.create_publisher(LaserScan, '/proc_scan', qos_reliable)
            # Publica flechas y lineas
            self.marker_pub = self.create_publisher(MarkerArray, '/debug_markers', qos_reliable)

        self.get_logger().info(f"TTC Gap Finder Ready. Debug Mode: {self.debug_mode}")
        self.get_logger().info(f"Parameters: FOV={np.degrees(self.fov_angle):.1f}°, Robot Width={self.width:.2f}m, TTC Min={self.ttc_min:.2f}s, Safety Margin={self.safety_margin:.2f}m")

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        if self.angles is None or len(self.angles) != len(ranges):
            self.angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
            
        # 1. MANEJO DE INFINITOS Y HORIZONTE
        # En lugar de dejar que los infinitos lleguen a range_max (ej. 12m), los limitamos 
        # a un horizonte táctico (ej. 4.0m). Así el robot se enfoca en el camino inmediato.
        horizon_dist = 4.0
        ranges = np.nan_to_num(ranges, posinf=horizon_dist, neginf=0.0)
        ranges = np.clip(ranges, 0.0, horizon_dist)

        # 2. Recorte FOV
        fov_mask = np.abs(self.angles) <= self.fov_angle
        proc_ranges = ranges[fov_mask].copy() 
        proc_angles = self.angles[fov_mask].copy()

        # 3. Filtro TTC (Se mantiene igual)
        if self.current_speed > 0.1:
            closing_speeds = self.current_speed * np.cos(proc_angles)
            closing_speeds[closing_speeds <= 0] = 0.001
            ttc = proc_ranges / closing_speeds
            unsafe_mask = ttc < self.ttc_min
            proc_ranges[unsafe_mask] = 0.0
        
        # 4. DISPARITY EXTENDER (El reemplazo de la burbuja única)
        threshold = 0.2  # Una diferencia de 20cm entre rayos es un "borde" de obstáculo
        diffs = np.diff(proc_ranges)
        disparity_indices = np.where(np.abs(diffs) > threshold)[0]
        
        for idx in disparity_indices:
            # Determinamos cuál rayo es la pared y cuál es el hueco
            if proc_ranges[idx] < proc_ranges[idx + 1]:
                closer_idx = idx
                farther_idx = idx + 1
            else:
                closer_idx = idx + 1
                farther_idx = idx
                
            closer_dist = proc_ranges[closer_idx]
            
            # Si el obstáculo está muy cerca, hacemos el margen un poco más grande
            margin = self.safety_margin
            if closer_dist < 0.5:
                margin *= 1.5 
                
            safety_radius = (self.width / 2.0) + margin 
            
            # Calculamos cuántos índices abarca este radio a esta distancia
            bubble_angle = np.arctan2(safety_radius, closer_dist)
            angle_inc = msg.angle_increment
            bubble_indices = int(bubble_angle / angle_inc)
            
            # "Engordamos" el obstáculo sobrescribiendo el espacio vacío con la distancia corta
            # Esto es mejor que poner 0.0, porque preserva la forma de la pared para el gap
            if closer_idx == idx: # Borde a la izquierda, extendemos hacia la derecha
                start = farther_idx
                end = min(len(proc_ranges), farther_idx + bubble_indices)
                proc_ranges[start:end] = closer_dist
            else: # Borde a la derecha, extendemos hacia la izquierda
                start = max(0, farther_idx - bubble_indices + 1)
                end = farther_idx + 1
                proc_ranges[start:end] = closer_dist

        # Opcional: Si el punto es MUY cercano (< 0.2m), lo forzamos a 0 para que sea "lava"
        proc_ranges[proc_ranges < 0.2] = 0.0

        # 5. Encontrar Gap (El hueco más ancho que queda después de inflar todo)
        gap_start, gap_end = self.find_max_gap(proc_ranges)
        
        # 6. CALCULAR OBJETIVO (Basado en la máxima profundidad)
        gap_ranges = proc_ranges[gap_start:gap_end]
        
        if len(gap_ranges) > 0:
            max_depth = np.max(gap_ranges)
            # Encontramos todos los índices que están casi a la profundidad máxima
            # Esto maneja el caso de que la profundidad máxima sea una pared recta ancha
            deepest_indices = np.where(gap_ranges >= max_depth - 0.1)[0]
            
            # Promediamos esos índices para apuntar al centro de la zona MÁS ALEJADA
            best_idx_in_gap = int(np.mean(deepest_indices))
            best_idx = gap_start + best_idx_in_gap
        else:
            # Fallback de seguridad si no hay huecos (apuntar al frente)
            best_idx = len(proc_ranges) // 2
            
        steering_angle = proc_angles[best_idx]

        # ---------------------------------------------------------
        # 6.5. MODULADOR GEOMÉTRICO DE VELOCIDAD (Freno por entorno)
        # ---------------------------------------------------------
        # A. Evaluar profundidad del objetivo
        target_depth = proc_ranges[best_idx]
        
        # B. Evaluar proximidad del obstáculo más cercano (situational awareness)
        # Usamos 'ranges' crudo para no ser engañados por la burbuja de ceros que dibujamos
        raw_valid = np.isfinite(ranges) & (ranges > 0.1)
        if np.any(raw_valid):
            closest_obstacle = np.min(ranges[raw_valid])
        else:
            closest_obstacle = 4.0

        # C. Ecuaciones de Modulación (Clamping lineal)
        # Si objetivo > 3.0m = 1.0 (100%). Si objetivo < 1.0m = 0.4 (40%)
        mult_depth = np.clip(target_depth / 3.0, 0.4, 1.0)
        
        # Si pared lateral > 0.8m = 1.0 (100%). Si pared < 0.3m = 0.5 (50%)
        mult_prox = np.clip(closest_obstacle / 0.8, 0.5, 1.0)
        
        # El multiplicador final es el "peor caso" entre la profundidad y la estrechez
        geometric_multiplier = min(mult_depth, mult_prox)

        # 7. Publicar
        out_msg = TwistStamped()
        out_msg.header = msg.header
        # Enviamos el ángulo como siempre
        out_msg.twist.angular.z = float(steering_angle)
        # ENVIAMOS EL MULTIPLICADOR EN EL CANAL LINEAL
        out_msg.twist.linear.x = float(geometric_multiplier) 
        
        self.angle_pub.publish(out_msg)

        # --- LOGICA DE VISUALIZACIÓN ---
        if self.debug_mode:
            self.publish_debug_scan(proc_ranges, msg)
            safe_end_idx = max(0, gap_end - 1)
            self.publish_debug_markers(steering_angle, proc_angles[gap_start], proc_angles[safe_end_idx], msg.header)

    def find_max_gap(self, ranges):
        mask = ranges > 0.05
        padded_mask = np.concatenate(([False], mask, [False]))
        diff = np.diff(padded_mask.astype(int))
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0]
        if len(starts) == 0: return 0, len(ranges)-1
        lengths = ends - starts
        best_score = -1
        best_idx = 0
        
        for i in range(len(starts)):
            width = ends[i] - starts[i]
            # Extraemos los rayos del hueco actual
            gap_rays = ranges[starts[i]:ends[i]]
            # Profundidad promedio del hueco
            depth = np.mean(gap_rays)
            
            # PUNTUACIÓN DE CAMPEONATO: Profundidad domina, el ancho desempata
            # Los factores (weights) los calibras en la pista
            score = (depth * 2.5) + (width * 1.0)
            
            if score > best_score:
                best_score = score
                best_idx = i
                
        return starts[best_idx], ends[best_idx]

    # --- FUNCIONES DE DEBUG ---

    def publish_debug_scan(self, proc_ranges, original_msg):
        """
        Publica un LaserScan falso que muestra lo que ve el algoritmo (Burbujas = 0)
        Rotado 180 grados (+ np.pi) puramente para visualización en RViz.
        """
        debug_msg = LaserScan()
        debug_msg.header = original_msg.header
        
        # FIX VISUAL: Sumamos np.pi (180 grados) a los ángulos base
        debug_msg.angle_min = -self.fov_angle + np.pi
        debug_msg.angle_max = self.fov_angle + np.pi
        debug_msg.angle_increment = original_msg.angle_increment
        debug_msg.time_increment = original_msg.time_increment
        debug_msg.scan_time = original_msg.scan_time
        debug_msg.range_min = original_msg.range_min
        debug_msg.range_max = original_msg.range_max
        
        debug_msg.ranges = proc_ranges.tolist()
        self.proc_scan_pub.publish(debug_msg)

    def publish_debug_markers(self, target_angle, gap_start_angle, gap_end_angle, header):
        marker_array = MarkerArray()
        
        # FIX VISUAL: Desfasamos todos los ángulos 180 grados (np.pi) solo para dibujar
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
        arrow.scale.x = 0.05 # Grosor flecha
        arrow.scale.y = 0.1 # Ancho cabeza
        arrow.scale.z = 0.1
        arrow.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0) # Verde
        
        # Puntos de la flecha (Origen -> Destino) usando vis_target
        start_pt = Point(x=0.0, y=0.0, z=0.0)
        end_pt = Point(
            x=2.0 * np.cos(vis_target), 
            y=2.0 * np.sin(vis_target), 
            z=0.0
        )
        arrow.points = [start_pt, end_pt]
        
        # Marcador 2: Límites del Gap (Líneas Rojas)
        gap_lines = Marker()
        gap_lines.header = header
        gap_lines.ns = "gap_boundaries"
        gap_lines.id = 1
        gap_lines.type = Marker.LINE_LIST
        gap_lines.action = Marker.ADD
        gap_lines.scale.x = 0.01
        gap_lines.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0) # Rojo
        
        # Linea Inicio Gap usando vis_start
        p1 = Point(x=3.0 * np.cos(vis_start), y=3.0 * np.sin(vis_start), z=0.0)
        # Linea Fin Gap usando vis_end
        p2 = Point(x=3.0 * np.cos(vis_end), y=3.0 * np.sin(vis_end), z=0.0)
        
        gap_lines.points = [start_pt, p1, start_pt, p2]

        marker_array.markers = [arrow, gap_lines]
        self.marker_pub.publish(marker_array)
def main(args=None):
    rclpy.init(args=args)
    node = TtcGapFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()