import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import SetParametersResult
import numpy as np

class FsmAebs(Node):
    def __init__(self):
        super().__init__('fsm_aebs_node')

        # --- PARÁMETROS GEOMÉTRICOS ---
        self.declare_parameter('robot_width', 0.40)
        self.declare_parameter('chassis_length', 0.37)
        self.declare_parameter('lidar_yaw_offset', 0.0)
        
        # --- PARÁMETROS DE SEGURIDAD ---
        self.declare_parameter('ttc_threshold_slow', 0.6) # TTC para velocidad baja
        self.declare_parameter('ttc_threshold_fast', 1.1) # TTC para velocidad alta (más tiempo para resbalar)
        self.declare_parameter('min_safe_distance', 0.20) # Burbuja estática
        self.declare_parameter('scan_timeout_sec', 0.5)
        self.declare_parameter('min_motion_velocity', 0.05)
        self.declare_parameter('straight_turn_w_threshold', 0.05)
        self.declare_parameter('lane_margin', 0.10)
        self.declare_parameter('range_min_valid', 0.10)
        self.declare_parameter('min_danger_points', 3)

        # --- PARÁMETROS EMPÍRICOS (Mapping PWM -> Real Speed) ---
        self.declare_parameter('v_slow_real', 1.30) 
        self.declare_parameter('v_fast_real', 1.40) 
        self.declare_parameter('pwm_fast_threshold', 0.15)
        self.declare_parameter('pwm_deadzone', 0.08)
        self.declare_parameter('stop_pwm', 0.0)

        # Cargar valores iniciales
        self._load_params()

        # Variables de estado
        self.ranges = None
        self.angles = None
        self.last_scan_time = self.get_clock().now()
        self.base_angles = None

        # ROS2 Setup
        qos_sensor = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_callback, qos_sensor)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel_raw', self.cmd_callback, 10)
        # Publica al tópico final del motor driver
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_aebs', 10)

        self.add_on_set_parameters_callback(self.parameters_callback)
        self.get_logger().info("FSM AEBS (Freno de Seguridad Inteligente) Activo.")

    def _load_params(self):
        self.width = self.get_parameter('robot_width').value
        self.half_width = (self.width / 2.0) + 0.1 
        self.bumper_x = self.get_parameter('chassis_length').value / 2.0
        self.ttc_slow = self.get_parameter('ttc_threshold_slow').value
        self.ttc_fast = self.get_parameter('ttc_threshold_fast').value
        self.min_dist = self.get_parameter('min_safe_distance').value
        self.timeout = self.get_parameter('scan_timeout_sec').value
        self.min_motion_velocity = self.get_parameter('min_motion_velocity').value
        self.turn_threshold = self.get_parameter('straight_turn_w_threshold').value
        self.lane_margin = self.get_parameter('lane_margin').value
        self.range_min_valid = self.get_parameter('range_min_valid').value
        self.min_danger_points = int(self.get_parameter('min_danger_points').value)
        self.yaw_offset = self.get_parameter('lidar_yaw_offset').value
        
        self.v_slow = self.get_parameter('v_slow_real').value
        self.v_fast = self.get_parameter('v_fast_real').value
        self.p_fast_th = self.get_parameter('pwm_fast_threshold').value
        self.p_dead = self.get_parameter('pwm_deadzone').value
        self.stop_pwm = self.get_parameter('stop_pwm').value

        self.half_width = (self.width / 2.0) + self.lane_margin

    def parameters_callback(self, params):
        for param in params:
            self.get_logger().info(f"Parámetro {param.name} actualizado.")
        self._load_params()
        return SetParametersResult(successful=True)

    # ==========================================================
    # LÓGICA DE PERCEPCIÓN
    # ==========================================================
    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()
        raw_ranges = np.array(msg.ranges)
        
        if self.base_angles is None or len(self.base_angles) != len(raw_ranges):
            angles = np.linspace(msg.angle_min, msg.angle_max, len(raw_ranges)) + self.yaw_offset
            self.base_angles = np.arctan2(np.sin(angles), np.cos(angles))
            
        # Filtro de puntos válidos
        valid = np.isfinite(raw_ranges) & (raw_ranges > self.range_min_valid)
        self.ranges = raw_ranges[valid]
        self.angles = self.base_angles[valid]

    # ==========================================================
    # LÓGICA DE CONTROL Y FRENO
    # ==========================================================
    def cmd_callback(self, msg):
        if self.ranges is None: return 

        # 1. Verificar Salud del Lidar
        dt_scan = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if dt_scan > self.timeout:
            self._emergency_stop(msg, "TIMEOUT LIDAR")
            return

        # 2. Obtener Velocidades 
        pwm_in = msg.twist.linear.x
        w_cmd = msg.twist.angular.z  # <--- Extraemos la intención de giro
        v_real = self._get_real_velocity(pwm_in)

        if abs(v_real) < self.min_motion_velocity: # Robot casi quieto
            self.cmd_pub.publish(msg)
            return

        # 3. Evaluación de Seguridad (Túnel Curvilíneo Inteligente)
        is_safe, val, reason = self._check_tunnel_safety(v_real, w_cmd)

        if not is_safe:
            self._emergency_stop(msg, f"{reason} ({val:.2f})")
        else:
            self.cmd_pub.publish(msg)

    def _get_real_velocity(self, pwm):
        """Mapea el comando PWM a la velocidad real que el robot alcanzará."""
        abs_pwm = abs(pwm)
        if abs_pwm < self.p_dead: return 0.0
        
        sign = np.sign(pwm)
        if abs_pwm >= self.p_fast_th:
            return sign * self.v_fast
        else:
            return sign * self.v_slow

    def _check_tunnel_safety(self, v_real, w_cmd):
        """
        Proyecta un túnel seguro y aplica TTC variable según la velocidad.
        """
        x_pts = self.ranges * np.cos(self.angles)
        y_pts = self.ranges * np.sin(self.angles)

        # 1. Filtro Longitudinal (Dirección de movimiento)
        if v_real > 0:
            mask_long = (x_pts > self.bumper_x)
            long_distances = x_pts[mask_long] - self.bumper_x
            y_rel = y_pts[mask_long]
            x_rel = x_pts[mask_long]
        else:
            mask_long = (x_pts < -self.bumper_x)
            long_distances = np.abs(x_pts[mask_long] + self.bumper_x)
            y_rel = y_pts[mask_long]
            x_rel = x_pts[mask_long]

        if len(long_distances) == 0:
            return True, 0.0, "NONE"

        # 2. SELECCIÓN DEL TTC DINÁMICO
        # Si la velocidad real es mayor que un umbral intermedio, usamos el TTC de alta velocidad
        v_midpoint = (self.v_slow + self.v_fast) / 2.0
        current_ttc_limit = self.ttc_fast if abs(v_real) > v_midpoint else self.ttc_slow

        # 3. EVALUACIÓN GEOMÉTRICA (Túnel Recto vs Curvo)
        if abs(w_cmd) < self.turn_threshold:
            in_lane_mask = np.abs(y_rel) < self.half_width
        else:
            R = v_real / w_cmd
            dist_to_icr = np.sqrt(x_rel**2 + (y_rel - R)**2)
            in_lane_mask = np.abs(dist_to_icr - abs(R)) < self.half_width

        distances_in_path = long_distances[in_lane_mask]

        if len(distances_in_path) == 0:
            return True, 0.0, "NONE"

        # 4. CAPA 1: Burbuja Estática
        min_dist_found = np.min(distances_in_path)
        if min_dist_found < self.min_dist:
            return False, min_dist_found, "BUBBLE"

        # 5. CAPA 2: TTC (Usando el límite seleccionado en el paso 2)
        ttc_values = distances_in_path / abs(v_real)
        danger_ttc = ttc_values[ttc_values < current_ttc_limit]
        
        if len(danger_ttc) >= self.min_danger_points:
            return False, np.min(danger_ttc), "TTC"

        return True, 0.0, "NONE"

    def _emergency_stop(self, original_msg, reason):
        """Corta el PWM a cero inmediatamente."""
        self.get_logger().error(f"!!! FRENO ACTIVADO: {reason} !!!", throttle_duration_sec=0.5)
        
        stop_msg = TwistStamped()
        stop_msg.header = original_msg.header
        stop_msg.twist.linear.x = self.stop_pwm  # PWM parametrizado: Stop total
        stop_msg.twist.angular.z = original_msg.twist.angular.z # Mantiene ángulo (opcional)
        self.cmd_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FsmAebs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()