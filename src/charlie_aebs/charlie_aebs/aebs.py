import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import SetParametersResult

import numpy as np

class AEBSNode(Node):
    def __init__(self):
        super().__init__('aebs_node')

        # --- PARÁMETROS ---
        self.declare_parameter('robot_width', 0.44)
        self.declare_parameter('ttc_threshold', 0.7) # Este es el valor BASE (para Tunnel)
        self.declare_parameter('aebs_mode', 'tunnel') 
        self.declare_parameter('safety_factor', 2.0) # Factor de seguridad para ajustar el umbral dinámicamente
        
        # Carga inicial de valores
        self.width = self.get_parameter('robot_width').value
        self.base_ttc = self.get_parameter('ttc_threshold').value # Guardamos el valor base
        self.mode = self.get_parameter('aebs_mode').value
        self.safety_factor = self.get_parameter('safety_factor').value
        self.half_width = (self.width / 2.0) + 0.1 

        # [NEW] Variable para el TTC efectivo (el que realmente se usa)
        self.effective_ttc = self.base_ttc 
        self.u_k = 0.0 # Velocidad actual del robot (para suavizado)
        self.alpha = 0.4 # Factor de suavizado para la velocidad (0.0 = sin suavizado, 1.0 = muy suave)
        
        # [NEW] Calculamos el umbral inicial según el modo configurado
        self.update_thresholds()

        # Callback para cambiar parámetros en tiempo real
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Variables de estado
        self.ranges = None
        self.angles = None

        # QoS para Lidar
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- SUSCRIPCIONES ---
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel_raw', self.cmd_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_sensor)

        # --- PUBLICACIÓN ---
        self.cmd_pub = self.create_publisher(TwistStamped, '/diffdrive_controller/cmd_vel', 10)

        self.u_k = 0.

        self.get_logger().info(f"AEBS System Active.")

    def update_thresholds(self):
        """
        [NEW] Lógica central de umbrales.
        Define qué TTC usar dependiendo del modo.
        """
        if self.mode == 'radial':
            # En modo radial, somos más conservadores (doble de tiempo)
            self.effective_ttc = self.base_ttc * self.safety_factor
        else:
            # En modo túnel, usamos el valor base para permitir maniobras cerradas
            self.effective_ttc = self.base_ttc

        self.get_logger().info(
            f"CONFIG -> Modo: {self.mode.upper()} | Factor: {self.safety_factor:.1f}x | TTC ACTIVO: {self.effective_ttc:.2f}s"
        )

    def parameters_callback(self, params):
        """Permite cambiar el modo o el umbral sin reiniciar el nodo"""
        update_needed = False
        
        for param in params:
            if param.name == 'aebs_mode':
                if param.value in ['tunnel', 'radial']:
                    self.mode = param.value
                    update_needed = True
                else:
                    return SetParametersResult(successful=False, reason="Mode must be 'tunnel' or 'radial'")
            
            if param.name == 'ttc_threshold':
                self.base_ttc = param.value
                update_needed = True

            if param.name == 'safety_factor':
                self.safety_factor = param.value
                update_needed = True
        
        if update_needed:
            self.update_thresholds() # Recalculamos el effective_ttc automáticamente
                
        return SetParametersResult(successful=True)

    def scan_callback(self, msg):
        self.ranges = np.array(msg.ranges)
        
        if self.angles is None or len(self.angles) != len(msg.ranges):
            self.angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            
        valid = np.isfinite(self.ranges) & (self.ranges > msg.range_min) & (self.ranges < msg.range_max)
        self.ranges = self.ranges[valid]
        self.angles = self.angles[valid]

    def cmd_callback(self, msg):
        if self.ranges is None: 
            return 

        vx = msg.twist.linear.x
        
        if abs(vx) < 0.05:
            self.cmd_pub.publish(msg)
            return

        is_safe = True
        min_ttc = float('inf')

        # --- SELECCIÓN DE ALGORITMO ---
        if self.mode == 'tunnel':
            is_safe, min_ttc = self.check_tunnel_safety(vx)
        elif self.mode == 'radial':
            is_safe, min_ttc = self.check_radial_safety(vx)

        # --- ACTUACIÓN ---
        if not is_safe:
            self.stop_robot(msg, min_ttc)
        else:
            self.u_k = msg.twist.linear.x
            self.u_k = msg.twist.linear.x
            self.cmd_pub.publish(msg)

    # ---------------------------------------------------------
    # MODO 1: TUNNEL 
    # ---------------------------------------------------------
    def check_tunnel_safety(self, vx):
        x_points = self.ranges * np.cos(self.angles)
        y_points = self.ranges * np.sin(self.angles)

        if vx > 0:
            direction_mask = x_points > 0
        else:
            direction_mask = x_points < 0

        tunnel_mask = direction_mask & (np.abs(y_points) < self.half_width)
        dangers_x = x_points[tunnel_mask]
        
        if len(dangers_x) > 0:
            ttc_values = dangers_x / vx 
            ttc_values = ttc_values[ttc_values > 0] 

            if len(ttc_values) > 0:
                min_ttc = np.min(ttc_values)
                # [NEW] Usamos effective_ttc en lugar de self.ttc_min
                if min_ttc < self.effective_ttc:
                    return False, min_ttc 

        return True, 0.0

    # ---------------------------------------------------------
    # MODO 2: RADIAL TTC 
    # ---------------------------------------------------------
    def check_radial_safety(self, vx):
        closing_speeds = vx * np.cos(self.angles)
        danger_mask = closing_speeds > 0.01 

        if np.any(danger_mask):
            relevant_ranges = self.ranges[danger_mask]
            relevant_speeds = closing_speeds[danger_mask]

            ttc_values = relevant_ranges / relevant_speeds
            min_ttc = np.min(ttc_values)

            # [NEW] Usamos effective_ttc (que aquí será el doble)
            if min_ttc < self.effective_ttc:
                return False, min_ttc 

        return True, 0.0

    def stop_robot(self, original_msg, ttc_val):
        direction_str = "ADELANTE" if original_msg.twist.linear.x > 0 else "ATRÁS"
        # Logueamos qué límite disparó el freno
        self.get_logger().warn(
            f"[{self.mode.upper()}] FRENO {direction_str}! TTC: {ttc_val:.2f}s (Lim: {self.effective_ttc:.2f}s)", 
            throttle_duration_sec=0.5
        )
        
        alpha = 0.1
        gain = 1.

        self.u_k = alpha * self.u_k + (1 - alpha) * 0.0
        safe_msg = TwistStamped()
        safe_msg.header = original_msg.header 
        safe_msg.twist.linear.x = self.u_k 
        safe_msg.twist.angular.z = original_msg.twist.angular.z*gain
        
        self.cmd_pub.publish(safe_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AEBSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()