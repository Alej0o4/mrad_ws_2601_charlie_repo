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
        self.declare_parameter('robot_width', 0.40)
        self.declare_parameter('ttc_threshold', 0.7) # Este es el valor BASE (para Tunnel)
        self.declare_parameter('aebs_mode', 'tunnel') 
        self.declare_parameter('safety_factor', 2.0) # Factor de seguridad para ajustar el umbral dinámicamente
        self.declare_parameter('lidar_yaw_offset', 3.14159)
        self.declare_parameter('chassis_length', 0.37)
        
        # Carga inicial de valores
        self.width = self.get_parameter('robot_width').value
        self.base_ttc = self.get_parameter('ttc_threshold').value # Guardamos el valor base
        self.mode = self.get_parameter('aebs_mode').value
        self.safety_factor = self.get_parameter('safety_factor').value
        self.half_width = (self.width / 2.0) + 0.1 
        self.yaw_offset = self.get_parameter('lidar_yaw_offset').value
        self.bumper_x = self.get_parameter('chassis_length').value / 2.0

        # [NEW] Variable para el TTC efectivo (el que realmente se usa)
        self.effective_ttc = self.base_ttc 
        self.u_k = 0.0 # Velocidad actual del robot (para suavizado)
        self.alpha = 0.4 # Factor de suavizado para la velocidad (0.0 = sin suavizado, 1.0 = muy suave)

        # [NEW] Control de latencia y salud del sensor
        self.last_scan_time = self.get_clock().now()
        self.scan_timeout_sec = 0.5 # 500 ms sin datos = Freno automático
        
        # [NEW] Precomputar ángulos genéricos (evita recalcular el linspace)
        self.base_angles = None
        
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
        self.scan_sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_callback, qos_sensor)

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
        self.last_scan_time = self.get_clock().now()
        raw_ranges = np.array(msg.ranges)
        
        if self.base_angles is None or len(self.base_angles) != len(raw_ranges):
            # 1. Generar ángulos crudos del sensor
            angles = np.linspace(msg.angle_min, msg.angle_max, len(raw_ranges))
            # 2. Aplicar rotación física (Alineación con base_link)
            angles += self.yaw_offset
            # 3. Normalizar ángulos entre -pi y pi para evitar distorsión en funciones trigonométricas
            self.base_angles = np.arctan2(np.sin(angles), np.cos(angles))
            
        # Filtro Sim2Real Reforzado: Elevamos el mínimo a 15cm (0.15m) para no ver los tornillos del chasis
        valid = np.isfinite(raw_ranges) & (raw_ranges > 0.15) & (raw_ranges < msg.range_max)
        
        self.ranges = raw_ranges[valid]
        self.angles = self.base_angles[valid]

    def cmd_callback(self, msg):
        if self.ranges is None: 
            return 

        # [CRÍTICO] Chequeo de salud del sensor (Watchdog)
        time_since_last_scan = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if time_since_last_scan > self.scan_timeout_sec:
            self.get_logger().fatal("¡PÉRDIDA DE SEÑAL LIDAR! Aplicando freno de emergencia.")
            self.stop_robot(msg, 0.0)
            return

        vx = msg.twist.linear.x
        
        if abs(vx) < 0.05:
            self.cmd_pub.publish(msg)
            return

        is_safe = True
        min_ttc = float('inf')

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
            # 1. Miramos solo lo que está adelante del parachoques frontal
            direction_mask = x_points > self.bumper_x
            tunnel_mask = direction_mask & (np.abs(y_points) < self.half_width)
            dangers_x = x_points[tunnel_mask]
            
            if len(dangers_x) > 0:
                # [CORRECCIÓN CRÍTICA]: TTC real = (Distancia al LiDAR - Longitud al parachoques) / velocidad
                real_distances = dangers_x - self.bumper_x
                ttc_values = real_distances / vx 
                
                ttc_values = ttc_values[ttc_values > 0] 
                if len(ttc_values) > 0:
                    danger_rays = ttc_values[ttc_values < self.effective_ttc]
                    if len(danger_rays) >= 3: # Umbral de consenso Sim2Real
                        min_ttc = np.min(danger_rays)
                        return False, min_ttc 

        else: # Movimiento en REVERSA
            # 1. Miramos solo lo que está detrás del parachoques trasero
            direction_mask = x_points < -self.bumper_x
            tunnel_mask = direction_mask & (np.abs(y_points) < self.half_width)
            dangers_x = x_points[tunnel_mask]
            
            if len(dangers_x) > 0:
                # [CORRECCIÓN CRÍTICA]: Ambos 'dangers_x' y 'bumper_x' son negativos aquí.
                # Al sumarlos y dividirlos por vx (que también es negativo), el TTC da positivo.
                real_distances = dangers_x + self.bumper_x
                ttc_values = real_distances / vx
                
                ttc_values = ttc_values[ttc_values > 0] 
                if len(ttc_values) > 0:
                    danger_rays = ttc_values[ttc_values < self.effective_ttc]
                    if len(danger_rays) >= 3: 
                        min_ttc = np.min(danger_rays)
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

            # [CORRECCIÓN CRÍTICA]: Restamos el radio de impacto aproximado del robot
            real_distances = relevant_ranges - self.bumper_x
            
            # Evitamos procesar distancias que ya cruzaron el límite por error de hardware
            valid_dist_mask = real_distances > 0.0
            
            ttc_values = real_distances[valid_dist_mask] / relevant_speeds[valid_dist_mask]
            
            if len(ttc_values) > 0:
                min_ttc = np.min(ttc_values)

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