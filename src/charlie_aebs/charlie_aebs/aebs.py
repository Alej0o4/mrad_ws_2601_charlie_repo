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
        self.declare_parameter('ttc_threshold', 0.7) 
        self.declare_parameter('aebs_mode', 'tunnel') 
        self.declare_parameter('safety_factor', 2.0) 
        self.declare_parameter('lidar_yaw_offset', 3.14159)
        self.declare_parameter('chassis_length', 0.37)
        
        # [NEW] Parámetro para la Burbuja de Seguridad (Arquitectura Híbrida)
        # Distancia mínima absoluta permitida desde el parachoques en metros.
        self.declare_parameter('min_safe_distance', 0.20) 
        
        self.width = self.get_parameter('robot_width').value
        self.base_ttc = self.get_parameter('ttc_threshold').value 
        self.mode = self.get_parameter('aebs_mode').value
        self.safety_factor = self.get_parameter('safety_factor').value
        self.half_width = (self.width / 2.0) + 0.1 
        self.yaw_offset = self.get_parameter('lidar_yaw_offset').value
        self.bumper_x = self.get_parameter('chassis_length').value / 2.0
        self.min_safe_dist = self.get_parameter('min_safe_distance').value

        self.effective_ttc = self.base_ttc 
        self.u_k = 0.0 
        self.alpha = 0.4 

        self.last_scan_time = self.get_clock().now()
        self.scan_timeout_sec = 0.5 
        
        self.base_angles = None
        self.update_thresholds()
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.ranges = None
        self.angles = None

        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel_raw', self.cmd_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_callback, qos_sensor)
        self.cmd_pub = self.create_publisher(TwistStamped, '/diffdrive_controller/cmd_vel', 10)

        self.get_logger().info(f"AEBS System Active. Hybrid Architecture (TTC + Bubble) Enabled.")

    def update_thresholds(self):
        if self.mode == 'radial':
            self.effective_ttc = self.base_ttc * self.safety_factor
        else:
            self.effective_ttc = self.base_ttc

    def parameters_callback(self, params):
        update_needed = False
        for param in params:
            if param.name == 'aebs_mode':
                if param.value in ['tunnel', 'radial']:
                    self.mode = param.value
                    update_needed = True
            if param.name == 'ttc_threshold':
                self.base_ttc = param.value
                update_needed = True
            if param.name == 'safety_factor':
                self.safety_factor = param.value
                update_needed = True
            # [NEW] Actualización dinámica de la burbuja
            if param.name == 'min_safe_distance':
                self.min_safe_dist = param.value
        
        if update_needed:
            self.update_thresholds() 
                
        return SetParametersResult(successful=True)

    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()
        raw_ranges = np.array(msg.ranges)
        
        if self.base_angles is None or len(self.base_angles) != len(raw_ranges):
            angles = np.linspace(msg.angle_min, msg.angle_max, len(raw_ranges))
            angles += self.yaw_offset
            self.base_angles = np.arctan2(np.sin(angles), np.cos(angles))
            
        valid = np.isfinite(raw_ranges) & (raw_ranges > 0.15) & (raw_ranges < msg.range_max)
        self.ranges = raw_ranges[valid]
        self.angles = self.base_angles[valid]

    def cmd_callback(self, msg):
        if self.ranges is None: 
            return 

        time_since_last_scan = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if time_since_last_scan > self.scan_timeout_sec:
            self.get_logger().fatal("¡PÉRDIDA DE SEÑAL LIDAR! Aplicando freno de emergencia.")
            self.stop_robot(msg, 0.0, "TIMEOUT")
            return

        vx = msg.twist.linear.x
        
        if abs(vx) < 0.05:
            self.cmd_pub.publish(msg)
            return

        is_safe = True
        trigger_val = float('inf')
        trigger_type = "NONE"

        if self.mode == 'tunnel':
            is_safe, trigger_val, trigger_type = self.check_tunnel_safety(vx)
        elif self.mode == 'radial':
            is_safe, trigger_val, trigger_type = self.check_radial_safety(vx)

        if not is_safe:
            self.stop_robot(msg, trigger_val, trigger_type)
        else:
            self.u_k = msg.twist.linear.x
            self.cmd_pub.publish(msg)

    def check_tunnel_safety(self, vx):
        x_points = self.ranges * np.cos(self.angles)
        y_points = self.ranges * np.sin(self.angles)

        if vx > 0:
            direction_mask = x_points > self.bumper_x
            tunnel_mask = direction_mask & (np.abs(y_points) < self.half_width)
            dangers_x = x_points[tunnel_mask]
            
            if len(dangers_x) > 0:
                real_distances = dangers_x - self.bumper_x
                
                # [NEW] Capa 1: Burbuja de Seguridad Estática (Absoluta)
                if np.any(real_distances < self.min_safe_dist):
                    return False, np.min(real_distances), "BUBBLE"

                # Capa 2: TTC Dinámico
                ttc_values = real_distances / vx 
                ttc_values = ttc_values[ttc_values > 0] 
                if len(ttc_values) > 0:
                    danger_rays = ttc_values[ttc_values < self.effective_ttc]
                    if len(danger_rays) >= 3: 
                        return False, np.min(danger_rays), "TTC" 

        else: # Movimiento en REVERSA
            direction_mask = x_points < -self.bumper_x
            tunnel_mask = direction_mask & (np.abs(y_points) < self.half_width)
            dangers_x = x_points[tunnel_mask]
            
            if len(dangers_x) > 0:
                real_distances = np.abs(dangers_x + self.bumper_x) # Ajustado a absoluto para la burbuja
                
                # [NEW] Capa 1: Burbuja de Seguridad
                if np.any(real_distances < self.min_safe_dist):
                    return False, np.min(real_distances), "BUBBLE"

                ttc_values = real_distances / abs(vx)
                ttc_values = ttc_values[ttc_values > 0] 
                if len(ttc_values) > 0:
                    danger_rays = ttc_values[ttc_values < self.effective_ttc]
                    if len(danger_rays) >= 3: 
                        return False, np.min(danger_rays), "TTC" 

        return True, 0.0, "NONE"

    def check_radial_safety(self, vx):
        closing_speeds = vx * np.cos(self.angles)
        danger_mask = closing_speeds > 0.01 

        if np.any(danger_mask):
            relevant_ranges = self.ranges[danger_mask]
            relevant_speeds = closing_speeds[danger_mask]

            real_distances = relevant_ranges - self.bumper_x
            valid_dist_mask = real_distances > 0.0
            
            # [NEW] Capa 1: Burbuja de Seguridad
            if np.any(real_distances[valid_dist_mask] < self.min_safe_dist):
                return False, np.min(real_distances[valid_dist_mask]), "BUBBLE"
            
            # Capa 2: TTC
            ttc_values = real_distances[valid_dist_mask] / relevant_speeds[valid_dist_mask]
            if len(ttc_values) > 0:
                min_ttc = np.min(ttc_values)
                if min_ttc < self.effective_ttc:
                    return False, min_ttc, "TTC" 

        return True, 0.0, "NONE"
    
    def stop_robot(self, original_msg, trigger_val, trigger_type):
        direction_str = "ADELANTE" if original_msg.twist.linear.x > 0 else "ATRÁS"
        
        # Logueamos qué límite disparó el freno (TTC o Burbuja)
        if trigger_type == "BUBBLE":
            msg_warn = f"[{self.mode.upper()}] FRENO {direction_str}! BURBUJA INVASIÓN: {trigger_val:.2f}m (Lim: {self.min_safe_dist:.2f}m)"
        else:
            msg_warn = f"[{self.mode.upper()}] FRENO {direction_str}! TTC: {trigger_val:.2f}s (Lim: {self.effective_ttc:.2f}s)"

        self.get_logger().warn(msg_warn, throttle_duration_sec=0.5)
        
        alpha = 0.1
        gain = 1.0

        self.u_k = alpha * self.u_k + (1 - alpha) * 0.0
        safe_msg = TwistStamped()
        safe_msg.header = original_msg.header 
        safe_msg.twist.linear.x = self.u_k 
        safe_msg.twist.angular.z = original_msg.twist.angular.z * gain
        
        self.cmd_pub.publish(safe_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AEBSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()