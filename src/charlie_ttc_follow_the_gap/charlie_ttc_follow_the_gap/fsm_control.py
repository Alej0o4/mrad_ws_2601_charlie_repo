import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math

class FsmControl(Node):
    def __init__(self):
        super().__init__('fsm_control')

        # --- PARÁMETROS GEOMÉTRICOS Y DE CONTROL ---
        self.declare_parameter('wheelbase', 0.257)      
        self.declare_parameter('max_steering_deg', 20.0) 
        self.declare_parameter('kp', 1.0)                
        self.declare_parameter('steering_alpha', 0.7)    # Suavizado del servo (0.0 a 1.0)
        
        # --- PARÁMETROS EMPÍRICOS ---
        self.declare_parameter('v_slow_real', 0.412) 
        self.declare_parameter('v_fast_real', 1.255) 
        
        # --- UMBRALES DE TRANSICIÓN ---
        self.declare_parameter('dist_critical', 0.8)   
        self.declare_parameter('angle_critical_deg', 15.0) 
        self.declare_parameter('depth_critical', 2.0)    # NUEVO: Distancia de frenado predictivo

        # Cargar valores
        self.L = self.get_parameter('wheelbase').value
        self.delta_max = math.radians(self.get_parameter('max_steering_deg').value)
        self.kp = self.get_parameter('kp').value
        self.alpha = self.get_parameter('steering_alpha').value
        self.v_slow = self.get_parameter('v_slow_real').value
        self.v_fast = self.get_parameter('v_fast_real').value
        self.d_crit = self.get_parameter('dist_critical').value
        self.a_crit = math.radians(self.get_parameter('angle_critical_deg').value)
        self.depth_crit = self.get_parameter('depth_critical').value

        # Variables de estado interno
        self.last_steering = 0.0
        self.state = "IDLE"

        # ROS2
        self.data_sub = self.create_subscription(TwistStamped, '/gap_data', self.control_callback, 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/cmd_vel_ctrl', 10)

        self.get_logger().info("FSM Control Refactorizado Iniciado.")

    # ==========================================================
    # ORQUESTADOR PRINCIPAL (Callback)
    # ==========================================================
    def control_callback(self, msg):
        target_angle = msg.twist.angular.z
        closest_dist = msg.twist.linear.x
        target_depth = msg.twist.linear.y  # Usamos la métrica predictiva

        # 1. Determinar el estado y las velocidades objetivo
        self.state, v_cmd, v_real = self._determine_fsm_state(closest_dist, target_angle, target_depth)

        # 2. Calcular el comando de dirección suavizado y limitado dinámicamente
        final_steering = self._compute_steering(target_angle, v_real)

        # 3. Publicar hacia los motores / odometría
        self._publish_command(msg.header, v_cmd, final_steering)

        # 4. Debug ocasional (1 de cada 10 ciclos aprox para no saturar la terminal)
        if self.get_clock().now().nanoseconds % 10 == 0:
            w_max = self._calculate_dynamic_w_max(v_real)
            self.get_logger().info(f"St: {self.state} | w_max: {w_max:.2f} | Dist: {closest_dist:.2f} | Depth: {target_depth:.2f}")

    # ==========================================================
    # MÉTODOS PRIVADOS (Lógica de Negocio)
    # ==========================================================
    def _determine_fsm_state(self, closest_dist, target_angle, target_depth):
        """Evalúa las métricas y retorna: Estado, PWM_lineal, Velocidad_real_esperada."""
        if closest_dist < 0.2:
            return "IDLE", 0.0, 0.0
            
        # Condición para SLOW: Peligro inminente OR Curva cerrada OR Final del pasillo cerca
        if closest_dist < self.d_crit or abs(target_angle) > self.a_crit or target_depth < self.depth_crit:
            return "SLOW", 0.04, self.v_slow
            
        return "FAST", 0.05, self.v_fast

    def _calculate_dynamic_w_max(self, v_real):
        """Calcula el límite angular seguro basado en la cinemática de Ackermann."""
        if v_real <= 0.001:
            return 0.0
        return (v_real / self.L) * math.tan(self.delta_max)

    def _compute_steering(self, target_angle, v_real):
        """Aplica Control P, satura dinámicamente y suaviza la salida."""
        w_max = self._calculate_dynamic_w_max(v_real)
        
        # Control Proporcional y Saturación
        angular_out = target_angle * self.kp
        angular_out = max(min(angular_out, w_max), -w_max)

        # Suavizado (Low Pass Filter)
        final_steering = (self.alpha * angular_out) + (1.0 - self.alpha) * self.last_steering
        self.last_steering = final_steering
        
        return final_steering

    def _publish_command(self, header, v_cmd, steering_cmd):
        """Construye y publica el mensaje de control."""
        out_msg = TwistStamped()
        out_msg.header = header
        out_msg.twist.linear.x = float(v_cmd)
        out_msg.twist.angular.z = float(steering_cmd)
        self.vel_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FsmControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()