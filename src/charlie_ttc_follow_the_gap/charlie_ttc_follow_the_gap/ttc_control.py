#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped

class TtcControl(Node): 
    def __init__(self):
        super().__init__("ttc_control") 

        # --- PARÁMETROS ---
        # Kp: Convierte grados de error en velocidad de giro (rad/s)
        self.declare_parameter('kp', 1.5)  
        # Velocidad máxima en recta (m/s) - ¡Sube esto si quieres ganar la carrera!
        self.declare_parameter('max_speed', 1.5) 
        # Límite físico de giro del robot (rad/s)
        self.declare_parameter('max_steering', 1.0)

        self.last_target_angle = 0.0

        # Leer parámetros
        self.kp = self.get_parameter('kp').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering = self.get_parameter('max_steering').value

        # QoS
        qos_profile = QoSProfile(depth=10)

        # 1. SUBSCRIBER
        # Escuchamos el ángulo que calculó el nodo 'ttc_gap_finder'
        self.angle_sub = self.create_subscription(
            TwistStamped, 
            '/gap_angle', 
            self.callback, 
            qos_profile
        )

        # 2. PUBLISHER
        # Publicamos el comando final hacia el Mux o el Robot
        self.vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel_ctrl', qos_profile)

        self.get_logger().info("TTC Control Node Initialized")
    
    def callback(self, msg):
        # msg viene del gap_finder. 
        # msg.twist.angular.z contiene el ángulo hacia el centro del hueco (gap).
        
        #target_angle = msg.twist.angular.z
        raw_angle = msg.twist.angular.z

        # --- [MEJORA] FILTRO DE SUAVIZADO (Exponential Moving Average) ---
        # alpha controla la suavidad:
        # 1.0 = Sin suavizado (reacción instantánea, mucha oscilación)
        # 0.1 = Muy suave (reacción lenta, como un barco)
        # 0.6 es un buen equilibrio para Racing: reacciona rápido pero filtra el ruido.
        alpha = 0.52 
        
        target_angle = (alpha * raw_angle) + ((1.0 - alpha) * self.last_target_angle)
        self.last_target_angle = target_angle # Guardamos para la siguiente vez

        # --- LEY DE CONTROL DE DIRECCIÓN (STEERING) ---
        # Como el robot mira a 0, el error es: Deseado - Actual(0) = target_angle
        # Aplicamos control Proporcional
        angular_cmd = self.kp * target_angle

        # Saturación (Clamping)
        # Evitamos mandar comandos que el robot no puede ejecutar físicamente
        angular_cmd = max(min(angular_cmd, self.max_steering), -self.max_steering)

        # --- LEY DE CONTROL DE VELOCIDAD (THROTTLE) ---
        # Estrategia de Racing:
        # - Recta (ángulo 0) -> Velocidad Máxima
        # - Curva cerrada (ángulo alto) -> Velocidad Mínima
        # Fórmula: V = V_max / (1 + abs(giro) * factor)
        linear_cmd = self.max_speed / (1.0 + abs(angular_cmd))

        # --- PUBLICACIÓN ---
        out_msg = TwistStamped()
        
        # CRÍTICO: Copiar el header para mantener la sincronización de tiempo
        # Si el gap_finder detectó el hueco en T=100, este comando es para T=100.
        out_msg.header = msg.header
        
        out_msg.twist.linear.x = float(linear_cmd)
        out_msg.twist.angular.z = float(angular_cmd)
        
        self.vel_publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TtcControl() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()