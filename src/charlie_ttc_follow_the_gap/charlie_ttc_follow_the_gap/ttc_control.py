#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped
import numpy as np

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
        raw_angle = msg.twist.angular.z
        geometric_multiplier = msg.twist.linear.x  # <--- RECIBIMOS EL MULTIPLICADOR

        # Filtro de suavizado
        alpha = 0.85
        target_angle = (alpha * raw_angle) + ((1.0 - alpha) * self.last_target_angle)
        self.last_target_angle = target_angle 

        # --- LEY DE CONTROL (Generación de Omega) ---
        angular_cmd = self.kp * target_angle
        angular_cmd = max(min(angular_cmd, self.max_steering), -self.max_steering)

        # ---------------------------------------------------------
        # LEY DE VELOCIDAD ADAPTATIVA (Fusión Geometría + Cinemática)
        # ---------------------------------------------------------
        min_survival_speed = 0.3 
        k_factor = 0.8           
        
        # 1. Aplicamos el castigo por entorno (Pasillos estrechos o paredes de frente)
        dynamic_max_speed = self.max_speed * geometric_multiplier
        
        # 2. Aplicamos el castigo por giro (Campana de Gauss para no derrapar en curva)
        decay = np.exp(-k_factor * (angular_cmd ** 2))
        
        # Rango de velocidad disponible basado en el nuevo tope dinámico
        speed_range = dynamic_max_speed - min_survival_speed
        
        # Velocidad final segura
        linear_cmd = min_survival_speed + (speed_range * decay)

        # --- PUBLICACIÓN ---
        out_msg = TwistStamped()
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