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
        self.declare_parameter('kp', 2.5)  
        self.declare_parameter('max_speed', 2.0)  # Recomendado: 1.8 a 2.5 para diferencial
        self.declare_parameter('max_steering', 1.57)

        self.last_target_angle = 0.0

        # Leer parámetros
        self.kp = self.get_parameter('kp').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering = self.get_parameter('max_steering').value

        # QoS
        qos_profile = QoSProfile(depth=10)

        # 1. SUBSCRIBER
        self.angle_sub = self.create_subscription(
            TwistStamped, 
            '/gap_angle', 
            self.callback, 
            qos_profile
        )

        # 2. PUBLISHER
        self.vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel_ctrl', qos_profile)

        self.get_logger().info("TTC Control Node Initialized - Ultimate Racing Mode")
    
    def callback(self, msg):
        raw_angle = msg.twist.angular.z

        # Filtro de suavizado (Mantenido)
        alpha = 0.85
        target_angle = (alpha * raw_angle) + ((1.0 - alpha) * self.last_target_angle)
        self.last_target_angle = target_angle 

        # --- LEY DE CONTROL (Generación de Omega) ---
        # El target_angle (error angular hacia el hueco) se multiplica por Kp
        # para generar una VELOCIDAD ANGULAR (rad/s), no una posición de llanta.
        # El twist_cmd_node de abajo se encargará de convertir esta Omega al Delta físico.
        angular_cmd = self.kp * target_angle

        # Opcional: Podrías saturar la Omega aquí, pero el twist_cmd_node ya protegerá
        # la llanta físicamente, así que es redundante. Se mantiene por seguridad en simulación.
        angular_cmd = max(min(angular_cmd, self.max_steering), -self.max_steering)

        # ---------------------------------------------------------
        # LEY DE VELOCIDAD EXPONENCIAL (GAUSSIANA)
        # ---------------------------------------------------------
        min_survival_speed = 0.5 # Velocidad base para que la dirección funcione
        k_factor = 0.6           # Qué tan agresivo es el frenado (Tuning param)
        
        # El decaimiento es 1.0 (100%) cuando angular_cmd es 0, y tiende a 0.0 en giros fuertes
        decay = np.exp(-k_factor * (angular_cmd ** 2))
        
        # Rango de velocidad disponible para jugar
        speed_range = self.max_speed - min_survival_speed
        
        # Velocidad final: El mínimo vital + el rango extra multiplicado por el decaimiento
        linear_cmd = min_survival_speed + (speed_range * decay)

        # --- LEY DE VELOCIDAD ADAPTATIVA (Seguridad Ackermann) ---
        # Frenamos fuertemente en las curvas para evitar el derrape
        #min_survival_speed = 0.3 # El carro debe avanzar para poder rotar
        
        # ratio de qué tan "fuerte" estamos pidiendo girar respecto a lo que puede el carro
        #steering_ratio = abs(angular_cmd) / self.max_steering
        
        # Caída drástica en curvas cerradas (hasta el min_survival_speed)
        #linear_cmd = self.max_speed * (1.0 - (0.8 * steering_ratio))
        #linear_cmd = max(linear_cmd, min_survival_speed)

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