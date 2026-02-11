import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped # <--- CAMBIO 1: Importar TwistStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

class AEBSNode(Node):
    def __init__(self):
        super().__init__('aebs_node')

        # Parámetros
        self.declare_parameter('robot_width', 0.44)
        self.declare_parameter('ttc_threshold', 0.7)
        
        self.width = self.get_parameter('robot_width').value
        self.ttc_min = self.get_parameter('ttc_threshold').value
        self.half_width = (self.width / 2.0) + 0.1 # +10cm margen

        # Variables para Numpy
        self.ranges = None
        self.angles = None

        # QoS para Lidar
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 1. SUSCRIPCIÓN (Input) - TwistStamped
        # Escuchamos al topic _raw que sale de tu TwistMux
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel_raw', self.cmd_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_sensor)

        # 2. PUBLICACIÓN (Output) - TwistStamped
        # Publicamos al topic final que escucha tu controlador
        self.cmd_pub = self.create_publisher(TwistStamped, '/diffdrive_controller/cmd_vel', 10)

        self.get_logger().info("AEBS System Active (TwistStamped Mode)")

    def scan_callback(self, msg):
        # Conversión a Numpy (Igual que antes)
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
        
        # 1. Chequeamos si hay movimiento significativo (Adelante O Atrás)
        if abs(vx) > 0.05:
            
            # --- CÁLCULO VECTORIAL ---
            x_points = self.ranges * np.cos(self.angles)
            y_points = self.ranges * np.sin(self.angles)

            # 2. DEFINIR LA MÁSCARA DE DIRECCIÓN (La clave del cambio)
            if vx > 0:
                # Si voy adelante -> Busco obstáculos con X positiva
                direction_mask = x_points > 0
            else:
                # Si voy atrás -> Busco obstáculos con X negativa
                direction_mask = x_points < 0

            # 3. Filtro de Túnel (Funciona igual para ambos lados)
            # Queremos puntos en la dirección del movimiento Y dentro del ancho
            tunnel_mask = direction_mask & (np.abs(y_points) < self.half_width)
            
            # Obtenemos las coordenadas X de los obstáculos peligrosos
            dangers_x = x_points[tunnel_mask]
            
            if len(dangers_x) > 0:
                # 4. Cálculo de TTC
                # Matemáticamente funciona perfecto para reversa:
                # Distancia (-2m) / Velocidad (-1m/s) = TTC Positivo (2s)
                ttc_values = dangers_x / vx 
                
                # OJO: A veces por ruido numérico puede salir un TTC negativo pequeño, 
                # filtramos para quedarnos solo con los positivos reales.
                ttc_values = ttc_values[ttc_values > 0]

                if len(ttc_values) > 0:
                    min_ttc = np.min(ttc_values)

                    if min_ttc < self.ttc_min:
                        direction_str = "ADELANTE" if vx > 0 else "ATRÁS"
                        self.get_logger().warn(f"FRENO {direction_str}! TTC: {min_ttc:.2f}s", throttle_duration_sec=0.5)
                        
                        safe_msg = TwistStamped()
                        safe_msg.header = msg.header 
                        safe_msg.twist.linear.x = 0.0
                        safe_msg.twist.angular.z = msg.twist.angular.z
                        
                        self.cmd_pub.publish(safe_msg)
                        return

        # Si no hay peligro o estamos quietos, republicamos
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AEBSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()