import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped 
from rclpy.qos import qos_profile_sensor_data
import math

class AEBSOverrideNode(Node):
    def __init__(self):
        super().__init__('aebs_override_node')

        # --- CONFIGURACIÓN DE GEOMETRÍA (Basado en tus Xacros) ---
        self.declare_parameter('robot_width', 0.44)
        self.robot_width = self.get_parameter('robot_width').get_parameter_value().double_value

        self.declare_parameter('safety_margin', 0.18)
        self.safety_margin = self.get_parameter('safety_margin').get_parameter_value().double_value

        self.declare_parameter('ttc_threshold', 0.5)
        self.ttc_threshold = self.get_parameter('ttc_threshold').get_parameter_value().double_value
            
        # Calculamos el medio ancho del "Túnel de Seguridad"
        # Si un obstáculo está dentro de este ancho Y, es peligroso. Si está fuera, lo ignoramos.
        self.safe_half_width = (self.robot_width / 2.0) + self.safety_margin
        
        self.latest_scan = None

        # Suscripciones y Publicadores
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data)

        self.joy_sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel_joy', 
            self.monitor_callback,
            10)

        self.override_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel_aebs', 
            10)
        
      
        self.ctrl_sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel_ctrl', 
            self.monitor_callback,
            10)


        self.get_logger().info(f'AEBS Geométrico Iniciado.')
        self.get_logger().info(f'Ancho Robot: {self.robot_width}m | Margen: {self.safety_margin}m')
        self.get_logger().info(f'Túnel de detección: +/- {self.safe_half_width:.3f}m del centro.')
        self.get_logger().info(f'Umbral de TTC: {self.ttc_threshold:.2f}s')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def calculate_ttc(self, intended_speed):
        """
        Calcula TTC usando filtrado cartesiano (Túnel de Seguridad).
        """
        if self.latest_scan is None:
            return float('inf')

        min_ttc = float('inf')
        
        # Pre-cálculos para eficiencia
        angle_min = self.latest_scan.angle_min
        angle_inc = self.latest_scan.angle_increment
        ranges = self.latest_scan.ranges

        for i, r in enumerate(ranges):
            # 1. Filtro de rango válido
            if not (self.latest_scan.range_min < r < self.latest_scan.range_max):
                continue

            # 2. Calcular ángulo
            theta = angle_min + (i * angle_inc)
            
            # 3. CONVERSIÓN A CARTESIANAS (La clave de la geometría)
            # Y = Distancia lateral respecto al centro del robot
            # X = Distancia frontal
            y_obstacle = r * math.sin(theta)
            x_obstacle = r * math.cos(theta)

            # 4. FILTRO GEOMÉTRICO (El "Túnel")
            # Si el obstáculo está más afuera que nuestro ancho seguro, lo ignoramos.
            # Esto permite pasar por pasillos donde las paredes están cerca pero no tocan.
            if abs(y_obstacle) > self.safe_half_width:
                continue

            # 5. FILTRO DE DIRECCIÓN
            # Si vamos adelante (speed > 0), solo nos importan obstáculos con X > 0 (adelante)
            # Si vamos atrás (speed < 0), solo nos importan obstáculos con X < 0 (atrás)
            # Nota: Usamos una pequeña zona muerta (0.05) para no detectar el propio chasis si el lidar está mal puesto
            if intended_speed > 0 and x_obstacle < 0.05:
                continue
            if intended_speed < 0 and x_obstacle > -0.05:
                continue

            # 6. CÁLCULO TTC (Proyección de velocidad)
            # closing_speed = Speed_X_component
            # Como ya filtramos por geometría, proyectamos la velocidad sobre el rayo
            closing_speed = intended_speed * math.cos(theta)

            if closing_speed > 0:
                ttc = r / closing_speed
                if ttc < min_ttc:
                    min_ttc = ttc
        
        return min_ttc

    def monitor_callback(self, msg):
        current_vx = msg.twist.linear.x
        
        # Calculamos riesgo si hay movimiento
        if abs(current_vx) > 0.01:
            
            ttc = self.calculate_ttc(current_vx)

            if ttc < self.ttc_threshold:
                # Logueamos solo cada 1 seg para no saturar
                self.get_logger().warning(f'¡COLISIÓN EN TRAYECTORIA! TTC: {ttc:.2f}s', throttle_duration_sec=1.0)
                
                override_msg = TwistStamped()
                override_msg.header.stamp = self.get_clock().now().to_msg()
                override_msg.header.frame_id = msg.header.frame_id
                
                override_msg.twist.linear.x = 0.0      # FRENAR
                override_msg.twist.angular.z = msg.twist.angular.z # Permitir esquivar
                
                self.override_pub.publish(override_msg)
                return

def main(args=None):
    rclpy.init(args=args)
    node = AEBSOverrideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()