#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA # Para definir colores

class DistFinder(Node):
    def __init__(self):
        super().__init__("dist_finder")

        # --- PARAMETERS ---
        # Distancia deseada a la pared
        self.declare_parameter('desired_distance', 1.5)
        self.desired_distance = self.get_parameter('desired_distance').get_parameter_value().double_value

        # Lado de la pared: Left (1) or Right (-1)
        self.declare_parameter('desired_wall_side', -1) 
        self.desired_wall_side = self.get_parameter('desired_wall_side').get_parameter_value().integer_value

        # Lookahead Distance (L): Qué tan adelante proyectamos el error
        self.declare_parameter('lookahead_dist', 0.5) 
        self.lookahead_dist = self.get_parameter('lookahead_dist').get_parameter_value().double_value

        # Ángulos
        # Rayo B: Perpendicular al robot (90 grados)
        self.declare_parameter('ray_b_angle', self.desired_wall_side * np.pi/2) 
        self.ray_b_angle = self.get_parameter('ray_b_angle').get_parameter_value().double_value

        # Theta: Separación entre rayos (45 grados)
        self.declare_parameter('theta', np.radians(60)) 
        self.theta = self.get_parameter('theta').get_parameter_value().double_value

        # QoS
        qos_profile = QoSProfile(depth=10)

        # Subs / Pubs
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callback, qos_profile)
        self.marker_publisher = self.create_publisher(Marker, '/debug_rays', qos_profile) # Para visualización en RViz
        self.error_publisher = self.create_publisher(Float32, '/error', qos_profile)

        self.get_logger().info("DistFinder Node Initialized")
        self.get_logger().info(f"Desired Distance: {self.desired_distance} | Wall Side: {'Left' if self.desired_wall_side == 1 else 'Right'} | Lookahead: {self.lookahead_dist}")

    def callback(self, msg):
        # 1. Obtener distancias a y b
        b, a = self.getRange(msg)

        # Si el lidar da infinito o error, salimos para no dividir por cero
        if np.isinf(b) or np.isinf(a) or np.isnan(b) or np.isnan(a):
            return

        # 2. Calcular Alpha (Ángulo del robot respecto a la pared)
        # Fórmula: arctan( (a*cos(th) - b) / (a*sin(th)) )
        numerator = (a * np.cos(self.theta)) - b
        denominator = a * np.sin(self.theta)
        
        # Evitar división por cero
        if denominator == 0:
            return
            
        alpha = np.arctan(numerator / denominator)

        # 3. Calcular Distancia Actual (Dt) (AB en las diapositivas)
        dist_t = b * np.cos(alpha)

        # 4. Calcular Distancia Futura (Dt+1) (CD en las diapositivas)
        # Aquí usamos una distancia definida o también se puede hacer proporcional a la velocidad pero en este caso se dejó un valor constante (AC)
        # Dt+1 = Dt + AC * sin(alpha)
        dist_t_1 = dist_t + self.lookahead_dist * np.sin(alpha)

        # 5. Calcular Error (CORREGIDO PARA ROS)
        # Lógica:
        # - Pared Izq (1):  Queremos (Actual - Deseada)  -> Si Actual < Deseada => Negativo => Gira Derecha
        # - Pared Der (-1): Queremos (Deseada - Actual)  -> Si Actual < Deseada => Positivo => Gira Izquierda
        
        error = (dist_t_1 - self.desired_distance) * self.desired_wall_side

        # Publicar
        msg_error = Float32()
        msg_error.data = float(error) # Asegurar que es float de python
        self.error_publisher.publish(msg_error)
        self.publish_debug_rays(b, a, msg.header)
        
        # Debug (Opcional)
        # self.get_logger().info(f"Dt: {dist_t:.2f} | Futura: {dist_t_1:.2f} | Error: {error:.2f}")

    def getRange(self, msg):
        # Rayo B: A 90 grados
        ray_b_angle = self.ray_b_angle
        
        # Si pared izquierda (90): A = 90 - 45 = 45 (adelante-izq)
        # Si pared derecha (-90): A = -90 - (-45) = -45 (adelante-der)
        ray_a_angle = ray_b_angle - (self.theta * self.desired_wall_side)

        # Convertir ángulos a índices del array
        # Formula: index = (angle - min_angle) / increment
        ray_b_index = int((ray_b_angle - msg.angle_min) / msg.angle_increment)
        ray_a_index = int((ray_a_angle - msg.angle_min) / msg.angle_increment)

        # Protección de índices (por si se salen del array 0-360)
        num_readings = len(msg.ranges)
        ray_b_index = np.clip(ray_b_index, 0, num_readings - 1)
        ray_a_index = np.clip(ray_a_index, 0, num_readings - 1)

        # Obtener rangos
        b_range = msg.ranges[ray_b_index]
        a_range = msg.ranges[ray_a_index]

        return b_range, a_range
    
    


    def publish_debug_rays(self, dist_b, dist_a,header_lidar):
        marker = Marker()
        marker.header = header_lidar
        marker.ns = "wall_follower_rays"
        marker.id = 0
        marker.type = Marker.LINE_LIST # Dibujará líneas entre pares de puntos
        marker.action = Marker.ADD
        
        # Grosor de la línea
        marker.scale.x = 0.01 

        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Para evitar que se queden líneas viejas "congeladas" si el nodo se detiene
        # o que Rviz parpadee. Le decimos que el marcador dura 0.1 segundos.
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200000000 # 0.1 segundos

        # CALCULAR COORDENADAS CARTESIANAS
        # X = dist * cos(theta), Y = dist * sin(theta)
        
        # Ángulos actuales (Recalculamos lo mismo que en getRange para dibujar)
        angle_b = self.ray_b_angle
        angle_a = angle_b - (self.theta * self.desired_wall_side)

        # Punto B (Final del rayo)
        pb_x = dist_b * np.cos(angle_b)
        pb_y = dist_b * np.sin(angle_b)

        # Punto A (Final del rayo)
        pa_x = dist_a * np.cos(angle_a)
        pa_y = dist_a * np.sin(angle_a)

        # DEFINIR PUNTOS (Origen -> Destino)
        p_origin = Point(x=0.0, y=0.0, z=0.0) # Origen del sensor
        p_b = Point(x=pb_x, y=pb_y, z=0.0)
        p_a = Point(x=pa_x, y=pa_y, z=0.0)

        # Lista de puntos: [Origen, DestinoB, Origen, DestinoA]
        marker.points = [p_origin, p_b, p_origin, p_a]

        # DEFINIR COLORES (RGBA)
        # Color para Rayo B (MAGENTA)
        c_b = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0) 
        # Color para Rayo A (CYAN)
        c_a = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)

        # Asignar colores a cada vértice
        marker.colors = [c_b, c_b, c_a, c_a]

        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = DistFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()