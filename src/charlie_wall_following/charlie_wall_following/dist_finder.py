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
        self.declare_parameter('desired_wall_side', 1) 
        self.desired_wall_side = self.get_parameter('desired_wall_side').get_parameter_value().integer_value

        # Lookahead Distance (L): Qué tan adelante proyectamos el error
        self.declare_parameter('lookahead_dist', 0.5) 
        self.lookahead_dist = self.get_parameter('lookahead_dist').get_parameter_value().double_value

        # Ángulos
        # Rayo B: Perpendicular al robot (90 grados)
        self.declare_parameter('ray_b_angle', self.desired_wall_side * np.pi/2) 
        self.ray_b_angle = self.get_parameter('ray_b_angle').get_parameter_value().double_value

        # Theta: Separación entre rayos (60 grados)
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
        # 1. Perception: Check surroundings
        total = len(msg.ranges)
        left_sector = np.array(msg.ranges[int(total*0.7):int(total*0.9)]) # Adjusted indices
        right_sector = np.array(msg.ranges[int(total*0.1):int(total*0.3)])
        
        # Filter valid points
        left_valid = left_sector[np.isfinite(left_sector)]
        right_valid = right_sector[np.isfinite(right_sector)]
        
        # 2. Logic: Choose wall (with a bit of stickiness/hysteresis)
        # Only change side if the current side is totally lost or front is blocked
        l_dist = np.mean(left_valid) if len(left_valid) > 0 else 5.0
        r_dist = np.mean(right_valid) if len(right_valid) > 0 else 5.0

        # Example: Simple logic to favor the closer wall
        side_threshold = 10000.8
        right_priority_bonus = 1000.4

        if self.desired_wall_side == -1:
            # Solo cambia a la izquierda si la pared izquierda está REALMENTE más cerca
            # que la derecha (incluyendo el bonus de prioridad)
            if l_dist < (r_dist - right_priority_bonus - side_threshold):
                self.desired_wall_side = 1
                self.get_logger().info("Cambiando a pared IZQUIERDA")
        
        # Si estamos en la izquierda (1), el cambio a la derecha es más fácil
        else:
            if r_dist < (l_dist - side_threshold):
                self.desired_wall_side = -1
                self.get_logger().info("Cambiando a pared DERECHA (Prioridad)")
        # si no supera el umbral → mantener lado actual


        self.ray_b_angle = self.desired_wall_side * np.pi/2    

        front_sector = np.array(msg.ranges[int(total*0.45):int(total*0.55)])
        front_valid = front_sector[np.isfinite(front_sector)]
        front_min = np.min(front_valid) if len(front_valid) > 0 else 5.0
        

        # if front_min < 1.0: # Si hay un obstáculo cercano al frente, cambiar de lado
        #     self.desired_wall_side = 1 if l_dist > r_dist else -1



        # 3. Calculation
        b, a = self.getRange(msg)
        
        # Fallback for missing data
        if not np.isfinite(a) or not np.isfinite(b):
            # Publish a 'safe' error to keep turning away from wall
            error = 0.5 * self.desired_wall_side
        else:
            # Standard Alpha/Dist calculation (Your existing math)
            num = (a * np.cos(self.theta)) - b
            den = a * np.sin(self.theta)
            alpha = np.arctan2(num, den) # Using arctan2 is safer
            
            dist_t = b * np.cos(alpha)
            dist_t_1 = dist_t + self.lookahead_dist * np.sin(alpha)
            
            # Calculate Error
            error = (dist_t_1 - self.desired_distance) * self.desired_wall_side

        # 4. Publish
        error_msg = Float32()
        error_msg.data = float(error)
        self.error_publisher.publish(error_msg)
        self.publish_debug_rays(b, a, msg.header)

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