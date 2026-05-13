import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np
from scipy.interpolate import splprep, splev
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

class PathSmoother(Node):
    def __init__(self):
        super().__init__('path_smoother_node')
        
        # Parámetros tuneables (¡Aquí está la magia del Racing!)
        self.declare_parameter('smoothing_factor', 2.0)  # Qué tanto permitimos que corte las esquinas
        self.declare_parameter('point_spacing', 0.05)     # Distancia exacta entre puntos (10 cm por defecto)
        self.declare_parameter('closed_loop_threshold', 0.5) # Umbral para detectar circuitos cerrados (en metros
        self.declare_parameter('max_steering_angle_deg', 20.0) # Ángulo máximo de dirección en grados (ajustable según tu vehículo)
        self.declare_parameter('wheelbase', 0.257) # Distancia entre ejes de tu vehículo en metros (ajusta según tu carro)
        


        # Parámetro cinemático del vehículo (Ajusta 'L' a la distancia entre ejes real en metros de tu carro)
        wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        max_steering_rad = math.radians(self.get_parameter('max_steering_angle_deg').get_parameter_value().double_value)
        self.min_turning_radius = wheelbase / math.tan(max_steering_rad)
        
        self.sub = self.create_subscription(Path, '/current_active_path', self.path_callback, 10)

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Aplicarlo al publicador de la ruta suavizada
        self.pub = self.create_publisher(Path, '/smoothed_path', latched_qos)
        
        self.get_logger().info('NODO DE SUAVIZADO GEOMÉTRICO (B-SPLINES) LISTO.')

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Convierte un ángulo de Euler (Yaw) a un Cuaternión de ROS 2."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def path_callback(self, msg: Path):
        # 1. Validación de seguridad
        if len(msg.poses) < 4:
            self.get_logger().warn("Ruta muy corta para un B-Spline cúbico. Publicando original...")
            self.pub.publish(msg)
            return

        # 2. Extraer coordenadas crudas del ARA*
        pts = []
        for pose in msg.poses:
            pts.append([pose.pose.position.x, pose.pose.position.y])
        pts = np.array(pts)

        # 3. Limpieza de datos (Crucial para SciPy)
        # Si ARA* o el Waypoint Manager enviaron dos puntos exactamente iguales seguidos, 
        # las matemáticas de SciPy colapsan con una división por cero. Los filtramos:
        diffs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        mask = np.insert(diffs > 1e-4, 0, True)
        pts = pts[mask]

        if len(pts) < 4:
            self.pub.publish(msg)
            return

        x = pts[:, 0]
        y = pts[:, 1]

        # ¿Es un circuito cerrado? (Distancia entre inicio y fin < 0.5m)
        is_closed_loop = np.linalg.norm(pts[0] - pts[-1]) < 0.5

        # Extraer parámetros actuales
        s_param = self.get_parameter('smoothing_factor').value
        spacing = self.get_parameter('point_spacing').value

        try:
            # ==========================================
            # EL MOTOR MATEMÁTICO (B-SPLINES)
            # ==========================================
            # tck: Tupla con los nudos y coeficientes de la curva
            # u: Parámetro que va de 0 (inicio) a 1 (fin)
            # k=3: Polinomio cúbico (C^2 continuo)
            per_flag = 1 if is_closed_loop else 0
            tck, _ = splprep([x, y], s=s_param, k=3)
            
            # 4. Remuestreo Equidistante
            # Para el MPC, necesitamos que los puntos estén a la misma distancia física.
            # Primero evaluamos una curva muy densa para medir la longitud real:
            dense_u = np.linspace(0, 1, len(x) * 10)
            eval_dense = splev(dense_u, tck)
            dx_dense = np.diff(eval_dense[0])
            dy_dense = np.diff(eval_dense[1])
            total_length = np.sum(np.sqrt(dx_dense**2 + dy_dense**2))
            
            # Calculamos cuántos puntos necesitamos exactamente según el spacing (0.1m)
            num_points = max(int(total_length / spacing), 2)
            u_resampled = np.linspace(0, 1, num_points)
            
            # Obtenemos las nuevas coordenadas suaves (x, y)
            new_x, new_y = splev(u_resampled, tck)
            
            # Derivadas de primer orden (Yaw)
            der1_x, der1_y = splev(u_resampled, tck, der=1)
            # Derivadas de segundo orden (Para curvatura)
            der2_x, der2_y = splev(u_resampled, tck, der=2)
            
            # ==========================================
            # EMPAQUETADO Y PUBLICACIÓN
            # ==========================================
            smoothed_path = Path()
            smoothed_path.header = msg.header
            smoothed_path.header.stamp = self.get_clock().now().to_msg()

            max_curvature_found = 0.0
            
            for i in range(num_points):
                p = PoseStamped()
                p.header = smoothed_path.header
                p.pose.position.x = float(new_x[i])
                p.pose.position.y = float(new_y[i])
                p.pose.position.z = 0.0
                
                # Orientación (Yaw)
                yaw = math.atan2(der1_y[i], der1_x[i])
                p.pose.orientation = self.yaw_to_quaternion(yaw)
                
                smoothed_path.poses.append(p)
                
                # Cálculo de curvatura kappa = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
                denominator = (der1_x[i]**2 + der1_y[i]**2)**1.5
                if denominator > 1e-6:
                    kappa = abs(der1_x[i] * der2_y[i] - der1_y[i] * der2_x[i]) / denominator
                    max_curvature_found = max(max_curvature_found, kappa)

            self.pub.publish(smoothed_path)
            
            # Análisis de viabilidad de la trayectoria generada
            actual_min_radius = 1.0 / max_curvature_found if max_curvature_found > 1e-6 else float('inf')
            
            self.get_logger().info(f"Ruta suavizada: {total_length:.2f}m en {num_points} puntos.")
            
            if actual_min_radius < self.min_turning_radius:
                self.get_logger().warn(
                    f"¡ALERTA CINEMÁTICA! El radio mínimo de la curva generada ({actual_min_radius:.2f}m) "
                    f"es MENOR al límite físico del carro ({self.min_turning_radius:.2f}m). "
                    f"Aumenta el 'smoothing_factor' o el controlador forzará la dirección más allá del límite mecánico."
                )
            else:
                self.get_logger().info(
                    f"Trayectoria viable. Radio mínimo generado: {actual_min_radius:.2f}m "
                    f"(Límite físico: {self.min_turning_radius:.2f}m)."
                )
            
        except Exception as e:
            self.get_logger().error(f"Fallo en cálculo de Splines: {e}")
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()