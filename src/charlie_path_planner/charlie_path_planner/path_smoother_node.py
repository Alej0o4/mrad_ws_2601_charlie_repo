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
            
            # ¡EL TRUCO SECRETO!: Derivadas de primer orden para obtener el Yaw perfecto
            der_x, der_y = splev(u_resampled, tck, der=1)
            
            # ==========================================
            # EMPAQUETADO Y PUBLICACIÓN
            # ==========================================
            smoothed_path = Path()
            smoothed_path.header = msg.header
            smoothed_path.header.stamp = self.get_clock().now().to_msg()
            
            for i in range(num_points):
                p = PoseStamped()
                p.header = smoothed_path.header
                p.pose.position.x = float(new_x[i])
                p.pose.position.y = float(new_y[i])
                p.pose.position.z = 0.0
                
                # Orientación basada en la tangente de la curva
                yaw = math.atan2(der_y[i], der_x[i])
                p.pose.orientation = self.yaw_to_quaternion(yaw)
                
                smoothed_path.poses.append(p)
                
            self.pub.publish(smoothed_path)
            self.get_logger().info(f"Ruta suavizada: {total_length:.2f}m divididos en {num_points} puntos.")
            
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