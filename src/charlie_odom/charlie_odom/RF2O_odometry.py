#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
import transforms3d

class RF2OOdometryNode(Node):
    def __init__(self):
        super().__init__('rf2o_odometry_node')
        
        # Suscriptores y Publicadores
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_rf2o', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Variables de estado
        self.prev_ranges = None
        self.last_time = None
        
        # Pose global acumulada
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.get_logger().info("🚀 Nodo RF2O (Python) inicializado basado en Jaimez 2016.")

    def scan_callback(self, msg):
        current_time = self.get_clock().now()
        ranges = np.array(msg.ranges)
        
        # 1. Filtrar lecturas inválidas (infinitos o fuera de rango)
        valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max)
        
        if self.prev_ranges is None:
            self.prev_ranges = ranges
            self.prev_valid_mask = valid_mask
            self.last_time = current_time
            return

        # Calcular Delta de tiempo (dt)
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        # 2. Encontrar puntos válidos en AMBOS escaneos (instante t y t-1)
        mask = valid_mask & self.prev_valid_mask
        
        if np.sum(mask) < 50: # Si hay muy pocos puntos, ignorar
            self.get_logger().warn("Puntos insuficientes para calcular RF2O")
            return

        # Aplicar máscara
        R_t = ranges[mask]
        R_t1 = self.prev_ranges[mask]
        
        # 3. Calcular ángulos de los rayos válidos
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        alpha = angles[mask]

        # 4. Derivada Temporal (Cambio de rango respecto al tiempo) -> El vector 'k'
        dR_dt = (R_t - R_t1) / dt
        
        # 5. Derivada Espacial (Gradiente del escaneo respecto al ángulo)
        # Usamos np.gradient para aproximar la pendiente del entorno
        gradient_R = np.gradient(self.prev_ranges, msg.angle_increment)
        dR_dalpha = gradient_R[mask]

        # 6. Construir la Matriz Jacobiana H (Basado en la Range Flow Constraint)
        # Para un robot en 2D buscando [vx, vy, w]
        H = np.zeros((len(R_t), 3))
        H[:, 0] = -np.cos(alpha)              # Efecto de la velocidad lineal en X
        H[:, 1] = -np.sin(alpha)              # Efecto de la velocidad lineal en Y
        H[:, 2] = dR_dalpha / R_t1            # Efecto de la velocidad angular W
        
        # 7. Resolver el sistema H * xi = dR_dt mediante mínimos cuadrados (Least Squares)
        # Esto minimiza el error de alineación (Scan Matching denso)
        try:
            xi, residuals, rank, s = np.linalg.lstsq(H, dR_dt, rcond=None)
            vx, vy, w = xi
        except np.linalg.LinAlgError:
            self.get_logger().error("Error matemático al resolver mínimos cuadrados.")
            return

        # 8. Integrar las velocidades para obtener la Odometría global
        # Rotamos las velocidades locales al marco de referencia global
        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.theta += w * dt

        # 9. Publicar Odometría y TF
        self.publish_odometry(current_time.to_msg(), vx, vy, w)
        
        # Actualizar memoria para la siguiente iteración
        self.prev_ranges = ranges
        self.prev_valid_mask = valid_mask
        self.last_time = current_time

    def publish_odometry(self, stamp, vx, vy, w):
        # Crear cuaternión a partir del ángulo theta
        q = transforms3d.euler.euler2quat(0, 0, self.theta) # Devuelve (w, x, y, z)
        
        # Publicar TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        self.tf_broadcaster.sendTransform(t)

        # Publicar Mensaje de Odometría
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = RF2OOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()