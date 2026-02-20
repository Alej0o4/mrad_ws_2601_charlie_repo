import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import csv
import os
from datetime import datetime
import math

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')

        # --- PARÁMETROS ---
        # Define el nombre de la prueba (ej: "wall_follower" o "gap_follower")
        self.declare_parameter('test_name', 'default_test')
        self.test_name = self.get_parameter('test_name').get_parameter_value().string_value

        # --- SUSCRIPTORES ---
        # Asegúrate de que este sea tu topic de odometría correcto (a veces es /diff_cont/odom)
        self.subscription = self.create_subscription(
            Odometry,
            '/diffdrive_controller/odom', 
            self.odom_callback,
            10)

        # --- PUBLICADORES (Para RViz) ---
        self.path_publisher = self.create_publisher(Path, '/charlie_path', 10)
        
        # --- VARIABLES ---
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'  # El marco de referencia fijo
        
        # Configuración de archivo CSV
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"charlie_data_{self.test_name}_{timestamp}.csv"
        
        self.csv_file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Encabezados del Excel
        self.csv_writer.writerow(['time_sec', 'x', 'y', 'theta_rad', 'linear_vel', 'angular_vel'])
        
        self.get_logger().info(f'Grabando datos en: {self.filename}')

    def odom_callback(self, msg):
        # 1. VISUALIZACIÓN EN RVIZ
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(pose)
        
        # Publicar para ver la línea en RViz
        self.path_publisher.publish(self.path_msg)

        # 2. GUARDADO DE DATOS
        # Extraer posición
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extraer orientación (Yaw) de cuaterniones
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # Velocidades
        v_lin = msg.twist.twist.linear.x
        v_ang = msg.twist.twist.angular.z
        
        # Tiempo
        time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Escribir fila
        self.csv_writer.writerow([time_sec, x, y, theta, v_lin, v_ang])

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info('Archivo CSV cerrado correctamente.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()