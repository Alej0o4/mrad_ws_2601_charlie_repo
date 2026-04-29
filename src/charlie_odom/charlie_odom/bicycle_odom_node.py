#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def quaternion_from_euler(roll, pitch, yaw):
    """Convierte ángulos de Euler a Cuaternión."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

class BicycleOdomNode(Node):
    def __init__(self):
        super().__init__('bicycle_odom_node')

        # --- Declaración de Parámetros ---
        self.declare_parameter('rpm_topic', '/esc/rpm')
        self.declare_parameter('steering_topic', '/servo/steering_cmd')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        
        # Parámetros Mecánicos
        self.declare_parameter('wheelbase', 0.257) 
        self.declare_parameter('wheel_radius', 0.053)
        self.declare_parameter('gear_ratio', 3.5) # Relación motor a rueda (Ajustar al carro real)
        self.declare_parameter('rpm_deadband', 50.0) # RPM mínimas para considerar movimiento

        # --- Obtención de Parámetros ---
        self.rpm_topic = self.get_parameter('rpm_topic').value
        self.steering_topic = self.get_parameter('steering_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.rpm_deadband = self.get_parameter('rpm_deadband').value

        # --- Configuración de QoS (Importante para datos de sensores en tiempo real) ---
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # --- Suscripciones ---
        # Guardamos el ángulo del servo asíncronamente
        self.steering_sub = self.create_subscription(
            Float32, self.steering_topic, self.steering_callback,sensor_qos)
            
        # El callback de RPM es el "reloj" de nuestro sistema
        self.rpm_sub = self.create_subscription(
            Float32, self.rpm_topic, self.rpm_callback, sensor_qos)
        # --- Publicadores ---
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Timers ---
        self.timer = self.create_timer(1.0 / 50.0, self.publish_loop)
        self.last_rpm_time = self.get_clock().now()

        # --- Variables de Estado Interno ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Inputs actuales
        self.current_steering_angle = 0.0
        self.wheel_rotation_angle = 0.0 # Acumulador para visualización en RViz
        
        self.last_time = self.get_clock().now()

        self.get_logger().info("Bicycle Kinematic Odom Node iniciado.")

    def steering_callback(self, msg: Float32):
        """Actualiza el estado interno del comando de dirección (en radianes)."""
        self.current_steering_angle = msg.data

    def rpm_callback(self, msg: Float32):
        """
        Callback principal. Calcula la cinemática e integra la posición cada vez
        que llega un nuevo dato de velocidad desde el ESC.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Evitar integraciones con saltos de tiempo absurdos si el ESC se desconecta
        if dt > 0.5:
            self.get_logger().warn(f"Salto de tiempo muy grande ({dt:.2f}s). Ignorando integración.")
            return

        raw_rpm = msg.data

        # 1. Aplicar Deadband para FOC Lazo Abierto
        if abs(raw_rpm) < self.rpm_deadband:
            raw_rpm = 0.0

        # 2. Calcular Velocidad Lineal (v) a partir de RPM
        # RPM motor -> Rad/s rueda motor -> Vel Lineal
        wheel_omega = (raw_rpm / self.gear_ratio) * (2.0 * math.pi / 60.0)
        v = wheel_omega * self.wheel_radius

        # 3. Calcular Velocidad Angular (w) usando el Modelo de Bicicleta
        # w = (v / L) * tan(delta)
        delta = self.current_steering_angle
        w = (v / self.wheelbase) * math.tan(delta)

        # 4. Integración Exacta (Arco de circunferencia)
        if abs(w) > 0.0001:
            # Movimiento curvo
            self.x += (v / w) * (math.sin(self.theta + w * dt) - math.sin(self.theta))
            self.y += (v / w) * (-math.cos(self.theta + w * dt) + math.cos(self.theta))
            self.theta += w * dt
        else:
            # Movimiento recto (evita división por cero)
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
            # theta se mantiene igual

        # Normalizar theta entre -pi y pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # 6. Actualizar joints (ruedas girando)
        self.wheel_rotation_angle += wheel_omega * dt
        self.wheel_rotation_angle = math.fmod(self.wheel_rotation_angle, 2 * math.pi)

    
    def publish_loop(self):
        current_time = self.get_clock().now()
        
        # Timeout de seguridad: Si hace más de 0.2s no llegan RPM, asumimos velocidad cero
        dt_since_last_msg = (current_time - self.last_rpm_time).nanoseconds / 1e9
        if dt_since_last_msg > 0.2:
            self.current_v = 0.0
            self.current_w = 0.0

        self.publish_tf(current_time)
        self.publish_odometry(current_time, self.current_v, self.current_w)
        self.publish_joint_states(current_time)

    def publish_tf(self, timestamp):
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_from_euler(0, 0, self.theta)
        self.tf_broadcaster.sendTransform(t)

    def publish_odometry(self, timestamp, v, w):
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = quaternion_from_euler(0, 0, self.theta)
        
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

        # Matrices de Covarianza (Fundamentales para EKF)
        # Formato: Fila-Mayor, orden [x, y, z, roll, pitch, yaw]
        # Diagonal: Varianza. Valores fuera de diagonal: Covarianza.
        pose_cov = [0.0] * 36
        pose_cov[0]  = 0.01  # x
        pose_cov[7]  = 0.01  # y
        pose_cov[35] = 0.05  # yaw
        odom_msg.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0]  = 0.05  # vx
        twist_cov[35] = 0.05  # w (yaw rate)
        odom_msg.twist.covariance = twist_cov

        self.odom_publisher.publish(odom_msg)

    def publish_joint_states(self, timestamp):
        joint_msg = JointState()
        joint_msg.header.stamp = timestamp.to_msg()
        joint_msg.name = [
            'right_steering_joint', 
            'left_steering_joint',
            'rightf_wheel_joint', 
            'leftf_wheel_joint',
            'rightb_wheel_joint', 
            'leftb_wheel_joint'
        ]
        joint_msg.position = [
            self.current_steering_angle,   # right_steering_joint
            self.current_steering_angle,   # left_steering_joint
            self.wheel_rotation_angle,     # rightf_wheel_joint
            self.wheel_rotation_angle,     # leftf_wheel_joint
            self.wheel_rotation_angle,     # rightb_wheel_joint
            self.wheel_rotation_angle      # leftb_wheel_joint
        ]
        self.joint_publisher.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BicycleOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()