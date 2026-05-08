#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import threading
from collections import deque

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, TwistStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState, Imu  # ← NUEVO: Importamos Imu
from std_msgs.msg import Float32, Bool, String, Int16
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

# ← NUEVO: Función para extraer el Yaw de la IMU
def yaw_from_quaternion(q):
    """Extrae el ángulo Yaw (Z) de un mensaje de cuaternión."""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class BicycleOdomNode(Node):
    def __init__(self):
        super().__init__('bicycle_odom_node')

        # --- Declaración de Parámetros ---
        self.declare_parameter('rpm_topic', '/esc/speed_rpm')
        self.declare_parameter('cfoc_state_topic', '/esc/cfoc_state')
        self.declare_parameter('steering_topic', '/servo/steering_cmd')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_valid_topic', '/odom/valid')
        self.declare_parameter('twist_topic', '/odom/twist')
        self.declare_parameter('imu_topic', '/imu/data_corrected') # ← NUEVO: Tópico de la IMU
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        
        # Parámetros Mecánicos
        self.declare_parameter('wheelbase', 0.257) 
        self.declare_parameter('wheel_radius', 0.053)
        self.declare_parameter('gear_ratio', 9.6)
        self.declare_parameter('rpm_deadband', 50.0)
        
        # Filtrado y validación
        self.declare_parameter('rpm_lpf_alpha', 0.1)      
        self.declare_parameter('rpm_timeout_s', 0.2)      
        self.declare_parameter('require_closed_loop', True) 
        self.declare_parameter('publish_tf', True)        
        self.declare_parameter('publish_twist', True)     
        self.declare_parameter('pose_cov_multiplier', 1.0) 
        self.declare_parameter('twist_cov_multiplier', 1.0)

        # --- Obtención de Parámetros ---
        self.rpm_topic = self.get_parameter('rpm_topic').value
        self.cfoc_state_topic = self.get_parameter('cfoc_state_topic').value
        self.steering_topic = self.get_parameter('steering_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_valid_topic = self.get_parameter('odom_valid_topic').value
        self.twist_topic = self.get_parameter('twist_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value # ← NUEVO
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.rpm_deadband = self.get_parameter('rpm_deadband').value
        
        self.rpm_lpf_alpha = self.get_parameter('rpm_lpf_alpha').value
        self.rpm_timeout_s = self.get_parameter('rpm_timeout_s').value
        self.require_closed_loop = self.get_parameter('require_closed_loop').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_twist = self.get_parameter('publish_twist').value
        self.pose_cov_mult = self.get_parameter('pose_cov_multiplier').value
        self.twist_cov_mult = self.get_parameter('twist_cov_multiplier').value

        # --- Configuración de QoS ---
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # --- Suscripciones ---
        self.steering_sub = self.create_subscription(
            Float32, self.steering_topic, self._steering_callback, sensor_qos)
        
        self.rpm_sub = self.create_subscription(
            Int16, self.rpm_topic, self._rpm_callback, sensor_qos) 
        
        self.cfoc_state_sub = self.create_subscription(  
            String, self.cfoc_state_topic, self._cfoc_state_callback, sensor_qos)

        # ← NUEVO: Suscripción a la IMU
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._imu_callback, 10)

        # --- Publicadores ---
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.odom_valid_publisher = self.create_publisher(Bool, self.odom_valid_topic, 10) 
        self.twist_publisher = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Timers ---
        self.timer = self.create_timer(1.0 / 50.0, self._publish_loop)

        # --- Estado Interno ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Ahora será gobernado por la IMU
        
        self.current_steering_angle = 0.0
        self.wheel_rotation_angle = 0.0
        
        self.current_rpm_raw = 0.0
        self.current_rpm_filtered = 0.0
        self.current_v = 0.0
        self.current_w = 0.0 # Ahora será gobernado por la IMU
        self.last_time = self.get_clock().now()
        self.last_rpm_time = self.get_clock().now()
        
        self.cfoc_state = 'IDLE'
        self.odom_valid = False  
        self.last_direction = 0  
        
        self._lock = threading.Lock()

        self.get_logger().info("Bicycle Odom Node (MODO VERIFICACIÓN IMU) iniciado.")

    # ← NUEVO: Callback de la IMU
    def _imu_callback(self, msg: Imu):
        """Actualiza la orientación y velocidad angular directamente desde la IMU."""
        with self._lock:
            # Reemplazamos la velocidad angular cinemática por la real del giroscopio
            self.current_w = msg.angular_velocity.z
            
            # Reemplazamos el Theta (Yaw) integrado por la orientación absoluta del filtro Madgwick
            self.theta = yaw_from_quaternion(msg.orientation)

    def _steering_callback(self, msg: Float32):
        with self._lock:
            self.current_steering_angle = msg.data

    def _cfoc_state_callback(self, msg: String):  
        with self._lock:
            self.cfoc_state = msg.data
            if msg.data == 'CLOSED_LOOP' and self.cfoc_state != 'CLOSED_LOOP':
                self.get_logger().info('→ CFOC entered CLOSED_LOOP. Odom validity ON.')
            elif msg.data != 'CLOSED_LOOP' and self.cfoc_state == 'CLOSED_LOOP':
                self.get_logger().warn(f'← CFOC left CLOSED_LOOP → {msg.data}. Odom validity OFF.')

    def _rpm_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt > 0.5 or dt < 0.0:
            return

        with self._lock:
            self.last_rpm_time = current_time
            raw_rpm = float(msg.data)

            # 1. Aplicar Deadband
            if abs(raw_rpm) < self.rpm_deadband:
                raw_rpm = 0.0

            # 2. LPF
            if self.current_rpm_filtered == 0.0:
                self.current_rpm_filtered = raw_rpm
            else:
                self.current_rpm_filtered = (
                    self.rpm_lpf_alpha * raw_rpm +
                    (1.0 - self.rpm_lpf_alpha) * self.current_rpm_filtered
                )
            self.current_rpm_raw = raw_rpm

            # 3. Calcular velocidad Lineal
            wheel_omega = (self.current_rpm_filtered / self.gear_ratio) * (2.0 * math.pi / 60.0)
            self.current_v = wheel_omega * self.wheel_radius

            # NOTA: Omitimos el cálculo cinemático de self.current_w porque la IMU ya nos lo da.

            # 4. Detectar cambios de dirección
            curr_direction = 1 if self.current_v > 0 else (-1 if self.current_v < 0 else 0)
            self.last_direction = curr_direction

            # 5. Integración de posición X, Y
            # Usamos el self.theta proporcionado por la IMU para saber hacia dónde avanzamos
            self.x += self.current_v * math.cos(self.theta) * dt
            self.y += self.current_v * math.sin(self.theta) * dt

            # 6. Actualizar rotación de ruedas
            self.wheel_rotation_angle += wheel_omega * dt
            self.wheel_rotation_angle = math.fmod(self.wheel_rotation_angle, 2 * math.pi)

    def _publish_loop(self):
        current_time = self.get_clock().now()
        with self._lock:
            dt_since_rpm = (current_time - self.last_rpm_time).nanoseconds / 1e9
            is_rpm_fresh = dt_since_rpm <= self.rpm_timeout_s
            is_closed_loop = (self.cfoc_state == 'CLOSED_LOOP') or not self.require_closed_loop
            publish_odom = is_rpm_fresh and is_closed_loop

            valid_msg = Bool(data=publish_odom)
            self.odom_valid_publisher.publish(valid_msg)

            if not publish_odom:
                v_pub = 0.0
                w_pub = 0.0
            else:
                v_pub = self.current_v
                w_pub = self.current_w

            if self.publish_tf:
                self._publish_tf(current_time)
            self._publish_odometry(current_time, v_pub, w_pub, publish_odom)
            if self.publish_twist:
                self._publish_twist(current_time, v_pub, w_pub)
            self._publish_joint_states(current_time)

    def _publish_tf(self, timestamp):
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_from_euler(0, 0, self.theta)
        self.tf_broadcaster.sendTransform(t)

    def _publish_odometry(self, timestamp, v, w, valid):
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = quaternion_from_euler(0, 0, self.theta)
        
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

        # Covarianzas (puedes dejarlas igual por ahora)
        if valid:
            pose_cov_base = [0.02, 0.02, 0.10]
            twist_cov_base = [0.05, 0.05, 0.10]
            mult = self.pose_cov_mult
        else:
            pose_cov_base = [1.0, 1.0, 1.0]
            twist_cov_base = [1.0, 1.0, 1.0]
            mult = 10.0

        pose_cov = [0.0] * 36
        pose_cov[0]  = pose_cov_base[0] * mult  
        pose_cov[7]  = pose_cov_base[1] * mult  
        pose_cov[35] = pose_cov_base[2] * mult  
        odom_msg.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0]  = twist_cov_base[0] * mult  
        twist_cov[35] = twist_cov_base[2] * mult  
        odom_msg.twist.covariance = twist_cov

        self.odom_publisher.publish(odom_msg)

    def _publish_twist(self, timestamp, v, w):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = timestamp.to_msg()
        twist_msg.header.frame_id = self.base_frame
        twist_msg.twist.linear.x = v
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = w
        self.twist_publisher.publish(twist_msg)

    def _publish_joint_states(self, timestamp):
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
            self.current_steering_angle,
            self.current_steering_angle,
            self.wheel_rotation_angle,
            self.wheel_rotation_angle,
            self.wheel_rotation_angle,
            self.wheel_rotation_angle
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