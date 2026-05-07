#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import threading
from collections import deque

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, TwistStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool, String,Int16
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
        self.declare_parameter('rpm_topic', '/esc/speed_rpm')
        self.declare_parameter('cfoc_state_topic', '/esc/cfoc_state')  # ← NUEVO
        self.declare_parameter('steering_topic', '/servo/steering_cmd')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_valid_topic', '/odom/valid')    # ← NUEVO
        self.declare_parameter('twist_topic', '/odom/twist')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        
        # Parámetros Mecánicos
        self.declare_parameter('wheelbase', 0.257) 
        self.declare_parameter('wheel_radius', 0.053)
        self.declare_parameter('gear_ratio', 9.6)
        self.declare_parameter('rpm_deadband', 50.0)
        
        # ← NUEVOS: Filtrado y validación
        self.declare_parameter('rpm_lpf_alpha', 0.1)      # LPF para RPM ruidoso
        self.declare_parameter('rpm_timeout_s', 0.2)      # timeout si no llegan RPM
        self.declare_parameter('require_closed_loop', True) # solo publica en CL
        self.declare_parameter('publish_tf', True)        # habilita/deshabilita la publicación de TF
        self.declare_parameter('publish_twist', True)     # habilita/deshabilita la publicación de TwistStamped
        self.declare_parameter('pose_cov_multiplier', 1.0) # escala dinámica de covarianza
        self.declare_parameter('twist_cov_multiplier', 1.0)

        # --- Obtención de Parámetros ---
        self.rpm_topic = self.get_parameter('rpm_topic').value
        self.cfoc_state_topic = self.get_parameter('cfoc_state_topic').value
        self.steering_topic = self.get_parameter('steering_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_valid_topic = self.get_parameter('odom_valid_topic').value
        self.twist_topic = self.get_parameter('twist_topic').value
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
            Int16, self.rpm_topic, self._rpm_callback, sensor_qos)  # ← Cambio a Int16 (motor ESC publica así)
        
        self.cfoc_state_sub = self.create_subscription(  # ← NUEVO
            String, self.cfoc_state_topic, self._cfoc_state_callback, sensor_qos)

        # --- Publicadores ---
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.odom_valid_publisher = self.create_publisher(Bool, self.odom_valid_topic, 10)  # ← NUEVO
        self.twist_publisher = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Timers ---
        self.timer = self.create_timer(1.0 / 50.0, self._publish_loop)

        # --- Estado Interno ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.current_steering_angle = 0.0
        self.wheel_rotation_angle = 0.0
        
        # ← NUEVOS: Tracking de velocidades y estado
        self.current_rpm_raw = 0.0
        self.current_rpm_filtered = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        self.last_time = self.get_clock().now()
        self.last_rpm_time = self.get_clock().now()
        
        self.cfoc_state = 'IDLE'
        self.odom_valid = False  # ← Bandera de validez
        self.last_direction = 0  # +1: FWD, -1: REV, 0: neutral. Detectar cambios
        
        self._lock = threading.Lock()

        self.get_logger().info(
            f"Bicycle Odom Node iniciado.\n"
            f"  Require CL: {self.require_closed_loop}\n"
            f"  RPM LPF: {self.rpm_lpf_alpha}\n"
            f"  Wheelbase: {self.wheelbase:.3f} m\n"
            f"  Wheel radius: {self.wheel_radius:.4f} m\n"
            f"  Publish TF: {self.publish_tf}"
        )

    def _steering_callback(self, msg: Float32):
        """Actualiza el comando de dirección (radianes)."""
        with self._lock:
            self.current_steering_angle = msg.data

    def _cfoc_state_callback(self, msg: String):  # ← NUEVO
        """Monitorea el estado CFOC del ESC."""
        with self._lock:
            self.cfoc_state = msg.data
            # Transición a CL: valida odom
            if msg.data == 'CLOSED_LOOP' and self.cfoc_state != 'CLOSED_LOOP':
                self.get_logger().info('→ CFOC entered CLOSED_LOOP. Odom validity ON.')
            # Salida de CL: invalida odom
            elif msg.data != 'CLOSED_LOOP' and self.cfoc_state == 'CLOSED_LOOP':
                self.get_logger().warn(f'← CFOC left CLOSED_LOOP → {msg.data}. Odom validity OFF.')

    def _rpm_callback(self, msg):
        """
        Callback principal de velocidad del motor.
        - Aplicar LPF
        - Calcular cinemática
        - Integrar posición
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Evitar integraciones con saltos de tiempo absurdos
        if dt > 0.5 or dt < 0.0:
            self.get_logger().warn(f"Salto de tiempo anómalo ({dt:.3f}s). Ignorando.")
            return

        with self._lock:
            self.last_rpm_time = current_time
            raw_rpm = float(msg.data)  # En case de Int16 en el ESC

            # 1. Aplicar Deadband
            if abs(raw_rpm) < self.rpm_deadband:
                raw_rpm = 0.0

            # 2. LPF para suavizar ruido sensorless
            if self.current_rpm_filtered == 0.0:
                self.current_rpm_filtered = raw_rpm
            else:
                self.current_rpm_filtered = (
                    self.rpm_lpf_alpha * raw_rpm +
                    (1.0 - self.rpm_lpf_alpha) * self.current_rpm_filtered
                )
            
            self.current_rpm_raw = raw_rpm

            # 3. Calcular velocidades
            wheel_omega = (self.current_rpm_filtered / self.gear_ratio) * (2.0 * math.pi / 60.0)
            self.current_v = wheel_omega * self.wheel_radius

            delta = self.current_steering_angle
            if abs(self.current_v) > 0.001:  # Evita división por cero en w
                self.current_w = (self.current_v / self.wheelbase) * math.tan(delta)
            else:
                self.current_w = 0.0

            # 4. Detectar cambios de dirección (FWD ↔ REV)
            curr_direction = 1 if self.current_v > 0 else (-1 if self.current_v < 0 else 0)
            if (self.last_direction != 0 and curr_direction != 0 and
                    self.last_direction != curr_direction):
                self.get_logger().warn(
                    f"Direction reversal detected: {self.last_direction:+d} → {curr_direction:+d}. "
                    f"Odometry may accumulate error in reverse.")
                # Opcionalmente: aumentar covarianza temporalmente
            self.last_direction = curr_direction

            # 5. Integración de pose (Modelo de Bicicleta Exacto)
            if abs(self.current_w) > 0.0001:
                # Movimiento curvo
                radius = self.current_v / self.current_w
                dtheta = self.current_w * dt
                self.x += radius * (math.sin(self.theta + dtheta) - math.sin(self.theta))
                self.y += radius * (-math.cos(self.theta + dtheta) + math.cos(self.theta))
                self.theta += dtheta
            else:
                # Movimiento recto
                self.x += self.current_v * math.cos(self.theta) * dt
                self.y += self.current_v * math.sin(self.theta) * dt

            # Normalizar theta
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            # 6. Actualizar rotación de ruedas (visualización)
            self.wheel_rotation_angle += wheel_omega * dt
            self.wheel_rotation_angle = math.fmod(self.wheel_rotation_angle, 2 * math.pi)

    def _publish_loop(self):
        """Loop de publicación @ 50 Hz."""
        current_time = self.get_clock().now()
        with self._lock:
            # 1. Evaluar frescura de RPM
            dt_since_rpm = (current_time - self.last_rpm_time).nanoseconds / 1e9
            is_rpm_fresh = dt_since_rpm <= self.rpm_timeout_s

            if not is_rpm_fresh and dt_since_rpm > self.rpm_timeout_s + 0.1:
                self.get_logger().debug(f"RPM timeout: {dt_since_rpm:.2f}s > {self.rpm_timeout_s}s")

            # 2. Evaluar condición de lazo cerrado
            is_closed_loop = (self.cfoc_state == 'CLOSED_LOOP') or not self.require_closed_loop

            # 3. La odometría es válida SI los datos son frescos Y cumple la condición del ESC
            publish_odom = is_rpm_fresh and is_closed_loop

            # Publicar bandera de validez
            valid_msg = Bool(data=publish_odom)
            self.odom_valid_publisher.publish(valid_msg)

            # Si no es válida, congelamos velocidades pero podemos seguir publicando pose
            # (con covarianzas muy altas)
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
        """Publicar transformación odom → base_link."""
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
        """Publicar mensaje Odometry."""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = quaternion_from_euler(0, 0, self.theta)
        
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

        # ← NUEVO: Covarianzas adaptativas
        if valid:
            # En CLOSED_LOOP: confianza moderada
            pose_cov_base = [
                0.02,  # x
                0.02,  # y
                0.10,  # yaw (sensorless sin encoder es incierto en heading)
            ]
            twist_cov_base = [
                0.05,  # vx
                0.05,  # vy (no usamos, pero OKs)
                0.10,  # w
            ]
            mult = self.pose_cov_mult
        else:
            # Inválida: covarianzas muy altas (desconfia el EKF)
            pose_cov_base = [1.0, 1.0, 1.0]
            twist_cov_base = [1.0, 1.0, 1.0]
            mult = 1.0  # Force alta incertidumbre

        pose_cov = [0.0] * 36

        for i in (0, 7, 14, 21, 28, 35):
            pose_cov[i] = 1e-9

        pose_cov[0]  = pose_cov_base[0] * mult  # x
        pose_cov[7]  = pose_cov_base[1] * mult  # y
        pose_cov[35] = pose_cov_base[2] * mult  # yaw
        odom_msg.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0]  = twist_cov_base[0] * mult  # vx
        twist_cov[35] = twist_cov_base[2] * mult  # w
        odom_msg.twist.covariance = twist_cov

        self.odom_publisher.publish(odom_msg)

    def _publish_twist(self, timestamp, v, w):
        """Publicar la velocidad estimada por el modelo cinemático."""
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
        """Publicar estados de articulaciones para visualización en RViz."""
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