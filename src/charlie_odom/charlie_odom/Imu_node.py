#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
import numpy as np
import tf2_ros
from tf2_ros import TransformException
from charlie_odom.madgwic_filter import MadgwickAHRS

def _quat_to_rot(x, y, z, w) -> np.ndarray:
    """Convierte cuaternión (x,y,z,w) a matriz de rotación 3x3"""
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-w*z),    2*(x*z+w*y)  ],
        [2*(x*y+w*z),    1-2*(x*x+z*z),  2*(y*z-w*x)  ],
        [2*(x*z-w*y),    2*(y*z+w*x),    1-2*(x*x+y*y)],
    ])

class ImuProcessorNode(Node):
    def __init__(self):
        super().__init__('imu_processor')
        
        # --- PARÁMETROS ---
        self.declare_parameter('imu_raw_topic', '/imu/data_raw')
        self.declare_parameter('mag_raw_topic', '/imu/mag')
        self.declare_parameter('imu_output_topic', '/imu/data_corrected')
        self.declare_parameter('imu_freq_hz', 10.0)  # Coincide con Yahboom
        self.declare_parameter('madgwick_beta', 0.04)  # Tuning conservador
        self.declare_parameter('mag_declination_rad', 0.0)  # Angulo local (opcional)
        self.declare_parameter('publish_imu_raw_tf', False)  # Debug: publicar frame imu_link
        
        # Obtener parámetros
        imu_raw_topic = self.get_parameter('imu_raw_topic').value
        mag_raw_topic = self.get_parameter('mag_raw_topic').value
        imu_output_topic = self.get_parameter('imu_output_topic').value
        imu_freq = float(self.get_parameter('imu_freq_hz').value)
        beta = float(self.get_parameter('madgwick_beta').value)
        self.mag_declination = float(self.get_parameter('mag_declination_rad').value)
        self.publish_imu_raw_tf = bool(self.get_parameter('publish_imu_raw_tf').value)
        
        self.sample_period = 1.0 / max(imu_freq, 1.0)
        
        # TF Listener (para corregir orientación si IMU está rotada respecto base_link)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._R_imu_to_base = None
        
        # Filtro Madgwick con magnetómetro
        self.ahrs = MadgwickAHRS(sample_period=self.sample_period, beta=beta)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        
        # Almacenar último mensaje de magnetómetro
        self._last_mag = np.array([0.0, 0.0, 0.0])
        self._mag_received = False
        
        # QoS sensor (Best Effort, depth=5)
        sensor_qos = qos_profile_sensor_data
        
        # --- SUSCRIPCIONES ---
        self.imu_raw_sub = self.create_subscription(
            Imu, imu_raw_topic, self._imu_callback, sensor_qos
        )
        self.mag_raw_sub = self.create_subscription(
            MagneticField, mag_raw_topic, self._mag_callback, sensor_qos
        )
        
        # --- PUBLICADORES ---
        self.imu_corrected_pub = self.create_publisher(
            Imu, imu_output_topic, sensor_qos
        )
        
        self.get_logger().info(
            f"IMU Processor iniciado\n"
            f"  Input:  {imu_raw_topic} @ {imu_freq:.1f} Hz\n"
            f"  Mag:    {mag_raw_topic}\n"
            f"  Output: {imu_output_topic}\n"
            f"  Filter: Madgwick (β={beta:.4f})"
        )

    def _get_imu_rotation(self):
        """Obtiene matriz de rotación imu_link -> base_link del URDF"""
        if self._R_imu_to_base is not None:
            return self._R_imu_to_base
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                'base_link', 'imu_link', rclpy.time.Time()
            )
            q = tf_msg.transform.rotation
            self._R_imu_to_base = _quat_to_rot(q.x, q.y, q.z, q.w)
            self.get_logger().info("Transformación URDF (base_link <- imu_link) cargada")
        except TransformException as e:
            self.get_logger().debug(f"TF lookup aún no disponible: {e}")
        return self._R_imu_to_base

    def _mag_callback(self, msg: MagneticField):
        """Recibe magnetómetro crudo de Yahboom"""
        self._last_mag = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        ])
        self._mag_received = True

    def _imu_callback(self, msg: Imu):
        """Procesa IMU cruda con filtro Madgwick + magnetómetro"""
        
        # --- EXTRACCIÓN DE DATOS CRUDOS EN imu_link ---
        raw_gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            -msg.angular_velocity.z
        ])
        raw_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        # --- CORRECCIÓN: SOLO GIROSCOPIO ROTADO (opcional si IMU está desalineada) ---
        R = self._get_imu_rotation()
        if R is not None:
            # Si el IMU está rotado respecto base_link, rotar giroscopio
            gyro_corrected = R @raw_gyro
        else:
            # Si no hay transformación, asumir que IMU está bien alineada
            gyro_corrected = raw_gyro
        
        # Acelerómetro sin rotar (como en Yahboom)
        accel_for_filter = raw_accel
        
        # --- ACTUALIZAR FILTRO MADGWICK CON MAGNETÓMETRO ---
        # El magnetómetro ayuda a corregir drift de yaw del giroscopio
        mag_for_filter = self._last_mag if self._mag_received else None
        
        self.q = self.ahrs.update(
            self.q,
            gyr=gyro_corrected,
            acc=accel_for_filter,
            mag=mag_for_filter  #  Con magnetómetro
        )
        
        # --- PUBLICAR IMU CORREGIDA EN imu_link ---
        out_msg = Imu()
        out_msg.header = msg.header  # Preservar timestamp y frame_id
        out_msg.header.frame_id = 'base_link'  # Mantener en frame original
        
        # Giroscopio corregido
        out_msg.angular_velocity.x = gyro_corrected[0]
        out_msg.angular_velocity.y = gyro_corrected[1]
        out_msg.angular_velocity.z = gyro_corrected[2]
        
        # Acelerómetro (sin cambios)
        out_msg.linear_acceleration.x = accel_for_filter[0]
        out_msg.linear_acceleration.y = accel_for_filter[1]
        out_msg.linear_acceleration.z = accel_for_filter[2]
        
        # Orientación corregida (Madgwick output)
        out_msg.orientation.w = self.q[0]
        out_msg.orientation.x = self.q[1]
        out_msg.orientation.y = self.q[2]
        out_msg.orientation.z = self.q[3]
        
        # --- COVARIANCES (para el EKF) ---
        # Orientación: confianza media en la estimación
        out_msg.orientation_covariance[0] = 0.01   # q_x
        out_msg.orientation_covariance[4] = 0.01   # q_y
        out_msg.orientation_covariance[8] = 0.005  # q_z (más confiable con mag)
        
        # Velocidad angular: confianza alta (giroscopio es preciso)
        out_msg.angular_velocity_covariance[0] = 0.0001  # gx
        out_msg.angular_velocity_covariance[4] = 0.0001  # gy
        out_msg.angular_velocity_covariance[8] = 0.0001  # gz
        
        # Aceleración: confianza media (bajo ruido, pero afectada por movimiento)
        out_msg.linear_acceleration_covariance[0] = 0.001  # ax
        out_msg.linear_acceleration_covariance[4] = 0.001  # ay
        out_msg.linear_acceleration_covariance[8] = 0.001  # az
        
        self.imu_corrected_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()