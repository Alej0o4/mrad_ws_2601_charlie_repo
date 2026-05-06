#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped # Usaremos esto temporalmente para mandar el Yaw al FSM
import numpy as np
import tf2_ros
from tf2_ros import TransformException

# Necesitarás instalar o incluir la librería MadgwickAHRS que menciona tu profe
from ahrs.filters import Madgwick # (pip install ahrs)

def _quat_to_rot(x, y, z, w) -> np.ndarray:
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-w*z),    2*(x*z+w*y)  ],
        [2*(x*y+w*z),    1-2*(x*x+z*z),  2*(y*z-w*x)  ],
        [2*(x*z-w*y),    2*(y*z+w*x),    1-2*(x*x+y*y)],
    ])

class ImuProcessorNode(Node):
    def __init__(self):
        super().__init__('imu_processor')
        
        # TF Listener para leer el URDF automáticamente
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._R_imu_to_base = None

        # Filtro Madgwick
        self.ahrs = Madgwick(sample_period=0.02, beta=0.1)
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) # w, x, y, z

        # Suscriptor al IMU crudo de Yahboom
        self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        
        # Publicador de la IMU corregida (Para el EKF futuro)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_corrected', 10)

    def _get_imu_rotation(self):
        if self._R_imu_to_base is not None:
            return self._R_imu_to_base
        try:
            tf_msg = self._tf_buffer.lookup_transform('base_link', 'imu_link', rclpy.time.Time())
            q = tf_msg.transform.rotation
            self._R_imu_to_base = _quat_to_rot(q.x, q.y, q.z, q.w)
            self.get_logger().info("Matriz de rotación URDF cargada exitosamente.")
        except TransformException:
            pass
        return self._R_imu_to_base

    def imu_callback(self, msg):
        R = self._get_imu_rotation()
        if R is None:
            return # Esperamos a que el URDF esté listo

        # 1. Extraer datos crudos
        raw_gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        raw_accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        # 2. LA REGLA DE ORO DE YAHBOOM
        gyro_corrected = R @ raw_gyro  # Rotamos el gyro
        accel_for_ahrs = raw_accel     # NUNCA rotamos el accel

        # 3. Pasar por Madgwick
        # Nota: Si tu placa no tiene magnetómetro, pasamos None
        self.q = self.ahrs.updateIMU(self.q, gyr=gyro_corrected, acc=accel_for_ahrs)

        # 4. Publicar la IMU corregida
        out_msg = Imu()
        out_msg.header = msg.header
        out_msg.header.frame_id = 'base_link' # Ahora los datos viven en el marco del robot
        
        out_msg.angular_velocity.x = gyro_corrected[0]
        out_msg.angular_velocity.y = gyro_corrected[1]
        out_msg.angular_velocity.z = gyro_corrected[2]
        
        out_msg.linear_acceleration.x = accel_for_ahrs[0]
        out_msg.linear_acceleration.y = accel_for_ahrs[1]
        out_msg.linear_acceleration.z = accel_for_ahrs[2]
        
        out_msg.orientation.w = self.q[0]
        out_msg.orientation.x = self.q[1]
        out_msg.orientation.y = self.q[2]
        out_msg.orientation.z = self.q[3]

        self.imu_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()