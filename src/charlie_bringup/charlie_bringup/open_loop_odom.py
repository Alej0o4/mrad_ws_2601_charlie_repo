#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

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

class OpenLoopOdomNode(Node):
    def __init__(self):
        super().__init__('open_loop_odom_node')

        # --- Declaración de Parámetros ---
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_aebs')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('wheelbase', 0.257) 
        
        # Look-Up Table: Velocidades Empíricas (Verdad Absoluta, m/s y rad/s)
        self.declare_parameter('v_slow', 0.412) 
        self.declare_parameter('v_fast', 1.255) 
        self.declare_parameter('w_turn', 1.5)   

        # Umbrales de detección para el PWM de entrada
        self.declare_parameter('pwm_fast_threshold', 0.045) # Mayor a 4.5% se asume FAST
        self.declare_parameter('pwm_deadzone', 0.01)        # Menor a 1% se asume IDLE
        self.declare_parameter('pwm_turn_threshold', 0.02)  # Umbral de detección de giro
        self.declare_parameter('w_scale', 1.0) # Ajustar según pruebas

        # --- Obtención de Parámetros ---
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.wheelbase = self.get_parameter('wheelbase').value
        
        self.v_slow = self.get_parameter('v_slow').value
        self.v_fast = self.get_parameter('v_fast').value
        self.w_turn = self.get_parameter('w_turn').value
        
        
        self.pwm_fast_threshold = self.get_parameter('pwm_fast_threshold').value
        self.pwm_deadzone = self.get_parameter('pwm_deadzone').value
        self.pwm_turn_threshold = self.get_parameter('pwm_turn_threshold').value
        self.w_scale = self.get_parameter('w_scale').value

        # --- Configuración de ROS2 ---
        # Ahora nos suscribimos a TwistStamped
        self.subscription = self.create_subscription(
            TwistStamped, self.cmd_vel_topic, self.cmd_callback, 10)
        
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        timer_period = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(timer_period, self.integration_loop)

        # --- Variables de Estado Interno ---
        self.current_state = "IDLE"
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.get_logger().info("Nodo de Odometría (Clasificador de PWM) iniciado.")

    def cmd_callback(self, msg: TwistStamped):
        """
        Recibe el TwistStamped. 
        Clasifica la tracción (salto brusco) pero mantiene la dirección continua.
        """
        cmd_v = msg.twist.linear.x
        cmd_w = msg.twist.angular.z

        # 1. TRACCIÓN (Discreta por salto brusco)
        self.v_real = 0.0
        if cmd_v >= self.pwm_fast_threshold:
            self.v_real = self.v_fast
        elif cmd_v > self.pwm_deadzone:
            self.v_real = self.v_slow
        elif cmd_v <= -self.pwm_fast_threshold:
            self.v_real = -self.v_fast
        elif cmd_v < -self.pwm_deadzone:
            self.v_real = -self.v_slow

        # 2. DIRECCIÓN (Continua y proporcional)
        # En un Ackermann puro, si no hay avance lineal (v = 0), 
        # no hay rotación sobre el eje Z (w = 0).
        if abs(self.v_real) < 0.001:
            self.w_real = 0.0
        else:
            # Aquí aplicamos una relación lineal simple (Proporcional).
            # Multiplicamos el comando crudo por un factor de escala paramétrico.
            # Ejemplo: Si cmd_w es 1.0 (PWM máximo de giro), y w_scale es 1.6 rad/s, 
            # w_real será 1.6 rad/s. Si cmd_w es 0.5, w_real será 0.8 rad/s.
            self.w_real = cmd_w * self.w_scale

    def get_velocities_from_state(self):
        """Retorna las velocidades calculadas por el modelo híbrido."""
        # Ya no usamos LUT para la dirección, solo devolvemos los valores procesados
        return getattr(self, 'v_real', 0.0), getattr(self, 'w_real', 0.0)

    def integration_loop(self):
        """Bucle principal que integra la posición de forma continua."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 1. Obtener velocidades reales desde la LUT
        v, w = self.get_velocities_from_state()

        # 2. Cinemática (Bicycle Model)
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = w * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # 3. Publicar Transform (TF)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_from_euler(0, 0, self.theta)
        self.tf_broadcaster.sendTransform(t)

        # 4. Publicar Mensaje de Odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = t.transform.rotation
        
        # Agregar las velocidades empíricas reales que estamos inyectando
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()