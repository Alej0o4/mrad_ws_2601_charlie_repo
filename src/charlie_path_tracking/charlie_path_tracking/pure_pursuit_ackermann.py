#!/usr/bin/env python3
import math
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path

import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose_stamped

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def yaw_from_quaternion(q) -> float:
    """Conversión rápida de cuaternión a Yaw."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        # ---- Parámetros de Tópicos
        self.declare_parameter("path_topic", "/current_active_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("base_frame", "base_link")

        # ---- Parámetros de Control
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("v_nominal", 1.5)
        self.declare_parameter("max_speed", 2.5)
        self.declare_parameter("max_omega", 2.0)           # Límite de velocidad angular [rad/s]
        self.declare_parameter("goal_tolerance", 0.3)

        # ---- Parámetros de Lookahead
        self.declare_parameter("lookahead_L0", 0.6)
        self.declare_parameter("lookahead_kv", 0.1)
        self.declare_parameter("lookahead_min", 0.4)
        self.declare_parameter("lookahead_max", 2.0)

        self.declare_parameter("eps", 1e-6)
        self.declare_parameter("tf_timeout_sec", 0.1)

        # ---- Inicialización de Variables
        self.path_topic = self.get_parameter("path_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.rate_hz = self.get_parameter("control_rate_hz").value
        self.max_omega = self.get_parameter("max_omega").value
        
        # Lookahead y Tolerancias
        self.L0 = self.get_parameter("lookahead_L0").value
        self.kv = self.get_parameter("lookahead_kv").value
        self.Lmin = self.get_parameter("lookahead_min").value
        self.Lmax = self.get_parameter("lookahead_max").value
        self.goal_tol = self.get_parameter("goal_tolerance").value
        self.eps = self.get_parameter("eps").value
        self.tf_timeout = self.get_parameter("tf_timeout_sec").value

        # ---- ROS 2 Interfaces
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Estado del Nodo
        self.path_array: Optional[np.ndarray] = None
        self.path_frame: Optional[str] = None
        self.has_path = False
        self.last_target_index = 0

        # ---- Timer de Control
        self.timer = self.create_timer(1.0/self.rate_hz, self.on_timer)
        self.get_logger().info(f"Nodo Pure Pursuit (Angular Velocity) iniciado en {self.cmd_topic}")

    def on_path(self, msg: Path) -> None:
        self.path_frame = msg.header.frame_id if msg.header.frame_id else None
        if len(msg.poses) > 0 and self.path_frame is not None:
            # Caché de la ruta en NumPy para eficiencia
            self.path_array = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            self.has_path = True
            self.last_target_index = 0
        else:
            self.has_path = False

    def on_timer(self) -> None:
        if not self.has_path or self.path_array is None:
            self.publish_stop()
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.path_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout)
            )
        except TransformException:
            return

        # Pose del robot
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        ryaw = yaw_from_quaternion(tf.transform.rotation)

        # 1. Verificar Meta
        if np.hypot(self.path_array[-1, 0] - rx, self.path_array[-1, 1] - ry) <= self.goal_tol:
            self.publish_stop()
            self.has_path = False
            return

        # 2. Lookahead Dinámico
        v_cmd = clamp(self.get_parameter("v_nominal").value, 0.0, self.get_parameter("max_speed").value)
        Ld = clamp(self.L0 + self.kv * v_cmd, self.Lmin, self.Lmax)

        # 3. Procesamiento Vectorizado con NumPy
        n = len(self.path_array)
        start = self.last_target_index
        end = min(start + 200, n)
        segment = self.path_array[start:end]

        dx = segment[:, 0] - rx
        dy = segment[:, 1] - ry
        
        # Transformación local (frame base_link)
        cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
        bx = dx * cos_y + dy * sin_y
        by = -dx * sin_y + dy * cos_y

        front_mask = bx > 0.0
        if not np.any(front_mask):
            self.publish_stop()
            return

        v_bx, v_by = bx[front_mask], by[front_mask]
        dist = np.hypot(v_bx, v_by)
        crossed = np.where(dist >= Ld)[0]

        if len(crossed) > 0:
            target_by = v_by[crossed[0]]
            self.last_target_index = start + np.where(front_mask)[0][crossed[0]]
        else:
            target_by = v_by[-1]
            self.last_target_index = start + np.where(front_mask)[0][-1]

        # 4. Cálculo de Velocidad Angular (Omega)
        # Curvatura kappa = 2*y / Ld^2
        kappa = (2.0 * target_by) / (Ld * Ld + self.eps)
        omega = clamp(v_cmd * kappa, -self.max_omega, self.max_omega)

        self.publish_cmd(v_cmd, omega)

    def publish_cmd(self, v: float, w: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w) # Publicamos omega directamente
        self.cmd_pub.publish(msg)

    def publish_stop(self) -> None:
        self.publish_cmd(0.0, 0.0)

def main():
    rclpy.init()
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()