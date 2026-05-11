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
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("control_rate_hz", 10.0)

        # ---- Parámetros de Velocidad (Porcentajes / Esfuerzo)
        self.declare_parameter("v_nominal_pct", 0.15) # Porcentaje de crucero
        self.declare_parameter("max_speed_pct", 0.20)
        self.declare_parameter("min_speed_pct", 0.05) # Velocidad % mínima al frenar
        
        # ---- Conversión Cinemática (CRÍTICO)
        # Relación: (Velocidad real m/s) / (Porcentaje v_cmd) -> ej: 1.35 / 0.15 = 9.0
        self.declare_parameter("effort_to_ms_ratio", 9.0) 

        self.declare_parameter("max_omega", 1.8)
        
        # ---- Parámetros de Frenado y Tolerancia
        self.declare_parameter("goal_tolerance", 0.1)
        self.declare_parameter("decel_distance", 1.5) # Distancia (m) para empezar a frenar

        # ---- Parámetros de Lookahead
        self.declare_parameter("lookahead_L0", 0.6)
        self.declare_parameter("lookahead_kv", 0.1)
        self.declare_parameter("lookahead_min", 0.4)
        self.declare_parameter("lookahead_max", 2.0)

        self.declare_parameter("eps", 1e-6)
        self.declare_parameter("tf_timeout_sec", 0.1)

        self.path_topic = self.get_parameter("path_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.rate_hz = self.get_parameter("control_rate_hz").value
        self.max_omega = self.get_parameter("max_omega").value
        
        self.L0 = self.get_parameter("lookahead_L0").value
        self.kv = self.get_parameter("lookahead_kv").value
        self.Lmin = self.get_parameter("lookahead_min").value
        self.Lmax = self.get_parameter("lookahead_max").value
        self.goal_tol = self.get_parameter("goal_tolerance").value
        self.eps = self.get_parameter("eps").value
        self.tf_timeout = self.get_parameter("tf_timeout_sec").value

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_array: Optional[np.ndarray] = None
        self.path_frame: Optional[str] = None
        self.has_path = False
        self.last_target_index = 0

        self.timer = self.create_timer(1.0/self.rate_hz, self.on_timer)

    def on_path(self, msg: Path) -> None:
        self.path_frame = msg.header.frame_id if msg.header.frame_id else None
        if len(msg.poses) > 0 and self.path_frame is not None:
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

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        ryaw = yaw_from_quaternion(tf.transform.rotation)

        # 1. Zona de Frenado (Deceleration Profile)
        dist_to_goal = np.hypot(self.path_array[-1, 0] - rx, self.path_array[-1, 1] - ry)

        if dist_to_goal <= self.goal_tol:
            self.publish_stop()
            self.has_path = False
            return

        v_nom_pct = self.get_parameter("v_nominal_pct").value
        min_pct = self.get_parameter("min_speed_pct").value
        decel_dist = self.get_parameter("decel_distance").value

        if dist_to_goal < decel_dist:
            # Desaceleración lineal según nos acercamos a la meta
            v_cmd = min_pct + (v_nom_pct - min_pct) * (dist_to_goal / decel_dist)
        else:
            v_cmd = v_nom_pct

        v_cmd = clamp(v_cmd, min_pct, self.get_parameter("max_speed_pct").value)

        # 2. Lookahead Dinámico
        Ld = clamp(self.L0 + self.kv * v_cmd, self.Lmin, self.Lmax)

        # 3. Ventana de Búsqueda Flexible (Sin límite de 200 puntos)
        start = self.last_target_index
        segment = self.path_array[start:] # Tomamos de la posición actual en adelante

        dx = segment[:, 0] - rx
        dy = segment[:, 1] - ry
        
        cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
        bx = dx * cos_y + dy * sin_y
        by = -dx * sin_y + dy * cos_y

        front_mask = bx > 0.0
        valid_indices = np.where(front_mask)[0]
        
        if len(valid_indices) == 0:
            self.publish_stop()
            return

        v_bx = bx[valid_indices]
        v_by = by[valid_indices]
        dist = np.hypot(v_bx, v_by)
        crossed = np.where(dist >= Ld)[0]

        if len(crossed) > 0:
            target_idx = crossed[0]
            target_by = v_by[target_idx]
            self.last_target_index = start + valid_indices[target_idx]
        else:
            # Si no hay puntos más allá del lookahead, apuntamos al último disponible
            target_by = v_by[-1]
            self.last_target_index = start + valid_indices[-1]

        # 4. Cinemática de Dirección Corregida
        kappa = (2.0 * target_by) / (Ld * Ld + self.eps)
        
        # Convertimos el % de comando a la velocidad física real (m/s) para la fórmula
        ms_ratio = self.get_parameter("effort_to_ms_ratio").value
        v_real_estimada = v_cmd * ms_ratio 
        
        # Ahora sí calculamos omega usando la velocidad real (m/s)
        omega = clamp(v_real_estimada * kappa, -self.max_omega, self.max_omega)

        self.publish_cmd(v_cmd, omega)

    def publish_cmd(self, v: float, w: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        # Se envía 'v' que es un porcentaje, pero 'w' fue calculado usando la física real.
        msg.twist.linear.x = float(v) 
        msg.twist.angular.z = float(w) 
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