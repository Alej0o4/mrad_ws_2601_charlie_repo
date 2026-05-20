#!/usr/bin/env python3
import math
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TwistStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, ReliabilityPolicy

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        self.declare_parameter("path_topic", "/smoothed_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("control_rate_hz", 10.0)

        # ---- Parámetros de Mapeo ESC (Curva Real)
        # Comando PWM normalizado
        self.declare_parameter("esc_cmd_points", [0.1, 0.2, 0.35, 0.5])
        # Velocidad física empírica (m/s) correspondientes
        self.declare_parameter("esc_speed_points", [1.3, 1.4, 1.7, 2.0])
        
        # ---- Parámetros de Velocidad Nominal
        self.declare_parameter("v_nominal_pct", 0.35)
        self.declare_parameter("max_speed_pct", 0.50)
        self.declare_parameter("min_speed_pct", 0.10)

        
        # ---- Parámetros de Frenado Dinámico (Curvatura y Meta)
        self.declare_parameter("goal_tolerance", 0.1)
        self.declare_parameter("decel_distance", 1.5)
        self.declare_parameter("curve_lookahead_pts", 20)
        self.declare_parameter("max_lat_accel", 1.2) # Aceleración centrípeta máxima permisible (m/s^2)

        # ---- Parámetros de Lookahead
        self.declare_parameter("lookahead_L0", 0.6)
        self.declare_parameter("lookahead_kv", 0.3)
        self.declare_parameter("lookahead_min", 0.4)
        self.declare_parameter("lookahead_max", 2.0)

        self.declare_parameter("eps", 1e-6)
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("wheelbase", 0.257)        # Distancia entre eje trasero y delantero (m)
        self.declare_parameter("max_steer_rad", 0.349)   # Ángulo máximo de dirección (ej: 20 grados)

        self.path_topic = self.get_parameter("path_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.rate_hz = self.get_parameter("control_rate_hz").value

        
        self.L0 = self.get_parameter("lookahead_L0").value
        self.kv = self.get_parameter("lookahead_kv").value
        self.Lmin = self.get_parameter("lookahead_min").value
        self.Lmax = self.get_parameter("lookahead_max").value
        self.goal_tol = self.get_parameter("goal_tolerance").value
        self.eps = self.get_parameter("eps").value
        self.tf_timeout = self.get_parameter("tf_timeout_sec").value

        path_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/pure_pursuit/debug_markers", 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, path_qos)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_array: Optional[np.ndarray] = None
        self.path_frame: Optional[str] = None
        self.has_path = False
        self.last_target_index = 0

        self.timer = self.create_timer(1.0/self.rate_hz, self.on_timer)

    def on_path(self, msg: Path) -> None:
        self.path_frame = msg.header.frame_id if msg.header.frame_id else "map"
        if len(msg.poses) > 0 and self.path_frame is not None:
            self.path_array = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            self.has_path = True
            self.last_target_index = 0
        else:
            self.has_path = False

    def _get_max_curvature(self, start_idx: int) -> float:
        """Calcula la curvatura de Menger máxima en una ventana futura para pre-frenado."""
        if self.path_array is None:
            return 0.0
            
        lookahead_pts = self.get_parameter("curve_lookahead_pts").value
        end_idx = min(start_idx + lookahead_pts, len(self.path_array))
        
        # Necesitamos al menos 3 puntos separados para calcular curvatura sin ruido
        step = 3 
        pts = self.path_array[start_idx:end_idx:step]
        
        if len(pts) < 3:
            return 0.0

        # Vectorizamos el cálculo del área y lados para P[i-1], P[i], P[i+1]
        A = pts[:-2]
        B = pts[1:-1]
        C = pts[2:]

        # Lados del triángulo
        a = np.linalg.norm(B - C, axis=1)
        b = np.linalg.norm(A - C, axis=1)
        c = np.linalg.norm(A - B, axis=1)

        # Filtro de seguridad por si hay puntos duplicados (distancia 0)
        valid = (a > self.eps) & (b > self.eps) & (c > self.eps)
        if not np.any(valid):
            return 0.0

        A, B, C = A[valid], B[valid], C[valid]
        a, b, c = a[valid], b[valid], c[valid]

        # Área mediante determinantes (Cross product 2D)
        area = 0.5 * np.abs(A[:,0]*(B[:,1] - C[:,1]) + B[:,0]*(C[:,1] - A[:,1]) + C[:,0]*(A[:,1] - B[:,1]))
        
        # Curvatura de Menger: K = 4*Area / (a*b*c)
        curvatures = (4.0 * area) / (a * b * c)
        
        return float(np.max(curvatures))

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

        dist_to_goal = np.hypot(self.path_array[-1, 0] - rx, self.path_array[-1, 1] - ry)
        puntos_restantes = len(self.path_array) - self.last_target_index
        estamos_en_recta_final = puntos_restantes < 60

        if dist_to_goal <= self.goal_tol and estamos_en_recta_final:
            self.publish_stop()
            self.has_path = False
            return

        # --- EXTRACCIÓN DE PARÁMETROS DINÁMICOS ---
        v_nom_pct = self.get_parameter("v_nominal_pct").value
        min_pct = self.get_parameter("min_speed_pct").value
        decel_dist = self.get_parameter("decel_distance").value
        
        esc_cmd_pts = self.get_parameter("esc_cmd_points").value
        esc_speed_pts = self.get_parameter("esc_speed_points").value
        max_lat_accel = self.get_parameter("max_lat_accel").value

        # 1. Base Target Velocity
        if estamos_en_recta_final and dist_to_goal < decel_dist:
            v_cmd_target = min_pct + (v_nom_pct - min_pct) * (dist_to_goal / decel_dist)
        else:
            v_cmd_target = v_nom_pct

        # 2. Limitación por Curvatura Anticipada
        k_max = self._get_max_curvature(self.last_target_index)
        
        if k_max > 0.01: # Si hay una curva apreciable
            # Máxima velocidad física permitida por la aceleración lateral ( V = sqrt(a / k) )
            v_real_limit = math.sqrt(max_lat_accel / k_max)
            # Mapeo inverso: Velocidad física -> Comando ESC necesario
            cmd_limit = np.interp(v_real_limit, esc_speed_pts, esc_cmd_pts)
            v_cmd_target = min(v_cmd_target, float(cmd_limit))

        # Asegurar límites absolutos del comando
        v_cmd = clamp(v_cmd_target, min_pct, self.get_parameter("max_speed_pct").value)
        
        # Lookahead dinámico basado en el comando FINAL
        Ld = clamp(self.L0 + self.kv * v_cmd, self.Lmin, self.Lmax)

        # --- BÚSQUEDA DEL PUNTO OBJETIVO ---
        start = self.last_target_index
        segment = self.path_array[start:]

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
        else:
            target_idx = -1

        target_by = v_by[target_idx]
        target_bx = v_bx[target_idx]
        global_idx = start + valid_indices[target_idx]
        target_x_global = self.path_array[global_idx, 0]
        target_y_global = self.path_array[global_idx, 1]
        
        self.last_target_index = global_idx

        # --- CONTROL DE DIRECCIÓN CON ESTIMACIÓN REAL ---
        kappa = (2.0 * target_by) / (Ld * Ld + self.eps)
        v_real_estimada = np.interp(v_cmd, esc_cmd_pts, esc_speed_pts)
        
        # 1. Calculamos el ángulo de dirección ideal basado en la curvatura deseada
        wheelbase = self.get_parameter("wheelbase").value
        delta_ideal = math.atan(kappa * wheelbase)
        
        # 2. Saturamos el ángulo de dirección a los límites mecánicos reales de tu servo
        max_steer = self.get_parameter("max_steer_rad").value
        delta_cmd = clamp(delta_ideal, -max_steer, max_steer)
        
        # 3. Calculamos la omega real que corresponde a esa dirección factible
        omega = (v_real_estimada * math.tan(delta_cmd)) / wheelbase

        self.publish_cmd(v_cmd, omega)
        self.publish_debug_markers(target_x_global, target_y_global, target_bx, target_by, Ld)

    # --- Los métodos publish_debug_markers, publish_cmd, publish_stop se mantienen idénticos ---
    def publish_debug_markers(self, tx_global: float, ty_global: float, tx_local: float, ty_local: float, Ld: float) -> None:
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        m_target = Marker()
        m_target.header.frame_id = self.path_frame
        m_target.header.stamp = now
        m_target.ns = "lookahead_point"
        m_target.id = 0
        m_target.type = Marker.SPHERE
        m_target.action = Marker.ADD
        m_target.pose.position.x = float(tx_global)
        m_target.pose.position.y = float(ty_global)
        m_target.pose.position.z = 0.0
        m_target.scale.x = 0.2
        m_target.scale.y = 0.2
        m_target.scale.z = 0.2
        m_target.color.r = 0.0
        m_target.color.g = 1.0
        m_target.color.b = 0.0
        m_target.color.a = 1.0

        m_radius = Marker()
        m_radius.header.frame_id = self.base_frame
        m_radius.header.stamp = now
        m_radius.ns = "lookahead_radius"
        m_radius.id = 1
        m_radius.type = Marker.CYLINDER
        m_radius.action = Marker.ADD
        m_radius.pose.position.x = 0.0
        m_radius.pose.position.y = 0.0
        m_radius.pose.position.z = -0.05 
        m_radius.scale.x = float(Ld * 2.0)
        m_radius.scale.y = float(Ld * 2.0)
        m_radius.scale.z = 0.02 
        m_radius.color.r = 0.0
        m_radius.color.g = 0.5
        m_radius.color.b = 1.0
        m_radius.color.a = 0.2 

        m_arrow = Marker()
        m_arrow.header.frame_id = self.base_frame
        m_arrow.header.stamp = now
        m_arrow.ns = "steering_vector"
        m_arrow.id = 2
        m_arrow.type = Marker.ARROW
        m_arrow.action = Marker.ADD
        p_start = Point(x=0.0, y=0.0, z=0.0)
        p_end = Point(x=float(tx_local), y=float(ty_local), z=0.0)
        m_arrow.points = [p_start, p_end]
        m_arrow.scale.x = 0.05 
        m_arrow.scale.y = 0.10 
        m_arrow.scale.z = 0.10 
        m_arrow.color.r = 1.0
        m_arrow.color.g = 0.0
        m_arrow.color.b = 0.0
        m_arrow.color.a = 0.8

        marker_array.markers = [m_target, m_radius, m_arrow]
        self.marker_pub.publish(marker_array)

    def publish_cmd(self, v: float, w: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = float(v) 
        msg.twist.angular.z = float(w) 
        self.cmd_pub.publish(msg)

    def publish_stop(self) -> None:
        self.publish_cmd(0.0, 0.0)
        
        marker_array = MarkerArray()
        m_del = Marker()
        m_del.action = Marker.DELETEALL
        marker_array.markers.append(m_del)
        self.marker_pub.publish(marker_array)

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