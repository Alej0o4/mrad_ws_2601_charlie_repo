#!/usr/bin/env python3
import math
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TwistStamped, Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # ¡Nuevo importe necesario para transformar puntos!

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
        self.declare_parameter("odom_frame", "odom") # <--- NUEVO PARÁMETRO
        self.declare_parameter("control_rate_hz", 10.0)

        self.declare_parameter("v_nominal_pct", 0.15)
        self.declare_parameter("max_speed_pct", 0.20)
        self.declare_parameter("min_speed_pct", 0.05)
        self.declare_parameter("effort_to_ms_ratio", 9.0) 
        self.declare_parameter("max_omega", 1.8)
        
        self.declare_parameter("goal_tolerance", 0.1)
        self.declare_parameter("decel_distance", 1.5)

        self.declare_parameter("lookahead_L0", 0.6)
        self.declare_parameter("lookahead_kv", 0.1)
        self.declare_parameter("lookahead_min", 0.4)
        self.declare_parameter("lookahead_max", 2.0)

        self.declare_parameter("eps", 1e-6)
        self.declare_parameter("tf_timeout_sec", 0.1)

        self.path_topic = self.get_parameter("path_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value # Guardamos odom
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
        self.marker_pub = self.create_publisher(MarkerArray, "/pure_pursuit/debug_markers", 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.global_path_poses = [] # Guardaremos las poses originales enteras
        self.path_frame: Optional[str] = None
        self.has_path = False
        self.last_target_index = 0

        self.timer = self.create_timer(1.0/self.rate_hz, self.on_timer)

    def on_path(self, msg: Path) -> None:
        self.path_frame = msg.header.frame_id if msg.header.frame_id else "map"
        if len(msg.poses) > 0 and self.path_frame is not None:
            # Guardamos los objetos PoseStamped originales para poder transformarlos luego
            self.global_path_poses = msg.poses
            self.has_path = True
            self.last_target_index = 0
        else:
            self.has_path = False

    def on_timer(self) -> None:
        if not self.has_path or len(self.global_path_poses) == 0:
            self.publish_stop()
            return

        try:
            # 1. Transformada para convertir el Path (map -> odom)
            tf_map_to_odom = self.tf_buffer.lookup_transform(
                self.odom_frame, self.path_frame, rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout)
            )
            
            # 2. Transformada suave del robot (odom -> base_link) para control
            tf_odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout)
            )
        except TransformException as e:
            # Si hay un salto en TF o no está listo, esperamos al siguiente ciclo
            return

        # Pose del robot en el frame suave ODOM
        rx = tf_odom_to_base.transform.translation.x
        ry = tf_odom_to_base.transform.translation.y
        ryaw = yaw_from_quaternion(tf_odom_to_base.transform.rotation)

        # Transformar solo la ventana relevante del path al frame ODOM
        start = self.last_target_index
        n = len(self.global_path_poses)
        
        # Extraemos el último punto global para la meta (lo transformamos a odom)
        goal_pose_odom = tf2_geometry_msgs.do_transform_pose_stamped(self.global_path_poses[-1], tf_map_to_odom)
        goal_x = goal_pose_odom.pose.position.x
        goal_y = goal_pose_odom.pose.position.y

        dist_to_goal = np.hypot(goal_x - rx, goal_y - ry)

        if dist_to_goal <= self.goal_tol:
            self.publish_stop()
            self.has_path = False
            return

        # Perfil de Velocidad
        v_nom_pct = self.get_parameter("v_nominal_pct").value
        min_pct = self.get_parameter("min_speed_pct").value
        decel_dist = self.get_parameter("decel_distance").value

        if dist_to_goal < decel_dist:
            v_cmd = min_pct + (v_nom_pct - min_pct) * (dist_to_goal / decel_dist)
        else:
            v_cmd = v_nom_pct

        v_cmd = clamp(v_cmd, min_pct, self.get_parameter("max_speed_pct").value)
        Ld = clamp(self.L0 + self.kv * v_cmd, self.Lmin, self.Lmax)

        # --- TRANSFORMAR VENTANA LOCAL A ODOM ---
        # En lugar de transformar los 500 puntos (costoso), tomamos desde el actual
        local_segment = self.global_path_poses[start:]
        path_array_odom = []
        
        for pose in local_segment:
            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(pose, tf_map_to_odom)
            path_array_odom.append([transformed_pose.pose.position.x, transformed_pose.pose.position.y])
            
        path_array_odom = np.array(path_array_odom)

        dx = path_array_odom[:, 0] - rx
        dy = path_array_odom[:, 1] - ry
        
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
        local_idx = valid_indices[target_idx]
        
        self.last_target_index = start + local_idx

        # Coordenadas para dibujar los markers (ahora los dibujamos en odom)
        target_x_odom = path_array_odom[local_idx, 0]
        target_y_odom = path_array_odom[local_idx, 1]

        # Control Cinemático Real
        kappa = (2.0 * target_by) / (Ld * Ld + self.eps)
        ms_ratio = self.get_parameter("effort_to_ms_ratio").value
        v_real_estimada = v_cmd * ms_ratio 
        omega = clamp(v_real_estimada * kappa, -self.max_omega, self.max_omega)

        self.publish_cmd(v_cmd, omega)
        
        # Markers: Se publican en odom frame
        self.publish_debug_markers(target_x_odom, target_y_odom, target_bx, target_by, Ld)

    def publish_debug_markers(self, tx_odom: float, ty_odom: float, tx_local: float, ty_local: float, Ld: float) -> None:
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        m_target = Marker()
        m_target.header.frame_id = self.odom_frame # AHORA USAMOS ODOM
        m_target.header.stamp = now
        m_target.ns = "lookahead_point"
        m_target.id = 0
        m_target.type = Marker.SPHERE
        m_target.action = Marker.ADD
        m_target.pose.position.x = float(tx_odom)
        m_target.pose.position.y = float(ty_odom)
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