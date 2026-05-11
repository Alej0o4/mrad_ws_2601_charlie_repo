#!/usr/bin/env python3
import math
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

# Usamos TwistStamped para que pase por tu twist_mux y llegue a yb_eb_node.py
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path

import tf2_ros
from tf2_ros import TransformException


def clamp(x: float, lo: float, hi: float) -> float:
    """Mantiene un valor 'x' dentro de los límites [lo, hi]."""
    return max(lo, min(hi, x))

def yaw_from_quaternion(q) -> float:
    """Extrae el ángulo Yaw (rotación en Z) de un cuaternión de forma súper rápida."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AckermannPurePursuitNode(Node):
    """
    Nodo de Pure Pursuit de Alto Rendimiento para Charlie.
    Usa vectorización con NumPy para el cálculo de la trayectoria.
    """
    def __init__(self):
        super().__init__("pure_pursuit_ackermann_node")

        # ---- Tópicos
        self.declare_parameter("path_topic", "/current_active_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("base_frame", "base_link")

        # ---- Hardware de Charlie
        self.declare_parameter("wheelbase", 0.257)          # [m] Distancia entre ejes
        self.declare_parameter("max_steering_angle", 0.349) # [rad] ~20 grados máximos del servo

        # ---- Desempeño
        self.declare_parameter("control_rate_hz", 10.0)    # [Hz] Tasa de actualización rápida para carrera
        self.declare_parameter("v_nominal", 0.15)           # [m/s] Velocidad de carrera
        self.declare_parameter("max_speed", 0.2)
        self.declare_parameter("goal_tolerance", 0.3)

        # ---- Lookahead Dinámico (Punto de mira)
        self.declare_parameter("lookahead_L0", 0.5)
        self.declare_parameter("lookahead_kv", 0.2)
        self.declare_parameter("lookahead_min", 0.4)
        self.declare_parameter("lookahead_max", 2.5)

        self.declare_parameter("eps", 1e-6)
        self.declare_parameter("tf_timeout_sec", 0.1)

        # ---- Leer Parámetros
        self.path_topic = self.get_parameter("path_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.max_steer = float(self.get_parameter("max_steering_angle").value)
        self.rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.v_nominal = float(self.get_parameter("v_nominal").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance").value)
        
        self.L0 = float(self.get_parameter("lookahead_L0").value)
        self.kv = float(self.get_parameter("lookahead_kv").value)
        self.Lmin = float(self.get_parameter("lookahead_min").value)
        self.Lmax = float(self.get_parameter("lookahead_max").value)
        self.eps = float(self.get_parameter("eps").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)

        # ---- Suscriptores y Publicadores
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, 10)

        # ---- TF2 (Para saber dónde está el robot)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Estado Interno
        self.path_array: Optional[np.ndarray] = None
        self.path_frame: Optional[str] = None
        self.has_path = False
        self.last_target_index = 0

        # ---- Iniciar Bucle
        dt = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(dt, self.on_timer)

        self.get_logger().info("🚀 Ackermann Pure Pursuit OPTIMIZADO (NumPy) iniciado para Charlie.")

    def on_path(self, msg: Path) -> None:
        """Carga la ruta en la memoria de NumPy UNA SOLA VEZ para máxima eficiencia."""
        self.path_frame = msg.header.frame_id if msg.header.frame_id else None
        
        if len(msg.poses) > 0 and self.path_frame is not None:
            # Transformamos los puntos a una matriz de NumPy [N, 2] donde col0=X, col1=Y
            self.path_array = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            self.has_path = True
            self.last_target_index = 0
            self.get_logger().info(f"Ruta matricial cargada: {len(self.path_array)} puntos listos.")
        else:
            self.has_path = False
            self.get_logger().warn("Ruta vacía recibida.")

    def on_timer(self) -> None:
        """Bucle principal de control a 50Hz."""
        if not self.has_path or self.path_array is None:
            self.publish_stop()
            return

        # 1. Obtener posición del robot en el mapa
        try:
            tf_map_to_base = self.tf_buffer.lookup_transform(
                self.path_frame, self.base_frame,
                rclpy.time.Time(), timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as ex:
            self.publish_stop()
            return

        # Coordenadas del robot
        rx = tf_map_to_base.transform.translation.x
        ry = tf_map_to_base.transform.translation.y
        ryaw = yaw_from_quaternion(tf_map_to_base.transform.rotation)

        cos_yaw = math.cos(ryaw)
        sin_yaw = math.sin(ryaw)

        # 2. Verificación de Meta
        goal_px, goal_py = self.path_array[-1]
        if math.hypot(goal_px - rx, goal_py - ry) <= self.goal_tol:
            self.publish_stop()
            self.has_path = False
            self.get_logger().info("🏁 ¡Meta alcanzada de forma exitosa!")
            return

        # 3. Lookahead Dinámico (Acelera mirada en rectas, recorta en curvas)
        v_cmd = clamp(self.v_nominal, 0.0, self.max_speed)
        Ld = clamp(self.L0 + self.kv * v_cmd, self.Lmin, self.Lmax)

        # ==========================================================
        # 4. OPTIMIZACIÓN NUMPY: Buscar objetivo sin bucles FOR
        # ==========================================================
        n = len(self.path_array)
        start = clamp(self.last_target_index, 0, n - 1)
        end = min(start + 200, n)  # Escaneamos 200 puntos hacia adelante (suficiente y rápido)

        segment = self.path_array[start:end]

        # Vectores desde el robot a todos los puntos del segmento
        dx = segment[:, 0] - rx
        dy = segment[:, 1] - ry

        # Rotación matricial 2D al frame del robot
        bx = dx * cos_yaw + dy * sin_yaw
        by = -dx * sin_yaw + dy * cos_yaw

        # Máscara: Solo puntos frente al robot
        front_mask = bx > 0.0

        if np.any(front_mask):
            valid_bx = bx[front_mask]
            valid_by = by[front_mask]
            
            # Distancias vectorizadas
            dist = np.hypot(valid_bx, valid_by)

            # Índices que superan el Lookahead (Ld)
            crossed_lookahead = np.where(dist >= Ld)[0]

            if len(crossed_lookahead) > 0:
                # Tomamos el primer punto que cruza la distancia de mira
                idx_in_valid = crossed_lookahead[0]
                target_bx = valid_bx[idx_in_valid]
                target_by = valid_by[idx_in_valid]
                
                # Guardar el índice para no re-escanear desde cero la próxima vez
                idx_in_segment = front_mask.nonzero()[0][idx_in_valid]
                self.last_target_index = start + idx_in_segment
            else:
                # Fallback: tomar el último punto frente a nosotros si ninguno llega a Ld
                target_bx = valid_bx[-1]
                target_by = valid_by[-1]
                self.last_target_index = start + front_mask.nonzero()[0][-1]
        else:
            self.publish_stop()
            return

        # 5. Matemática Ackermann (Ángulo de llanta)
        kappa = (2.0 * target_by) / (Ld * Ld + self.eps)
        steering_angle = clamp(math.atan(self.wheelbase * kappa), -self.max_steer, self.max_steer)

        # 6. Enviar a Hardware
        self.publish_cmd(v_cmd, steering_angle)

    def publish_cmd(self, speed: float, steering_angle: float) -> None:
        """Empaqueta la velocidad y el ángulo en un mensaje que Charlie entienda."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        
        # Velocidad en X, Ángulo de dirección del servo en Z
        cmd.twist.linear.x = float(speed)
        cmd.twist.angular.z = float(steering_angle)
        self.cmd_pub.publish(cmd)

    def publish_stop(self) -> None:
        self.publish_cmd(0.0, 0.0)


def main():
    rclpy.init()
    node = AckermannPurePursuitNode()
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