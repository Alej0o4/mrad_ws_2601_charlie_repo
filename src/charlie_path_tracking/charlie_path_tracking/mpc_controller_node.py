#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple, Dict

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path

import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose_stamped

# Importación crucial para el MPC
from scipy.optimize import minimize
import numpy as np
from typing import Tuple, Dict

def euler_from_quaternion(q) -> float:
    """
    Función auxiliar para convertir un cuaternión de ROS2 a ángulos de Euler.
    Retorna únicamente el ángulo Yaw (rotación sobre el eje Z).
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MpcControllerNode(Node):
    """
    Controlador MPC Cinemático para robot de accionamiento diferencial.
    Utiliza CasADi + IPOPT para la optimización en tiempo real.
    """

    def __init__(self):
        super().__init__("mpc_controller_node")

        # ==========================================================
        # 1. DECLARACIÓN DE PARÁMETROS (ROS2)
        # ==========================================================
        # Tópicos y Frames (Reciclado de Pure Pursuit)
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("tf_timeout_sec", 0.2)

        # Parámetros del Horizonte de Predicción (Nuevos para MPC)
        self.declare_parameter("mpc_N", 10)         # Horizonte de predicción (pasos)
        self.declare_parameter("mpc_dt", 0.1)       # Tiempo de muestreo (segundos)

        # Restricciones Cinemáticas (Límites de los actuadores)
        self.declare_parameter("v_max", 1.0)        # m/s
        self.declare_parameter("v_min", -0.2)       # m/s (permite ir en reversa un poco)
        self.declare_parameter("omega_max", 1.5)    # rad/s

        # Matrices de Pesos (Q: Estado, R: Control R_d:Cambio de la ley de control) - ¡Afinables al vuelo!
        self.declare_parameter("weight_x", 1.0)
        self.declare_parameter("weight_y", 1.0)
        self.declare_parameter("weight_theta", 0.5)
        self.declare_parameter("weight_v", 0.1)
        self.declare_parameter("weight_omega", 0.1)
        self.declare_parameter("weight_accel", 0.5)   # Penaliza cambios bruscos en v
        self.declare_parameter("weight_alpha", 0.5) # Penaliza cambios bruscos en omega

        # Lectura de parámetros... (Omitida por brevedad, asume que se leen aquí)
        self._load_parameters()

        # ==========================================================
        # 2. INFRAESTRUCTURA ROS2 (I/O)
        # ==========================================================
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, 10)
        
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Estado interno
        self.path: List[PoseStamped] = []
        self.path_frame: Optional[str] = None
        self.has_path = False

        # ==========================================================
        # 3. INICIALIZACIÓN DEL SOLVER MPC
        # ==========================================================
        # Aquí pre-compilamos el problema de optimización en memoria
        self.solver, self.mpc_args = self._setup_mpc_problem()

        # Timer de control
        dt_timer = 1.0 / self.rate_hz
        # Variables de estado para MPC y Warm Start
        self.last_cmd = np.array([0.0, 0.0])
        self.U_prev = np.zeros(2 * self.N) # Memoria del Warm Start
        self.last_closest_index = 0

        self.timer = self.create_timer(dt_timer, self.on_timer)
        self.get_logger().info("Nodo MPC inicializado y esperando trayectoria.")

    def _load_parameters(self):
        """Lee todos los parámetros declarados y los asigna a variables de clase."""
        self.path_topic = self.get_parameter("path_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.N = self.get_parameter("mpc_N").value
        self.dt = self.get_parameter("mpc_dt").value
        # ... (leer el resto de pesos y límites) ...

    # ==========================================================
    # SECCIÓN MATEMÁTICA (El Cerebro)
    # ==========================================================
    def _setup_mpc_problem(self) -> Tuple[None, Dict]:
        """
        Prepara las matrices de pesos y los límites para SciPy.
        Al usar Single-Shooting, el optimizador solo buscará un vector 1D
        con la secuencia de comandos: U = [v0, w0, v1, w1, ..., vN-1, wN-1]
        """
        self.get_logger().info("Configurando MPC usando SciPy (Single-Shooting)...")
        
        # 1. Construir las matrices de pesos Q y R como arreglos de numpy
        self.Q = np.diag([
            self.get_parameter("weight_x").value,
            self.get_parameter("weight_y").value,
            self.get_parameter("weight_theta").value
        ])
        
        self.R = np.diag([
            self.get_parameter("weight_v").value,
            self.get_parameter("weight_omega").value
        ])
        self.R_d = np.diag([
            self.get_parameter("weight_accel").value, # peso para suavizar aceleración lineal
            self.get_parameter("weight_alpha").value  # peso para suavizar aceleración angular
        ])
        
        # 2. Definir los límites (bounds) de los actuadores para todo el horizonte
        # self.N veces el par (limites_v, limites_w)
        v_min = self.get_parameter("v_min").value
        v_max = self.get_parameter("v_max").value
        w_max = self.get_parameter("omega_max").value
        
        self.bounds = []
        for _ in range(self.get_parameter("mpc_N").value):
            self.bounds.append((v_min, v_max))       # Límites para v_k
            self.bounds.append((-w_max, w_max))      # Límites para omega_k
            
        return None, {} # SciPy no compila un objeto solver externo
    
    def _kinematic_model(self, state: np.ndarray, v: float, omega: float) -> np.ndarray:
        """
        Ecuaciones del modelo cinemático discreto (Método de Euler).
        state = [x, y, theta]
        """
        x, y, theta = state[0], state[1], state[2]
        
        x_next = x + v * np.cos(theta) * self.dt
        y_next = y + v * np.sin(theta) * self.dt
        theta_next = theta + omega * self.dt
        
        return np.array([x_next, y_next, theta_next])
    
    def _cost_function(self, U: np.ndarray, current_state: np.ndarray, ref_traj: np.ndarray, current_cmd: np.ndarray) -> float:
        """
        Función objetivo J que SciPy intentará minimizar.
        Simula el robot hacia el futuro usando los comandos U y penaliza el error.
        """
        cost = 0.0
        state = np.copy(current_state)

        prev_u = np.copy(current_cmd)        
        # U viene como un arreglo 1D plano: [v0, w0, v1, w1, ...]
        for k in range(self.N):
            v = U[2*k]
            omega = U[2*k + 1]
            u_vec = np.array([v, omega])
            
            
            # 1. Simular un paso hacia adelante (La física)
            state = self._kinematic_model(state, v, omega)
            
            # 2. Calcular el error respecto a la trayectoria de referencia
            error = state - ref_traj[k]
            
            # Normalizar el error de orientación (theta) para que esté entre -pi y pi.
            # Esto evita que el robot dé vueltas locas si el error salta a 2*pi.
            error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))
            
            # 3. Sumar el costo de este paso: e^T * Q * e  +  u^T * R * u
            delta_u = u_vec - prev_u # Calcular el cambio brusco de velocidad (Delta U)
            
            state_cost = error.T @ self.Q @ error
            control_cost = u_vec.T @ self.R @ u_vec
            delta_control_cost = delta_u.T @ self.R_d @ delta_u
            
            cost += (state_cost + control_cost + delta_control_cost)

            prev_u = u_vec
            
        return cost
    
    def _solve_mpc(self, current_state: np.ndarray, ref_traj: np.ndarray) -> Tuple[float, float]:
        """
        Ejecuta el optimizador SLSQP de SciPy.
        """
        # Adivinanza inicial: Asumir que el robot se queda quieto (todo ceros)
        U0 = np.zeros(2 * self.N)
        U0[:-2] = self.U_prev[2:]  # Desplazar controles hacia la izquierda
        U0[-2:] = self.U_prev[-2:] # Repetir el último comando para llenar el vacío
        
        # Llamar al motor de optimización
        res = minimize(
            self._cost_function, 
            U0, 
            args=(current_state, ref_traj, self.last_cmd), 
            method='SLSQP', 
            bounds=self.bounds,
            options={'ftol': 1e-3, 'maxiter': 50}
        )
        
        if res.success:
            self.U_prev = res.x
            # Extraer solo el PRIMER par de comandos (Receding Horizon)
            v_opt = res.x[0]
            omega_opt = res.x[1]

            # Actualizar el último comando enviado
            self.last_cmd = np.array([v_opt, omega_opt])
            return float(v_opt), float(omega_opt)
        else:
            self.get_logger().warn(f"Optimizador falló: {res.message}. Deteniendo robot.")
            self.U_prev = np.zeros(2 * self.N) # Limpiar memoria corrupta
            self.last_cmd = np.array([0.0, 0.0])
            return 0.0, 0.0

    def _get_local_reference_trajectory(self, tf) -> np.ndarray:
        """
        Extrae un segmento de N puntos de la trayectoria global,
        transformados al marco base_link, para alimentar al solver.
        """
        ref_traj = np.zeros((self.N, 3))
        
        # Seguridad: Si no hay ruta, retornamos ceros (el robot intentará quedarse quieto)
        if not self.path:
            return ref_traj

        # ---------------------------------------------------------
        # PASO 1: Encontrar el índice del punto más cercano al robot
        # ---------------------------------------------------------
        min_dist = float('inf')
        closest_idx = 0
        
        # Reciclamos tu lógica de Pure Pursuit: empezar desde el último índice conocido
        # para no tener que buscar desde el inicio de la ruta en cada iteración.
        start_idx = getattr(self, 'last_closest_index', 0)
        n_poses = len(self.path)
        
        # Limitamos la búsqueda hacia adelante para eficiencia (ej. buscar en los próximos 50 puntos)
        search_range = min(start_idx + 50, n_poses)
        
        for i in range(start_idx, search_range):
            # Transformamos el punto global al chasis del robot
            pose_b = do_transform_pose_stamped(self.path[i], tf)
            x = pose_b.pose.position.x
            y = pose_b.pose.position.y
            
            # Distancia euclidiana al origen del base_link (0,0)
            dist = math.hypot(x, y)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Guardamos el índice para la próxima iteración (Timer)
        self.last_closest_index = closest_idx

        # ---------------------------------------------------------
        # PASO 2: Extraer N puntos hacia el futuro
        # ---------------------------------------------------------
        # En una implementación avanzada, calcularías la distancia geométrica entre puntos.
        # Aquí usaremos un salto de índices (index_step) aproximado.
        # Si tu path tiene puntos cada 5cm, y esperas que el robot avance 10cm por cada dt (0.1s),
        # deberías saltar de a 2 índices. Lo dejaremos parametrizado.
        
        index_step = 1 # Esto debería ser idealmente un parámetro o calculado dinámicamente
        
        for k in range(self.N):
            # Calculamos el índice objetivo saltando hacia adelante
            target_idx = closest_idx + (k * index_step)
            
            # Si el target_idx supera el final de la ruta, nos quedamos en el último punto (padding)
            # Esto es vital para que el robot se detenga suavemente al final y no se salga de rango.
            target_idx = min(target_idx, n_poses - 1)
            
            # Transformar el punto objetivo al base_link
            pose_b = do_transform_pose_stamped(self.path[target_idx], tf)
            
            x = pose_b.pose.position.x
            y = pose_b.pose.position.y
            
            # Extraer el cuaternión y convertirlo a Yaw (theta)
            q = pose_b.pose.orientation
            yaw = euler_from_quaternion(q)
            
            # Llenar la matriz de referencia
            ref_traj[k] = [x, y, yaw]

        return ref_traj

    # ==========================================================
    # SECCIÓN ROS2 (El Middleware y Loop de Control)
    # ==========================================================
    def on_path(self, msg: Path) -> None:
        """Callback de la trayectoria (Idéntico a Pure Pursuit)."""
        self.path = list(msg.poses)
        self.path_frame = msg.header.frame_id
        self.has_path = len(self.path) > 0

        self.last_closest_index = 0

    def on_timer(self) -> None:
        """Loop de control principal a N Hz."""
        if not self.has_path:
            self.publish_stop()
            return

        # 1. Obtener la Transformada Actual
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.path_frame,
                rclpy.time.Time(), timeout=Duration(seconds=self.tf_timeout)
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup falló: {ex}")
            self.publish_stop()
            return

        # 2. Verificar si llegamos a la meta (Idéntico a Pure Pursuit)
        # ... (Lógica de goal_dist <= goal_tol) ...

        # 3. Preparar los datos para el MPC
        # El estado actual del robot respecto a su propio chasis siempre es cero
        current_state = np.array([0.0, 0.0, 0.0]) 
        
        # Obtener la referencia futura local (Horizonte N)
        ref_traj = self._get_local_reference_trajectory(tf)

        # 4. Resolver el problema de optimización
        v_cmd, omega_cmd = self._solve_mpc(current_state, ref_traj)

        # 5. Publicar el comando
        self._publish_cmd(v_cmd, omega_cmd)

    def _publish_cmd(self, v: float, omega: float) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def publish_stop(self) -> None:
        self._publish_cmd(0.0, 0.0)

def main():
    rclpy.init()
    node = MpcControllerNode()
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