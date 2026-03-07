#!/usr/bin/env python3
"""
ARA* (Anytime Repairing A*) grid planner (ROS 2).
Basado en la estructura de Dijkstra proporcionada en clase.
"""
import math
import heapq
import time
from collections import deque
from typing import List, Tuple, Optional, Dict, Set

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
from dataclasses import dataclass, field
from typing import Tuple, Optional

@dataclass
class ARANode:
    x: int
    y: int
    g: float = float('inf')
    v: float = float('inf')
    parent: Optional[Tuple[int, int]] = None
    
    # Mantenemos el __lt__ para heapq
    def __lt__(self, other):
        return False


class ARAPlannerNode(Node):
    def __init__(self):
        super().__init__('ara_planner_node')

        # --- Parámetros de ROS 2 (Reciclados de Dijkstra) ---
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        
        # Opciones de grilla
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('use_8_connected', True)
        self.declare_parameter('inflate_radius', 0.15)

        # --- Parámetros NUEVOS para ARA* ---
        self.declare_parameter('epsilon_start', 2.5)       # Inflación inicial (Modo rápido)
        self.declare_parameter('epsilon_decrease', 0.5)    # Cuánto baja en cada iteración
        self.declare_parameter('time_limit_sec', 0.5)      # Presupuesto de tiempo total

        # --- Subscripciones y Publicadores ---
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value

        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        qos_map = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_cb, qos_map)

        # --- TF2 (Reciclado) ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Variables internas
        self._map: Optional[OccupancyGrid] = None
        self._obstacles: Optional[np.ndarray] = None
        self._dist_cells: Optional[np.ndarray] = None

        self.get_logger().info("ARA* Planner Node Iniciado y esperando el mapa...")

    # =================================================================
    # CALLBACKS DE ROS 2 (Reciclados - Mismo flujo que Dijkstra)
    # =================================================================
    def map_cb(self, msg: OccupancyGrid):
        """Recibe el mapa, lo guarda y pre-calcula los obstáculos (Brushfire)."""
        # [COPIA EXACTA DE LA FUNCIÓN map_cb DE TU SCRIPT DIJKSTRA]
        pass

    def compute_distance_to_obstacles(self, obstacles: np.ndarray) -> np.ndarray:
        """Usado para inflar paredes si es necesario."""
        # [COPIA EXACTA DE LA FUNCIÓN DE TU SCRIPT DIJKSTRA]
        pass

    def goal_cb(self, msg: PoseStamped):
        """Se activa al recibir una meta en RViz. Aquí arranca el ARA*."""
        if self._map is None or self._obstacles is None:
            self.get_logger().warn("No hay mapa todavía.")
            return

        # 1. Obtener la posición actual del robot (Start)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('global_frame').value,
                self.get_parameter('base_frame').value,
                rclpy.time.Time()
            )
            # Crear PoseStamped temporal para el inicio
            start_pose = PoseStamped()
            start_pose.header.frame_id = self.get_parameter('global_frame').value
            start_pose.pose.position.x = transform.transform.translation.x
            start_pose.pose.position.y = transform.transform.translation.y
            
        except Exception as e:
            self.get_logger().error(f"Error obteniendo TF: {e}")
            return

        # 2. Llamar al motor matemático del ARA*
        path_msg = self.plan_ara_star(start_pose, msg)

        # 3. Publicar si se encontró una ruta
        if path_msg is not None:
            self.path_pub.publish(path_msg)
            self.get_logger().info("¡Ruta ARA* publicada!")
        else:
            self.get_logger().error("ARA* falló al encontrar una ruta.")

    # =================================================================
    # 4. FUNCIONES AUXILIARES 
    # =================================================================
    def world_to_map(self, x, y, x0, y0, res, W, H) -> Optional[Tuple[int, int]]:
        # [COPIA EXACTA DE DIJKSTRA]
        pass

    def map_to_world(self, ix, iy, x0, y0, res) -> Tuple[float, float]:
        # [COPIA EXACTA DE DIJKSTRA]
        pass

    def get_neighbors(self, ix, iy, W, H) -> List[Tuple[int, int, float]]:
        """Retorna vecinos válidos y el costo de transición c(s, s')"""
        # [COPIA EXACTA DE DIJKSTRA]
        pass

    # =================================================================
    # 5. EL NÚCLEO MATEMÁTICO ARA* (Funciones COMPLETAMENTE NUEVAS)
    # =================================================================
    def calculate_heuristic(self, curr_idx: Tuple[int, int], goal_idx: Tuple[int, int]) -> float:
        """
        Calcula la estimación h(s) desde el nodo actual a la meta.
        Puedes usar distancia Euclidiana o Manhattan.
        """
        pass

    def f_value(self, g: float, h: float, epsilon: float) -> float:
        """Retorna el costo total estimado: f(s) = g(s) + epsilon * h(s)"""
        pass

    def improve_path(self, goal_idx, epsilon, state_space, OPEN, CLOSED, INCONS):
        """
        El equivalente al bucle interno de Dijkstra. 
        Expande nodos de OPEN mientras f(goal) > min(f(s) en OPEN).
        Actualiza g(s), v(s) y manda nodos a INCONS si es necesario.
        """
        pass

    def reconstruct_path(self, start_idx, goal_idx, state_space, path_msg_header, x0, y0, res) -> Path:
        """
        Navega hacia atrás usando state_space[nodo].parent para 
        construir el mensaje nav_msgs/Path a publicar en RViz.
        """
        pass

    def plan_ara_star(self, start: PoseStamped, goal: PoseStamped) -> Optional[Path]:
        """
        El procedimiento Main() del paper de Likhachev.
        Controla el presupuesto de tiempo y el ciclo de decremento de epsilon.
        """
        # 1. Extraer metadata del mapa
        info = self._map.info
        res = info.resolution
        x0, y0 = info.origin.position.x, info.origin.position.y
        W, H = info.width, info.height

        # 2. Convertir start y goal a índices
        s_idx = self.world_to_map(start.pose.position.x, start.pose.position.y, x0, y0, res, W, H)
        g_idx = self.world_to_map(goal.pose.position.x, goal.pose.position.y, x0, y0, res, W, H)

        if s_idx is None or g_idx is None:
            self.get_logger().error("Start o Goal fuera del mapa.")
            return None

        # 3. Inicializar parámetros del ARA*
        epsilon = self.get_parameter('epsilon_start').value
        eps_dec = self.get_parameter('epsilon_decrease').value
        time_limit = self.get_parameter('time_limit_sec').value

        # 4. Inicializar estructuras de datos (Las tres listas y el State Space)
        OPEN = []       # Cola de prioridad (heapq)
        CLOSED = set()  # Set para búsqueda rápida
        INCONS = set()  # Nodos a reparar
        
        # Diccionario para guardar todos los objetos ARANode instanciados
        # Llave: Tupla (x,y), Valor: Objeto ARANode
        state_space: Dict[Tuple[int, int], ARANode] = {} 

        # Crear nodo inicial
        start_node = ARANode(s_idx[0], s_idx[1])
        start_node.g = 0.0
        state_space[s_idx] = start_node
        
        # Calcular h(s_start) e insertarlo en OPEN
        h_start = self.calculate_heuristic(s_idx, g_idx)
        f_start = self.f_value(start_node.g, h_start, epsilon)
        heapq.heappush(OPEN, (f_start, s_idx))

        # 5. El Bucle Anytime (Controlado por tiempo y epsilon)
        start_time = time.time()
        best_path_found = False

        while epsilon >= 1.0:
            self.get_logger().info(f"Buscando ruta con epsilon={epsilon:.2f}...")
            
            # Llamar al motor de expansión
            self.improve_path(g_idx, epsilon, state_space, OPEN, CLOSED, INCONS)

            # Verificar si ImprovePath conectó el inicio con la meta
            if g_idx in state_space and state_space[g_idx].g < float('inf'):
                best_path_found = True
                self.get_logger().info(f"¡Ruta subóptima encontrada para eps={epsilon:.2f}!")
            
            # Revisar si se nos acabó el tiempo
            if (time.time() - start_time) > time_limit:
                self.get_logger().warn("Tiempo de cálculo agotado.")
                break
            
            # --- Fase de Reparación ---
            if epsilon == 1.0:
                break # Ya encontramos la óptima, salir del bucle
                
            # Disminuir epsilon
            epsilon -= eps_dec
            if epsilon < 1.0:
                epsilon = 1.0

            # Mover INCONS a OPEN
            for node_idx in INCONS:
                # Recalcular F con el nuevo epsilon y pushear a OPEN
                pass
            
            INCONS.clear()
            CLOSED.clear()

            # (Opcional) Reconstruir OPEN completamente para actualizar 
            # las prioridades F(s) de los nodos que ya estaban adentro.
            pass

        # 6. Reconstruir la ruta y retornar
        if best_path_found:
            return self.reconstruct_path(s_idx, g_idx, state_space, start.header, x0, y0, res)
        else:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ARAPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()