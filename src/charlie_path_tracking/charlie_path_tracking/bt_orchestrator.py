#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path  # IMPORTANTE: Añadir el mensaje de la ruta
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
import py_trees
import py_trees_ros.trees
import math
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

# ==========================================
# 1. COMPORTAMIENTO (Lógica Pura con Proyección)
# ==========================================
class IsPathClear(py_trees.behaviour.Behaviour):
    def __init__(self, name="Camino_Libre_?", lookahead_distance=3.0, 
                 robot_length=0.5, robot_width=0.3):
        super(IsPathClear, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name="Lector_Sensores")
        self.blackboard.register_key(key="latest_scan", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="latest_path", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="tf_buffer", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="node_logger", access=py_trees.common.Access.READ)
        
        # Asignamos los valores que nos pasen desde el Nodo ROS 2
        self.lookahead_distance = lookahead_distance
        self.robot_length = robot_length      # Largo del vehículo Ackermann (m)
        self.robot_width = robot_width        # Ancho del vehículo Ackermann (m)

    def _distance_point_to_rect(self, px, py, rect_x, rect_y, yaw, half_length, half_width):
        """Calcula la distancia mínima de un punto a un rectángulo orientado.
        
        Args:
            px, py: Coordenadas del punto en el frame del láser
            rect_x, rect_y: Centro del rectángulo (posición del robot)
            yaw: Ángulo de orientación del rectángulo (radianes)
            half_length: Mitad del largo del robot (eje X local)
            half_width: Mitad del ancho del robot (eje Y local)
            
        Returns:
            Distancia mínima del punto al rectángulo (negativa si está adentro)
        """
        # Trasladar el punto al sistema de coordenadas del rectángulo
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        # Vector del centro del rectángulo al punto
        dx = px - rect_x
        dy = py - rect_y
        
        # Proyectar el punto en los ejes locales del rectángulo
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        
        # Calcular la distancia desde el punto al rectángulo en coordenadas locales
        closest_x = max(-half_length, min(half_length, local_x))
        closest_y = max(-half_width, min(half_width, local_y))
        
        distance_x = local_x - closest_x
        distance_y = local_y - closest_y
        
        return math.hypot(distance_x, distance_y)

    def update(self):
        try:
            scan = self.blackboard.latest_scan
            path = self.blackboard.latest_path
            tf_buffer = self.blackboard.tf_buffer
            node_logger = self.blackboard.node_logger
            
            if scan is None or path is None or tf_buffer is None:
                return py_trees.common.Status.RUNNING

            source_frame = path.header.frame_id
            target_frame = scan.header.frame_id
            if not source_frame or not target_frame:
                return py_trees.common.Status.RUNNING

            try:
                transform = tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time()
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return py_trees.common.Status.RUNNING

            ruta_transformada = []
            distancia_acumulada = 0.0
            punto_anterior = None

            for pose_stamped in path.poses:
                punto_actual = pose_stamped.pose.position
                if punto_anterior is not None:
                    distancia_acumulada += math.hypot(
                        punto_actual.x - punto_anterior.x,
                        punto_actual.y - punto_anterior.y,
                    )
                if distancia_acumulada > self.lookahead_distance:
                    break

                pose_en_origen = PoseStamped()
                pose_en_origen.header.frame_id = source_frame
                pose_en_origen.header.stamp = path.header.stamp
                pose_en_origen.pose = pose_stamped.pose

                pose_en_laser = do_transform_pose(pose_en_origen, transform)
                ruta_transformada.append((
                    pose_en_laser.pose.position.x,
                    pose_en_laser.pose.position.y,
                    pose_en_laser.pose.orientation  # Guardamos la orientación
                ))
                punto_anterior = punto_actual

            puntos_laser = []
            angulo = scan.angle_min
            for distancia in scan.ranges:
                if math.isfinite(distancia):
                    puntos_laser.append((
                        distancia * math.cos(angulo),
                        distancia * math.sin(angulo),
                    ))
                angulo += scan.angle_increment

            half_length = self.robot_length / 2.0
            half_width = self.robot_width / 2.0
            
            # PRE-CÁLCULO: El radio máximo desde el centro hasta la esquina del robot
            radio_peligro_maximo = math.hypot(half_length, half_width)

            for ruta_x, ruta_y, orientation in ruta_transformada:
                # Extraer el yaw desde el quaternion (Z rotation)
                q = orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                
                for laser_x, laser_y in puntos_laser:
                    # 1. FILTRO RÁPIDO: Distancia euclidiana simple
                    distancia_rapida = math.hypot(laser_x - ruta_x, laser_y - ruta_y)
                    
                    # 2. EVALUACIÓN PRECISA: Solo si el láser pasa el filtro
                    if distancia_rapida <= radio_peligro_maximo:
                        dist_real = self._distance_point_to_rect(
                            laser_x, laser_y,
                            ruta_x, ruta_y,
                            yaw,
                            half_length, half_width
                        )
                        if dist_real < 0.0:  # Punto dentro del rectángulo
                            node_logger.warning(
                                f"Colisión inminente en ({ruta_x:.2f}, {ruta_y:.2f}). Evadiendo..."
                            )
                            return py_trees.common.Status.FAILURE

            return py_trees.common.Status.SUCCESS

        except KeyError:
            return py_trees.common.Status.RUNNING

class AccionFalsa(py_trees.behaviour.Behaviour):
    def update(self): return py_trees.common.Status.RUNNING

# ==========================================
# 2. NODO ROS 2 ESTÁNDAR (Comunicaciones)
# ==========================================
class OrchestratorNode(Node):
    def __init__(self):
        super().__init__("bt_orchestrator")

        self.declare_parameter("tree_frequency_hz", 10.0)
        self.declare_parameter("lidar_topic", "/scan_filtered")
        self.declare_parameter("path_topic", "/smoothed_path") # Nuevo parámetro
        self.declare_parameter("lookahead_distance", 3.0)
        self.declare_parameter("robot_length", 0.5)       # Largo del robot Ackermann (m)
        self.declare_parameter("robot_width", 0.3)        # Ancho del robot Ackermann (m)

        self.tree_frequency = self.get_parameter("tree_frequency_hz").value
        self.lidar_topic = self.get_parameter("lidar_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.lookahead_distance = self.get_parameter("lookahead_distance").value
        self.robot_length = self.get_parameter("robot_length").value
        self.robot_width = self.get_parameter("robot_width").value

        self.blackboard = py_trees.blackboard.Client(name="Escritor_Sensores")
        self.blackboard.register_key(key="latest_scan", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="latest_path", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="tf_buffer", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="node_logger", access=py_trees.common.Access.WRITE)
        
        self.blackboard.latest_scan = None
        self.blackboard.latest_path = None # Inicializamos la ruta

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.blackboard.tf_buffer = self.tf_buffer
        self.blackboard.node_logger = self.get_logger()

        # Subscriber: LiDAR
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub_lidar = self.create_subscription(LaserScan, self.lidar_topic, self.scan_callback, qos_lidar)

        # Subscriber: Path (La ruta suele ser Reliable, no Best Effort)
        qos_path = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_path = self.create_subscription(Path, self.path_topic, self.path_callback, qos_path)

        # Árbol y Timer (igual que antes)
        self.root = self.create_tree()
        self.tree = py_trees_ros.trees.BehaviourTree(root=self.root, unicode_tree_debug=True)
        self.tree.setup(timeout=15, node=self)
        self.timer = self.create_timer(1.0 / self.tree_frequency, self.tick_tree)
        
        self.get_logger().info("Orchestrator Initialized. Esperando Laser y Path...")

    def scan_callback(self, msg):
        self.blackboard.latest_scan = msg

    def path_callback(self, msg):
        self.blackboard.latest_path = msg # Guardamos la ruta planeada

    def tick_tree(self):
        self.tree.tick()

    def create_tree(self):
        root = py_trees.composites.Selector(name="Orquestador", memory=False)
        rama_principal = py_trees.composites.Sequence(name="Seguir_Ruta", memory=False)
        
        # INYECCIÓN DE DEPENDENCIAS: Instanciamos el nodo pasándole los parámetros de ROS 2
        nodo_condicion = IsPathClear(
            name="Camino_Libre_?",
            lookahead_distance=self.lookahead_distance,
            robot_length=self.robot_length,
            robot_width=self.robot_width
        )
        
        rama_principal.add_children([nodo_condicion, AccionFalsa(name="PurePursuit")])
        root.add_children([rama_principal, AccionFalsa(name="FollowTheGap")])
        return root
    
def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.tree.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()