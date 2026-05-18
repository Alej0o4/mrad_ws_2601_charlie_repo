import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, ReliabilityPolicy

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        self.cli = self.create_client(GetPlan, '/get_plan')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servidor ARA*...')
        
        self.waypoints = []
        self.full_path = Path()
        self.full_path.header.frame_id = 'map'
        
        self.state = "RECOLECTANDO" 
        self.current_req_index = 0
        self.declare_parameter('num_laps', 1)

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.path_pub = self.create_publisher(Path, '/current_active_path', latched_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        self.declare_parameter('use_rviz_clicks', True)
        self.declare_parameter(
            'static_waypoints',
            [0.0],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
        )

        self._use_clicks = self.get_parameter('use_rviz_clicks').get_parameter_value().bool_value
        self._num_laps = self.get_parameter('num_laps').get_parameter_value().integer_value
        raw_waypoints = self.get_parameter('static_waypoints').get_parameter_value().double_array_value

        self.setup_waypoint_mode(raw_waypoints)
        self.get_logger().info('✅ GESTOR DE WAYPOINTS LISTO.')

    def setup_waypoint_mode(self, raw_waypoints):
        if not self._use_clicks:
            self.get_logger().info("Modo Automático: Cargando puntos desde YAML...")
            for i in range(0, len(raw_waypoints), 2):
                if i + 1 < len(raw_waypoints):
                    self.waypoints.append((raw_waypoints[i], raw_waypoints[i+1]))
            
            self.get_logger().info(f"Se cargaron {len(self.waypoints)} puntos fijos.")
            self.publish_markers()
            self.state = "CALCULANDO"
            self.auto_timer = self.create_timer(2.0, self.auto_start_callback)
            
        else:
            self.get_logger().info("--- MODO INTERACTIVO RVIZ ACTIVADO ---")
            self.get_logger().info("1. Usa '2D Goal Pose' para añadir waypoints.")
            self.get_logger().info("2. Usa '2D Pose Estimate' para DESHACER el último punto.")
            self.get_logger().info("3. Usa 'Publish Point' para FINALIZAR e iniciar cálculos.")
            
            # Suscripciones mágicas de RViz
            self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.rviz_goal_callback, 10)
            self.undo_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.rviz_undo_callback, 10)
            self.start_sub = self.create_subscription(PointStamped, '/clicked_point', self.rviz_start_callback, 10)

    # ---------------- CALLBACKS DE RVIZ ----------------

    def rviz_goal_callback(self, msg):
        """ Equivale a hacer clic en el mapa """
        if self.state == "RECOLECTANDO":
            self.waypoints.append((msg.pose.position.x, msg.pose.position.y))
            self.get_logger().info(f'📍 Punto {len(self.waypoints)} guardado: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
            self.publish_markers()

    def rviz_undo_callback(self, msg):
        """ Equivale a presionar 'z' + ENTER """
        if self.state == "RECOLECTANDO":
            if len(self.waypoints) > 0:
                borrado = self.waypoints.pop()
                self.get_logger().info(f'🗑️ Punto eliminado: ({borrado[0]:.2f}, {borrado[1]:.2f})')
                self.publish_markers()
            else:
                self.get_logger().warn("No hay puntos para borrar.")

    def rviz_start_callback(self, msg):
        """ Equivale a presionar ENTER puro """
        if self.state == "RECOLECTANDO":
            self.get_logger().info("🎯 Comando de INICIO recibido desde RViz.")
            self.iniciar_secuencia()
        else:
            self.get_logger().warn("El nodo ya calculó la ruta. Reinicia el nodo para otro circuito.")

    # ---------------- LOGICA CORE ----------------

    def iniciar_secuencia(self):
        if len(self.waypoints) < 1:
            self.get_logger().warn("¡No hay puntos! Usa '2D Goal Pose' para marcar al menos 1 destino.")
            return
        
        if self._num_laps > 1:
            self.get_logger().info(f'🔄 Generando trayectoria para {self._num_laps} vueltas...')
            puntos_originales = list(self.waypoints)
            for _ in range(self._num_laps - 1):
                self.waypoints.extend(puntos_originales)

        self.get_logger().info('🚀 Cerrando recolección e iniciando cálculos en cadena...')
        self.state = "CALCULANDO"
        self.full_path.poses = []
        self.current_req_index = 0
        
        self.request_next_segment()

    def request_next_segment(self):
        # (Sin cambios, tu lógica aquí está perfecta)
        if self.current_req_index >= len(self.waypoints):
            self.state = "FINALIZADO"
            self.get_logger().info('🏁 ¡Ruta global calculada y concatenada exitosamente!')
            return

        req = GetPlan.Request()
        if self.current_req_index == 0:
            req.start.header.frame_id = "" 
        else:
            p_inicio = self.waypoints[self.current_req_index - 1]
            req.start.header.frame_id = 'map'
            req.start.pose.position.x = p_inicio[0]
            req.start.pose.position.y = p_inicio[1]

        p_destino = self.waypoints[self.current_req_index]
        req.goal.header.frame_id = 'map'
        req.goal.pose.position.x = p_destino[0]
        req.goal.pose.position.y = p_destino[1]

        self.get_logger().info(f'Pidiendo tramo hacia Waypoint {self.current_req_index + 1}...')
        
        future = self.cli.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        # (Sin cambios, tu lógica aquí está perfecta)
        try:
            res = future.result()
            if res.plan.poses:
                if self.current_req_index > 0 and len(res.plan.poses) > 0:
                    poses_to_add = res.plan.poses[1:]
                else:
                    poses_to_add = res.plan.poses
                
                self.full_path.poses.extend(poses_to_add)
                self.full_path.header.stamp = self.get_clock().now().to_msg()
                
                self.path_pub.publish(self.full_path)
                self.current_req_index += 1
                self.request_next_segment()
            else:
                self.get_logger().error("ARA* devolvió una ruta vacía. Abortando cadena.")
                self.state = "FINALIZADO"
                
        except Exception as e:
            self.get_logger().error(f'Error en el servicio: {e}')

    def auto_start_callback(self):
        if hasattr(self, 'auto_timer'):
            self.auto_timer.destroy()
        self.get_logger().info("¡Arrancando secuencia predefinida!")
        self.iniciar_secuencia()

    def publish_markers(self):
        # (Sin cambios, tu lógica de visualización está genial)
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, wp in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'waypoints_spheres'
            m.id = i * 2
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp[0]
            m.pose.position.y = wp[1]
            m.pose.position.z = 0.0
            m.scale.x = 0.4 
            m.scale.y = 0.4
            m.scale.z = 0.4
            m.color.r = 0.0  
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 0.8  
            marker_array.markers.append(m)

            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = self.get_clock().now().to_msg()
            t.ns = 'waypoints_text'
            t.id = (i * 2) + 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = wp[0]
            t.pose.position.y = wp[1]
            t.pose.position.z = 0.5 
            t.scale.z = 0.5        
            t.color.r = 1.0        
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 1.0
            t.text = str(i + 1)
            marker_array.markers.append(t)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()