import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
import threading
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from visualization_msgs.msg import Marker, MarkerArray

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

        # Publicador de la ruta final
        self.path_pub = self.create_publisher(Path, '/current_active_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        # Declarar y leer parámetros ANTES de configurar los modos
        self.declare_parameter('use_rviz_clicks', True)
        self.declare_parameter(
            'static_waypoints',
            [0.0],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
            
        )

        self._use_clicks = self.get_parameter('use_rviz_clicks').get_parameter_value().bool_value
        self._num_laps = self.get_parameter('num_laps').get_parameter_value().integer_value
        raw_waypoints = self.get_parameter('static_waypoints').get_parameter_value().double_array_value

        # Delegamos toda la lógica condicional a esta función
        self.setup_waypoint_mode(raw_waypoints)

        self.get_logger().info('✅ GESTOR DE WAYPOINTS LISTO.')

    def wait_for_user(self):
        while rclpy.ok():
            entrada = input("\n >>> ENTER para iniciar rutas | Escribe 'z' + ENTER para borrar el último punto <<< \n")
            if self.state == "RECOLECTANDO":
                if entrada.strip().lower() == 'z':
                    if len(self.waypoints) > 0:
                        borrado = self.waypoints.pop()
                        self.get_logger().info(f'Punto eliminado: ({borrado[0]:.2f}, {borrado[1]:.2f})')
                        self.publish_markers() # Actualizamos RViz
                    else:
                        self.get_logger().warn("No hay puntos para borrar.")
                else:
                    self.iniciar_secuencia()
            else:
                self.get_logger().warn("El nodo ya está calculando o finalizó. Reinicia para otra ruta.")

    def rviz_goal_callback(self, msg):
        if self.state == "RECOLECTANDO":
            self.waypoints.append((msg.pose.position.x, msg.pose.position.y))
            self.get_logger().info(f'Punto {len(self.waypoints)} guardado: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
            self.publish_markers()

    def iniciar_secuencia(self):
        if len(self.waypoints) < 1:
            self.get_logger().warn("¡No hay puntos! Marca al menos 1 destino en RViz.")
            return
        
        if self._num_laps > 1:
            self.get_logger().info(f'🔄 Generando trayectoria para {self._num_laps} vueltas...')
            # Guardamos los puntos originales y los repetimos
            puntos_originales = list(self.waypoints)
            for _ in range(self._num_laps - 1):
                self.waypoints.extend(puntos_originales)

        self.get_logger().info('🚀 Cerrando recolección e iniciando cálculos en cadena...')
        self.state = "CALCULANDO"
        self.full_path.poses = []
        self.current_req_index = 0
        
        # Disparamos la primera petición de la cadena
        self.request_next_segment()

    def request_next_segment(self):
        """ Envía la petición al ARA*. Si es el índice 0, el Start va vacío. """
        if self.current_req_index >= len(self.waypoints):
            self.state = "FINALIZADO"
            self.get_logger().info('🏁 ¡Ruta global calculada y concatenada exitosamente!')
            # Opcional: Aquí enviarías self.full_path al controlador local (Nav2 o Pure Pursuit)
            return

        req = GetPlan.Request()
        
        # LÓGICA DEL PUNTO DE INICIO (Start)
        if self.current_req_index == 0:
            # Magia pura: Dejamos el frame_id vacío. 
            # El ARA* detectará esto y usará el TF del robot automáticamente.
            req.start.header.frame_id = "" 
        else:
            # Para los siguientes segmentos, el inicio es el waypoint anterior
            p_inicio = self.waypoints[self.current_req_index - 1]
            req.start.header.frame_id = 'map'
            req.start.pose.position.x = p_inicio[0]
            req.start.pose.position.y = p_inicio[1]

        # LÓGICA DEL DESTINO (Goal)
        p_destino = self.waypoints[self.current_req_index]
        req.goal.header.frame_id = 'map'
        req.goal.pose.position.x = p_destino[0]
        req.goal.pose.position.y = p_destino[1]

        self.get_logger().info(f'Pidiendo tramo hacia Waypoint {self.current_req_index + 1}...')
        
        # Llamada Asíncrona. Cuando ARA* responda, se ejecuta service_response_callback
        future = self.cli.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            res = future.result()
            if res.plan.poses:
                # Si no es el primer segmento, borramos el primer punto de la respuesta
                # para evitar que el waypoint de conexión quede duplicado en la lista final.
                if self.current_req_index > 0 and len(res.plan.poses) > 0:
                    poses_to_add = res.plan.poses[1:]
                else:
                    poses_to_add = res.plan.poses
                
                self.full_path.poses.extend(poses_to_add)
                self.full_path.header.stamp = self.get_clock().now().to_msg()
                
                # Visualizamos cómo va creciendo la ruta
                self.path_pub.publish(self.full_path)
                
                # Incrementamos índice y pedimos el SIGUIENTE tramo (Efecto dominó)
                self.current_req_index += 1
                self.request_next_segment()
            else:
                self.get_logger().error("ARA* devolvió una ruta vacía. Abortando cadena.")
                self.state = "FINALIZADO"
                
        except Exception as e:
            self.get_logger().error(f'Error en el servicio: {e}')

    def auto_start_callback(self):
        # Destruimos este timer porque solo queremos que corra UNA vez
        if hasattr(self, 'auto_timer'):
            self.auto_timer.destroy()
            
        self.get_logger().info("¡Arrancando secuencia predefinida!")
        self.iniciar_secuencia()

    def setup_waypoint_mode(self, raw_waypoints):
        """
        Configura el modo de operación del nodo (Automático por YAML o Interactivo por RViz)
        e inicializa los recursos estrictamente necesarios.
        """
        if not self._use_clicks:
            self.get_logger().info("Modo Automático: Cargando puntos desde YAML...")
            
            for i in range(0, len(raw_waypoints), 2):
                if i + 1 < len(raw_waypoints):
                    self.waypoints.append((raw_waypoints[i], raw_waypoints[i+1]))
            
            self.get_logger().info(f"Se cargaron {len(self.waypoints)} puntos fijos.")
            self.publish_markers()

            self.state = "CALCULANDO"
            # No creamos hilos de teclado ni nos suscribimos a RViz. Ahorro de recursos total.
            self.auto_timer = self.create_timer(2.0, self.auto_start_callback)
            
        else:
            self.get_logger().info("Modo Interactivo: Esperando clics en RViz...")
            self.get_logger().info('📍 Marca puntos en RViz y presiona ENTER en la terminal para unirlos.')
            
            # 1. Solo nos suscribimos a los clics si realmente los vamos a usar
            self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.rviz_goal_callback, 10)
            
            # 2. Solo levantamos el hilo del teclado si esperamos interacción humana
            self.input_thread = threading.Thread(target=self.wait_for_user)
            self.input_thread.daemon = True
            self.input_thread.start()

    def publish_markers(self):
        marker_array = MarkerArray()

        # Marcador especial para borrar los anteriores antes de redibujar
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, wp in enumerate(self.waypoints):
            # 1. Esfera en la posición del waypoint
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
            m.scale.x = 0.4  # Tamaño de la esfera
            m.scale.y = 0.4
            m.scale.z = 0.4
            m.color.r = 0.0  # Color Cyan
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 0.8  # Transparencia
            marker_array.markers.append(m)

            # 2. Texto flotante con el número
            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = self.get_clock().now().to_msg()
            t.ns = 'waypoints_text'
            t.id = (i * 2) + 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = wp[0]
            t.pose.position.y = wp[1]
            t.pose.position.z = 0.5  # Medio metro arriba de la esfera
            t.scale.z = 0.5        # Tamaño de la letra
            t.color.r = 1.0        # Texto blanco
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