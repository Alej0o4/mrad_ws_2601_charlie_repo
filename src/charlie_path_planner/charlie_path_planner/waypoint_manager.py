import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
import threading

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        self.cli = self.create_client(GetPlan, '/get_plan')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servidor ARA*...')
        
        self.waypoints = []
        self.full_path = Path()
        self.full_path.header.frame_id = 'map'
        
        self.state = "RECOLECTANDO" # Estados: RECOLECTANDO, CALCULANDO, FINALIZADO
        self.current_req_index = 0

        # Subscripciones y Publicadores
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.rviz_goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/current_active_path', 10)

        # Hilo de teclado
        self.input_thread = threading.Thread(target=self.wait_for_user)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.get_logger().info('✅ GESTOR DE WAYPOINTS LISTO.')
        self.get_logger().info('📍 Marca puntos en RViz y presiona ENTER en la terminal para unirlos.')

    def wait_for_user(self):
        while rclpy.ok():
            input("\n >>> Presiona ENTER para iniciar la secuenciación de rutas <<< \n")
            if self.state == "RECOLECTANDO":
                self.iniciar_secuencia()
            else:
                self.get_logger().warn("El nodo ya está calculando o finalizó. Reinicia el nodo para otra ruta.")

    def rviz_goal_callback(self, msg):
        if self.state == "RECOLECTANDO":
            self.waypoints.append((msg.pose.position.x, msg.pose.position.y))
            self.get_logger().info(f'Punto {len(self.waypoints)} guardado: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def iniciar_secuencia(self):
        if len(self.waypoints) < 1:
            self.get_logger().warn("¡No hay puntos! Marca al menos 1 destino en RViz.")
            return

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