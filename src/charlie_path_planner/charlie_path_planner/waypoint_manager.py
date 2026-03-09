import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from std_srvs.srv import Trigger

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        self.cli = self.create_client(GetPlan, '/get_plan')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servidor ARA*...')
        
        # --- MEMORIA DE RUTA ---
        self.waypoints = []
        self.full_path = Path() # Aquí acumularemos TODO
        self.full_path.header.frame_id = 'map'
        
        self.state = "RECOLECTANDO"
        self.current_waypoint_index = 0
        self.timer = None

        # --- COMUNICACIÓN ---
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.rviz_goal_callback, 10)
        self.start_srv = self.create_service(Trigger, '/empezar_secuencia', self.empezar_secuencia_callback)
        self.path_pub = self.create_publisher(Path, '/current_active_path', 10)

        self.get_logger().info('✅ MODO ACUMULATIVO LISTO.')

    def rviz_goal_callback(self, msg):
        if self.state == "RECOLECTANDO":
            self.waypoints.append((msg.pose.position.x, msg.pose.position.y))
            self.get_logger().info(f'📍 Punto {len(self.waypoints)} guardado.')

    def empezar_secuencia_callback(self, request, response):
        if len(self.waypoints) < 2:
            response.success = False
            return response

        self.state = "VISUALIZANDO"
        self.full_path.poses = [] # Limpiamos por si hubo una ruta previa
        
        # Bajamos el tiempo a 0.2s para que se dibuje casi instantáneo
        self.timer = self.create_timer(0.2, self.timer_step_callback)
        
        response.success = True
        return response

    def timer_step_callback(self):
        if self.current_waypoint_index >= len(self.waypoints) - 1:
            self.get_logger().info('🏁 Trayectoria completa visualizada.')
            self.timer.destroy()
            return

        p_inicio = self.waypoints[self.current_waypoint_index]
        p_destino = self.waypoints[self.current_waypoint_index + 1]

        req = GetPlan.Request()
        req.start.header.frame_id = 'map'
        req.start.pose.position.x = p_inicio[0]
        req.start.pose.position.y = p_inicio[1]
        req.goal.header.frame_id = 'map'
        req.goal.pose.position.x = p_destino[0]
        req.goal.pose.position.y = p_destino[1]

        future = self.cli.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            res = future.result()
            if res.plan.poses:
                # CLAVE: En lugar de reemplazar, EXTENDEMOS la lista actual
                self.full_path.poses.extend(res.plan.poses)
                
                # Publicamos el acumulado
                self.full_path.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(self.full_path)
                
                self.current_waypoint_index += 1
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()