
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path, Odometry
import threading # ## NUEVO: Para manejar el teclado por separado

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
        self.current_waypoint_index = 0
        self.timer = None
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.odom_sub = self.create_subscription(Odometry, '/ekf/odometry', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.rviz_goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/current_active_path', 10)

        # ## PASO CLAVE: Crear el hilo que espera el Enter
        # target=self.wait_for_user significa "ejecuta esa función en paralelo"
        self.input_thread = threading.Thread(target=self.wait_for_user)
        self.input_thread.daemon = True # Para que el hilo se cierre si cierras el nodo
        self.input_thread.start()

        self.get_logger().info('✅ NODO LISTO.')
        self.get_logger().info('📍 Marca puntos en RViz y presiona ENTER aquí para graficar.')

    def wait_for_user(self):
        """ Esta función corre en segundo plano esperando el Enter """
        while rclpy.ok():
            input("\n >>> Presiona ENTER para graficar la trayectoria <<< \n")
            # Cuando el usuario presiona Enter, el código llega aquí:
            self.iniciar_secuencia()

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def rviz_goal_callback(self, msg):
        if self.state == "RECOLECTANDO":
            self.waypoints.append((msg.pose.position.x, msg.pose.position.y))
            self.get_logger().info(f'Punto {len(self.waypoints)} guardado.')

    def iniciar_secuencia(self):
        """ Lógica que antes estaba en el servicio, ahora la llamamos con Enter """
        if len(self.waypoints) < 1:
            self.get_logger().warn("¡No hay puntos guardados! Marca alguno en RViz primero.")
            return

        self.get_logger().info('🚀 Calculando ruta...')
        # Agregamos la posición actual del robot al inicio
        self.waypoints.insert(0, (self.robot_x, self.robot_y))
        
        self.full_path.poses = [] 
        self.current_waypoint_index = 0
        
        # Iniciamos el timer (graficación cada 1 segundo)
        self.timer = self.create_timer(1.0, self.timer_step_callback)

    def timer_step_callback(self):
        if self.current_waypoint_index >= len(self.waypoints) - 1:
            self.get_logger().info('🏁 Trayectoria visualizada.')
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
                self.full_path.poses.extend(res.plan.poses)
                self.full_path.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(self.full_path)
                self.current_waypoint_index += 1
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

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