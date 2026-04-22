import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class ViconToOdomNode(Node):
    def __init__(self):
        super().__init__('vicon_to_odom_node')
        
        # 1. Publicador de Odometría
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # 2. Broadcaster de TF
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 3. Suscriptor al VICON (Ajusta '/vicon/pose' al tópico real de tu lab)
        self.t = TransformStamped()
        self.t.header.frame_id = 'odom'
        self.t.child_frame_id = 'base_link'
        self.subscription = self.create_subscription(
            Pose,
            '/robot1/pose', 
            self.vicon_callback,
            10)
        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'
        
        self.timer  = self.create_timer(0.1, self.timer_callback) # Opcional: para publicar TF a una tasa fija
        
        self.get_logger().info("Nodo VICON->Odom activado. Escuchando Pose...")

    def timer_callback(self):
        # Publicar la transformación TF a una tasa fija (opcional)
        self.tf_broadcaster.sendTransform(self.t)
        self.odom_publisher.publish(self.odom_msg)

    
    def vicon_callback(self, msg):
        current_time = self.get_clock().now().to_msg()


        # --- CREAR Y PUBLICAR MENSAJE DE ODOMETRÍA ---
        self.odom_msg.header.stamp = current_time
        
        # Convertir de mm a m
        self.odom_msg.pose.pose.position.x = msg.position.x / 1000.0
        self.odom_msg.pose.pose.position.y = msg.position.y / 1000.0
        self.odom_msg.pose.pose.position.z = msg.position.z / 1000.0
        self.odom_msg.pose.pose.orientation = msg.orientation
        
        # Opcional: El VICON no da velocidades directamente. 
        # Si tu MPC requiere '/odom' con Twist (velocidades), tendríamos que derivar la posición aquí.
    
        self.t.header.stamp = current_time

        # --- CREAR Y PUBLICAR TRANSFORMACIÓN (TF) ---
        self.t.transform.translation.x = msg.position.x/1000.0  # Convertir de mm a m si es necesario
        self.t.transform.translation.y = msg.position.y/1000.0
        self.t.transform.translation.z = msg.position.z/1000.0

        self.t.transform.rotation = msg.orientation

def main(args=None):
    rclpy.init(args=args)
    node = ViconToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()