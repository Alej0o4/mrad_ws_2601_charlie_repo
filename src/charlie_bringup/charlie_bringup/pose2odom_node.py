import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
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
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vicon/pose', 
            self.vicon_callback,
            10)
        
        self.get_logger().info("Nodo VICON->Odom activado. Escuchando Pose...")

    def vicon_callback(self, msg):
        current_time = self.get_clock().now().to_msg()

        # --- CREAR Y PUBLICAR MENSAJE DE ODOMETRÍA ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        odom_msg.pose.pose.position = msg.pose.position
        odom_msg.pose.pose.orientation = msg.pose.orientation
        
        # Opcional: El VICON no da velocidades directamente. 
        # Si tu MPC requiere '/odom' con Twist (velocidades), tendríamos que derivar la posición aquí.
        
        self.odom_publisher.publish(odom_msg)

        # --- CREAR Y PUBLICAR TRANSFORMACIÓN (TF) ---
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

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