#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import numpy as np

class SysIdExcitationNode(Node):
    def __init__(self):
        super().__init__('sysid_excitation_node')

        # --- PARÁMETROS DE LISTAS (Keyframes) ---
        self.declare_parameter('t_keys', [0.0, 3.0, 6.0])
        self.declare_parameter('v_keys', [0.0, 0.5, 0.0])
        self.declare_parameter('w_keys', [0.0, 0.0, 0.0])
        self.declare_parameter('interpolation_mode', 'linear') # 'linear' o 'step'

        self.t_keys = self.get_parameter('t_keys').value
        self.v_keys = self.get_parameter('v_keys').value
        self.w_keys = self.get_parameter('w_keys').value
        self.mode = self.get_parameter('interpolation_mode').value

        # Validar que las listas tengan el mismo tamaño
        if not (len(self.t_keys) == len(self.v_keys) == len(self.w_keys)):
            self.get_logger().error("¡Las listas t_keys, v_keys y w_keys deben tener el mismo tamaño!")
            raise ValueError("Arrays size mismatch")

        # --- PUBLICADOR Y CONTROL DE TIEMPO ---
        # Publicamos al tópico que el twist_mux está escuchando
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_ctrl', 10)
        
        # Frecuencia de 50Hz (0.02s) para enviar comandos continuos y suaves
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.start_time = None
        
        self.get_logger().info(f"Nodo SysID Iniciado. Modo: {self.mode.upper()}. Esperando primer ciclo...")

    def timer_callback(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        # Tiempo transcurrido en segundos
        t_elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        msg = TwistStamped()

        # Si ya pasamos el último tiempo de la lista, detenemos el robot por seguridad
        if t_elapsed > self.t_keys[-1]:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info("Secuencia de excitación terminada. Vehículo detenido.", once=True)
            return

        # --- LÓGICA DE INTERPOLACIÓN ---
        if self.mode == 'linear':
            # numpy.interp interpola linealmente (Perfecto para RAMPAS - Sesión B)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = float(np.interp(t_elapsed, self.t_keys, self.v_keys))
            msg.twist.angular.z = float(np.interp(t_elapsed, self.t_keys, self.w_keys))
        
        elif self.mode == 'step':
            # Busca el índice del tiempo actual y mantiene el valor anterior (Perfecto para ESCALONES - Sesión A)
            # side='right' asegura que el escalón salte exactamente en el segundo indicado
            idx = np.searchsorted(self.t_keys, t_elapsed, side='right') - 1
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = float(self.v_keys[idx])
            msg.twist.angular.z = float(self.w_keys[idx])

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SysIdExcitationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()