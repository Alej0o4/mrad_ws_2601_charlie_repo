#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped
import numpy as np


class Control(Node): # Redefine node class
    def __init__(self):
        super().__init__("control") # Redefine node name

        # --- PARÁMETROS DE CONTROL (Tuning) ---
        # Kp: Qué tan fuerte reacciona al error actual
        self.declare_parameter('kp', 1.2) 
        # Kd: Qué tan fuerte reacciona al cambio del error (amortiguador)
        self.declare_parameter('kd', 0.4)         
        # Velocidad máxima lineal (m/s)
        self.declare_parameter('max_speed', 0.2) 
        # Límite máximo de giro (rad/s para diferencial)
        self.declare_parameter('max_steering', 0.8)


        # Quality of service (QoS) settings for the subscriber and publisher
        qos_profile = QoSProfile(depth=10)

        # Subscribers
        # Sucribe to the topic "/error" of type Float32, and define the callback function "callback"
        self.error_subscription = self.create_subscription(
            Float32,
            '/error',
            self.callback,
            qos_profile
        )

        # Publishers
        # Publish velocity commands to /cmd_vel_ctrl to reduce the error
        self.vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel_ctrl', qos_profile)

        # Variables de estado para el PD
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        self.get_logger().info("Control Node Initialized")
    
    def callback(self,msg):
        # 1. Leer parámetros (permitir tuning en tiempo real)
        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        max_speed = self.get_parameter('max_speed').value
        max_steering = self.get_parameter('max_steering').value

        # 2. Leer Error actual
        current_error = msg.data

        # 3. Calcular dt (Delta de tiempo)
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9 # Convertir a segundos

        if dt == 0:
            return # Evitar división por cero en la primera ejecución

        # 4. Cálculo del Controlador PD
        # Derivada: (Error_Actual - Error_Anterior) / Tiempo
        derivative = (current_error - self.prev_error) / dt
        
        # Salida del control (Giro)
        # Nota: El signo depende de cómo calculaste el error en el nodo anterior.
        # Si error positivo = lejos de la pared -> Necesitas girar HACIA la pared.
        angular_output = (kp * current_error) + (kd * derivative)

        # 5. Saturación (Slide 19)
        # Limitamos el giro para no exceder las capacidades físicas del robot
        angular_output = max(min(angular_output, max_steering), -max_steering)

        # 6. Velocidad Adaptativa (Slide 20)
        # "Si el error sube (curva), la velocidad baja".
        # Fórmula simple: Vel = Vel_Max / (1 + abs(Giro))
        # Esto hace que si el giro es 0, vas a max_speed. Si giras mucho, frenas.
        linear_velocity = max_speed / (1.0 + 0.25*np.exp(np.abs(angular_output)))

        # 7. Publicar Comando
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'base_link'
        cmd_msg.twist.linear.x = float(linear_velocity)
        cmd_msg.twist.angular.z = float(angular_output)

        
        self.vel_publisher.publish(cmd_msg)

        # 8. Actualizar variables previas
        self.prev_error = current_error
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = Control() # object definition (creation)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()