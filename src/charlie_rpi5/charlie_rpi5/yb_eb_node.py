#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, MagneticField
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data

from Rosmaster_Lib import Rosmaster
import math

class TWIST_CMD_NODE(Node): 
    def __init__(self):
        super().__init__("twist_cmd_node_charlie") 
        print("Node started")

        # Parámetros de mapeo y compensación para throttle
        self.declare_parameter('throttle.V_MAX', 1.0)    
        self.declare_parameter('throttle.PWM_CENTER', 90.0)
        self.declare_parameter('throttle.PWM_MIN_FWD', 99.0)  
        self.declare_parameter('throttle.PWM_MAX_FWD', 120.0)  
        self.declare_parameter('throttle.PWM_MIN_REV', 89.9)   
        self.declare_parameter('throttle.PWM_MAX_REV', 60.0)   
        
        # Parámetros de mapeo para steering
        self.declare_parameter('steering.ANGLE_MAX', 0.5)
        self.declare_parameter('steering.PWM_MIN', 47.5)
        self.declare_parameter('steering.PWM_MAX', 132.5)

        # [NEW] Gobernador de Hardware (Porcentajes de límite 0.0 - 1.0)
        self.declare_parameter('throttle.LIMIT_PERCENT', 1.0)
        self.declare_parameter('steering.LIMIT_PERCENT', 1.0)

        # Parámetro de seguridad para la batería LiPo 3S
        self.declare_parameter('min_battery_voltage', 11.7)

        # Parámetros de topicos
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_aebs')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('voltage_topic', 'voltage')

        # Extracción de Parámetros
        self.V_MAX = self.get_parameter('throttle.V_MAX').value    
        self.PWM_CENTER = self.get_parameter('throttle.PWM_CENTER').value
        self.PWM_MIN_FWD = self.get_parameter('throttle.PWM_MIN_FWD').value  
        self.PWM_MAX_FWD = self.get_parameter('throttle.PWM_MAX_FWD').value  
        self.PWM_MIN_REV = self.get_parameter('throttle.PWM_MIN_REV').value   
        self.PWM_MAX_REV = self.get_parameter('throttle.PWM_MAX_REV').value   

        self.ANGLE_MAX = self.get_parameter('steering.ANGLE_MAX').value
        self.STEERING_PWM_MIN = self.get_parameter('steering.PWM_MIN').value
        self.STEERING_PWM_MAX = self.get_parameter('steering.PWM_MAX').value

        # [NEW] Extracción de límites porcentuales
        self.V_LIMIT_PCT = self.get_parameter('throttle.LIMIT_PERCENT').value
        self.STEER_LIMIT_PCT = self.get_parameter('steering.LIMIT_PERCENT').value

        self.min_voltage = self.get_parameter('min_battery_voltage').value

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.mag_topic = self.get_parameter('mag_topic').value
        self.voltage_topic = self.get_parameter('voltage_topic').value

        self.twist_subs = self.create_subscription(TwistStamped, self.cmd_vel_topic, self.twist_callback, 1)

        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.imuPublisher = self.create_publisher(Imu, self.imu_topic, qos_profile_sensor_data)
        self.magPublisher = self.create_publisher(MagneticField, self.mag_topic, qos_profile_sensor_data)
        self.volPublisher = self.create_publisher(Float32, self.voltage_topic, qos_profile_sensor_data)

        self.robot = Rosmaster()
        self.robot.create_receive_threading()

        self.cmd_vel = TwistStamped()

    def timer_callback(self):
        time_stamp = Clock().now()
        imu = Imu()
        mag = MagneticField()
        battery = Float32()

        battery.data = self.robot.get_battery_voltage()
        ax, ay, az = self.robot.get_accelerometer_data()
        gx, gy, gz = self.robot.get_gyroscope_data()
        mx, my, mz = self.robot.get_magnetometer_data()
        mx = mx * 1.0
        my = my * 1.0
        mz = mz * 1.0

        if 5.0 < battery.data < self.min_voltage:
            self.get_logger().warn(f"¡ALERTA BATERÍA BAJA! Voltaje actual: {battery.data:.2f}V (Umbral: {self.min_voltage}V)", throttle_duration_sec=2.0)
            try:
                self.robot.set_beep(100) 
            except AttributeError:
                pass 

        imu.header.stamp = time_stamp.to_msg()
        imu.header.frame_id = 'imu_link'
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        mag.header.stamp = time_stamp.to_msg()
        mag.header.frame_id = 'imu_link'
        mag.magnetic_field.x = mx
        mag.magnetic_field.y = my
        mag.magnetic_field.z = mz

        self.imuPublisher.publish(imu)
        self.magPublisher.publish(mag)
        self.volPublisher.publish(battery)
        
        v = self.cmd_vel.twist.linear.x
        omega = self.cmd_vel.twist.angular.z
        
        delta_efectivo = self.cmd_to_delta(v, omega)

        # [NEW] Aplicamos el gobernador de dirección
        max_allowed_angle = self.ANGLE_MAX * self.STEER_LIMIT_PCT
        delta_efectivo = max(-max_allowed_angle, min(delta_efectivo, max_allowed_angle))

        throttle = self.compensate_and_map_velocity(v)
        steering = self.map_and_clamp(delta_efectivo, -self.ANGLE_MAX, self.ANGLE_MAX, self.STEERING_PWM_MIN, self.STEERING_PWM_MAX)

        self.robot.set_pwm_servo(1, throttle)
        self.robot.set_pwm_servo(2, steering)
        # self.get_logger().info(f"Comando de angulo enviado: {delta_efectivo}, PWM asociado: {steering}")
        # self.get_logger().info(f"Comando de throttle: {throttle}, PWM asociado: {v}")

    def cmd_to_delta(self, v, omega):
        L = 0.257 
        if abs(v) > 0.05:
            return math.atan((omega * L) / v)
        else:
            return math.atan((omega * L) / 0.05)

    def twist_callback(self, data):
        self.cmd_vel.twist.linear.x = data.twist.linear.x
        self.cmd_vel.twist.angular.z = data.twist.angular.z
    
    def compensate_and_map_velocity(self, v_cmd):
        if abs(v_cmd) < 0.01:
            return self.PWM_CENTER

        # [NEW] Aplicamos el gobernador de tracción
        max_allowed_v = self.V_MAX * self.V_LIMIT_PCT
        v_cmd = max(-max_allowed_v, min(v_cmd, max_allowed_v))

        if v_cmd > 0:
            pwm = self.map_math(v_cmd, 0.0, self.V_MAX, self.PWM_MIN_FWD, self.PWM_MAX_FWD)
        else:
            pwm = self.map_math(v_cmd, -self.V_MAX, 0.0, self.PWM_MAX_REV, self.PWM_MIN_REV)

        return pwm

    def map_math(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def map_and_clamp(self, x, in_min, in_max, out_min, out_max):
        x = max(in_min, min(x, in_max))
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def destroy_node(self):
        self.get_logger().info("Interceptando apagado (Ctrl+C). Frenando robot por seguridad...")
        try:
            self.robot.set_pwm_servo(1, self.PWM_CENTER)
            self.robot.set_pwm_servo(2, 90.0)
            self.robot.set_beep(200)
        except Exception as e:
            self.get_logger().error(f"Error al frenar en el apagado: {e}")
        
        super().destroy_node()

    def kill(self):
        del self.robot
    
def main(args=None):
    rclpy.init(args=args)
    node = TWIST_CMD_NODE() 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Señal de interrupción de teclado recibida.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.kill()

if __name__ == "__main__":
    main()