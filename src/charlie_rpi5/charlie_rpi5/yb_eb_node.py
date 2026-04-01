#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu,MagneticField
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data

from Rosmaster_Lib import Rosmaster



class TWIST_CMD_NODE(Node): # reemplazar YY por el numero de grupo
    def __init__(self):
        super().__init__("twist_cmd_node_charlie") # Redefine node name
        print("Node started")

        # Parámetros de mapeo y compensación para throttle
        self.declare_parameter('throttle.V_MAX', 0.5)    # Velocidad máxima de tu modelo
        self.declare_parameter('throttle.PWM_CENTER', 90.0)
        self.declare_parameter('throttle.PWM_MIN_FWD', 99.0)  # <- Valor donde apenas se mueve adelante
        self.declare_parameter('throttle.PWM_MAX_FWD', 132.5)  # <- Valor máximo adelante
        self.declare_parameter('throttle.PWM_MIN_REV', 83.4)   # <- Valor donde apenas se mueve en reversa
        self.declare_parameter('throttle.PWM_MAX_REV', 47.5)   # <- Valor máximo en reversa
        # Parámetros de mapeo para steering
        self.declare_parameter('steering.ANGLE_MAX', 0.5)
        self.declare_parameter('steering.PWM_MIN', 47.5)
        self.declare_parameter('steering.PWM_MAX', 132.5)

        # Parámetros de topicos
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_raw')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('voltage_topic', 'voltage')

        # Extracción de Parámetros
        self.V_MAX = self.get_parameter('throttle.V_MAX').value    # Velocidad máxima de tu modelo
        self.PWM_CENTER = self.get_parameter('throttle.PWM_CENTER').value
        self.PWM_MIN_FWD = self.get_parameter('throttle.PWM_MIN_FWD').value  # <- Valor donde apenas se mueve adelante
        self.PWM_MAX_FWD = self.get_parameter('throttle.PWM_MAX_FWD').value  # <- Valor máximo adelante
        self.PWM_MIN_REV = self.get_parameter('throttle.PWM_MIN_REV').value   # <- Valor donde apenas se mueve en reversa
        self.PWM_MAX_REV = self.get_parameter('throttle.PWM_MAX_REV').value   # <- Valor máximo en reversa

        self.ANGLE_MAX = self.get_parameter('steering.ANGLE_MAX').value
        self.STEERING_PWM_MIN = self.get_parameter('steering.PWM_MIN').value
        self.STEERING_PWM_MAX = self.get_parameter('steering.PWM_MAX').value

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.mag_topic = self.get_parameter('mag_topic').value
        self.voltage_topic = self.get_parameter('voltage_topic').value


        # create a topic subscriber
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.twist_subs = self.create_subscription(TwistStamped,self.cmd_vel_topic,self.twist_callback,1)

        # create a timer function to send msg
        timer_period = 0.1 # in [s]
        self.timer = self.create_timer(timer_period,self.timer_callback)


# ... dentro del __init__ ...
        self.imuPublisher = self.create_publisher(Imu, self.imu_topic, qos_profile_sensor_data)
        self.magPublisher = self.create_publisher(MagneticField,self.mag_topic,qos_profile_sensor_data)
        self.volPublisher = self.create_publisher(Float32,self.voltage_topic,qos_profile_sensor_data)

        #robot Create the Rosmaster object bot
        self.robot = Rosmaster()
        self.robot.create_receive_threading()

        self.cmd_vel = TwistStamped()

    def timer_callback(self):

        time_stamp = Clock().now()
        imu = Imu()
        mag = MagneticField()
        battery = Float32()

        print ("mag: ",self.robot.get_magnetometer_data())

        battery.data = self.robot.get_battery_voltage()
        ax, ay, az = self.robot.get_accelerometer_data()
        gx, gy, gz = self.robot.get_gyroscope_data()
        mx, my, mz = self.robot.get_magnetometer_data()
        mx = mx * 1.0
        my = my * 1.0
        mz = mz * 1.0

        # Publish gyroscope data
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
        

        # def twist variable

        throttle = self.compensate_and_map_velocity(self.cmd_vel.twist.linear.x)
        steering = self.map_and_clamp(self.cmd_vel.twist.angular.z, -self.ANGLE_MAX, self.ANGLE_MAX, self.STEERING_PWM_MIN, self.STEERING_PWM_MAX)
        msg = 'xd: {:.3f},{:.3f}, Thd: {:.3f},{:.3f}'.format(
            self.cmd_vel.twist.linear.x, throttle,
            self.cmd_vel.twist.angular.z, steering
        )
        self.get_logger().info(msg)

        self.robot.set_pwm_servo(1, throttle)
        self.robot.set_pwm_servo(2, steering)


    def twist_callback(self, data):
        msg = 'xd: {:.3f}, Thd: {:.3f}'.format(data.twist.linear.x, data.twist.angular.z)
        self.get_logger().info(msg)
        self.cmd_vel.twist.linear.x = data.twist.linear.x
        self.cmd_vel.twist.angular.z = data.twist.angular.z
    

    def compensate_and_map_velocity(self, v_cmd):
        # 1. Banda muerta de software (ignorar ruido minúsculo)
        if abs(v_cmd) < 0.01:
            return self.PWM_CENTER

        # 2. Saturación por seguridad (Clamp)
        v_cmd = max(-self.V_MAX, min(v_cmd, self.V_MAX))

        # 3. Mapeo por tramos (Saltando la zona muerta del hardware)
        if v_cmd > 0:
            # Mapear de [0.01 a 0.5] hacia [102.0 a 132.5]
            pwm = self.map_math(v_cmd, 0.0, self.V_MAX, self.PWM_MIN_FWD, self.PWM_MAX_FWD)
        else:
            # Mapear de [-0.01 a -0.5] hacia [76.0 a 47.5]
            pwm = self.map_math(v_cmd, -self.V_MAX, 0.0, self.PWM_MAX_REV, self.PWM_MIN_REV)

        return pwm

    def map_math(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def map_and_clamp(self, x, in_min, in_max, out_min, out_max):
        # 1. Saturar (Clamp) la entrada para no exceder límites físicos
        x = max(in_min, min(x, in_max))
        # 2. Mapear
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def kill(self):
        del self.robot
    

def main(args=None):
    rclpy.init(args=args)
    node = TWIST_CMD_NODE() # Definicion del objeto "node"
    
    # ejecucion ciclica 
    rclpy.spin(node)
    # finalizacion
    rclpy.shutdown()
    node.kill()

if __name__ == "__main__":
    main()