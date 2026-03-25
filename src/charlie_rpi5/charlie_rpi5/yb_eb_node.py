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
        super().__init__("twist_cmd_node_charlie_2402") # Redefine node name
        print("Node started")

        # create a topic subscriber
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.twist_subs = self.create_subscription(TwistStamped,'/cmd_vel_raw',self.twist_callback,1)

        # create a timer function to send msg
        timer_period = 0.1 # in [s]
        self.timer = self.create_timer(timer_period,self.timer_callback)


# ... dentro del __init__ ...
        self.imuPublisher = self.create_publisher(Imu, "/imu/data_raw", qos_profile_sensor_data)
        self.magPublisher = self.create_publisher(MagneticField,"/imu/mag",qos_profile_sensor_data)
        self.volPublisher = self.create_publisher(Float32,"voltage",qos_profile_sensor_data)

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

        throttle = self.map_and_clamp(self.cmd_vel.twist.linear.x, -0.5, +0.5, 80.0, 100.0)
        steering = self.map_and_clamp(self.cmd_vel.twist.angular.z, -0.5, +0.5, 47.5, 132.5)
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