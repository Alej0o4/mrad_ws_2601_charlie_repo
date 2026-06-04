#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32, String

from Rosmaster_Lib import Rosmaster


class YbEbExternalEscNode(Node):
    def __init__(self):
        super().__init__('yb_eb_external_esc_node')

        # --- Parameters ---
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_aebs')
        self.declare_parameter('steering_cmd_topic', '/servo/steering_cmd')
        self.declare_parameter('esc_state_topic', '/esc/cfoc_state')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('voltage_topic', 'voltage')

        self.declare_parameter('wheelbase', 0.257)
        self.declare_parameter('min_linear_speed_for_delta', 0.05)

        self.declare_parameter('steering.LIMIT_PERCENT', 1.0)
        self.declare_parameter('steering.PWM_MIN', 50.0)
        self.declare_parameter('steering.PWM_MAX', 130.0)
        self.declare_parameter('steering.PWM_CENTER', 90.0)
        self.declare_parameter('steering.SERVO_CHANNEL', 2)

        self.declare_parameter('closed_loop_state_value', 'closed_loop')
        self.declare_parameter('buzzer_enabled', True)
        self.declare_parameter('buzzer_pulse_count', 2)
        self.declare_parameter('buzzer_pulse_ms', 100)
        self.declare_parameter('buzzer_pause_ms', 120)

        self.declare_parameter('min_battery_voltage', 11.7)

        # --- Calibration parameters for mapping normalized cmd to physical speed ---
        self.declare_parameter('deadzone_threshold', 0.1)
        self.declare_parameter('cmd_min', 0.1)
        self.declare_parameter('cmd_max', 0.2)
        self.declare_parameter('speed_min_ms', 1.3)
        self.declare_parameter('speed_max_ms', 1.4)
        # Default max steering angle (rad). Adjust to your servo maximum if known.
        self.declare_parameter('max_steer_angle_rad', 1.0)
        # --- Parameter values ---
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.steering_cmd_topic = self.get_parameter('steering_cmd_topic').value
        self.esc_state_topic = self.get_parameter('esc_state_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.mag_topic = self.get_parameter('mag_topic').value
        self.voltage_topic = self.get_parameter('voltage_topic').value

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.min_linear_speed_for_delta = float(self.get_parameter('min_linear_speed_for_delta').value)

        self.steering_limit_percent = float(self.get_parameter('steering.LIMIT_PERCENT').value)
        self.steering_pwm_min = float(self.get_parameter('steering.PWM_MIN').value)
        self.steering_pwm_max = float(self.get_parameter('steering.PWM_MAX').value)
        self.steering_pwm_center = float(self.get_parameter('steering.PWM_CENTER').value)
        self.steering_servo_channel = int(self.get_parameter('steering.SERVO_CHANNEL').value)

        self.closed_loop_state_value = str(self.get_parameter('closed_loop_state_value').value).strip().lower()
        self.buzzer_enabled = bool(self.get_parameter('buzzer_enabled').value)
        self.buzzer_pulse_count = max(1, int(self.get_parameter('buzzer_pulse_count').value))
        self.buzzer_pulse_ms = max(1, int(self.get_parameter('buzzer_pulse_ms').value))
        self.buzzer_pause_ms = max(0, int(self.get_parameter('buzzer_pause_ms').value))

        self.min_battery_voltage = float(self.get_parameter('min_battery_voltage').value)

        # Calibration parameter values
        self.deadzone_threshold = float(self.get_parameter('deadzone_threshold').value)
        self.cmd_min = float(self.get_parameter('cmd_min').value)
        self.cmd_max = float(self.get_parameter('cmd_max').value)
        self.speed_min_ms = float(self.get_parameter('speed_min_ms').value)
        self.speed_max_ms = float(self.get_parameter('speed_max_ms').value)
        self.max_steer_angle_rad = float(self.get_parameter('max_steer_angle_rad').value)
        # --- QoS ---
        sensor_qos = qos_profile_sensor_data

        # --- ROS interfaces ---
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, self.cmd_vel_topic, self._cmd_vel_callback, 10)
        self.esc_state_sub = self.create_subscription(
            String, self.esc_state_topic, self._esc_state_callback, 10)

        self.steering_cmd_pub = self.create_publisher(Float32, self.steering_cmd_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, sensor_qos)
        self.mag_pub = self.create_publisher(MagneticField, self.mag_topic, sensor_qos)
        self.voltage_pub = self.create_publisher(Float32, self.voltage_topic, sensor_qos)

        self.timer = self.create_timer(0.1, self._timer_callback)

        # --- Hardware interface ---
        self.robot = Rosmaster()
        self.robot.create_receive_threading()
        self._rosmaster_lock = threading.Lock()

        # --- Internal state ---
        self.cmd_vel_msg = TwistStamped()
        self.target_physical_velocity = 0.0
        self._closed_loop_buzzed = False
        self._last_esc_state = 'unknown'

        self.get_logger().info(
            'yb_eb_external_esc_node iniciado.\n'
            f'  cmd_vel_topic: {self.cmd_vel_topic}\n'
            f'  steering_cmd_topic: {self.steering_cmd_topic}\n'
            f'  esc_state_topic: {self.esc_state_topic}\n'
            f'  wheelbase: {self.wheelbase:.3f} m\n'
                f'  steering limit: {self.max_steer_angle_rad * self.steering_limit_percent:.3f} rad\n'
            f'  buzzer enabled: {self.buzzer_enabled}'
        )

    def _cmd_vel_callback(self, msg: TwistStamped):
        # Keep the full message (for angular.z) but compute a physical velocity
        self.cmd_vel_msg = msg

        raw_cmd = float(msg.twist.linear.x)
        sign = math.copysign(1.0, raw_cmd) if raw_cmd != 0.0 else 1.0
        abs_cmd = abs(raw_cmd)

        # Deadzone
        if abs_cmd < self.deadzone_threshold:
            self.target_physical_velocity = 0.0
        else:
            # Guard against misconfiguration
            if self.cmd_max == self.cmd_min:
                self.get_logger().warn('cmd_max equals cmd_min; saturating to speed_max_ms')
                mapped = self.speed_max_ms
            else:
                if abs_cmd <= self.cmd_min:
                    mapped = self.speed_min_ms
                elif abs_cmd >= self.cmd_max:
                    mapped = self.speed_max_ms
                else:
                    # Linear interpolation between speed_min_ms and speed_max_ms
                    mapped = (
                        self.speed_min_ms
                        + ((self.speed_max_ms - self.speed_min_ms) / (self.cmd_max - self.cmd_min))
                        * (abs_cmd - self.cmd_min)
                    )

            self.target_physical_velocity = mapped * sign

        self._update_and_publish_steering()

    def _esc_state_callback(self, msg: String):
        state = str(msg.data).strip().lower()
        previous_state = self._last_esc_state
        self._last_esc_state = state

        if state == self.closed_loop_state_value and not self._closed_loop_buzzed:
            self._closed_loop_buzzed = True
            self.get_logger().info('ESC externa entró en closed_loop. Activando buzzer con 2 pitidos cortos.')
            if self.buzzer_enabled:
                threading.Thread(target=self._beep_closed_loop_sequence, daemon=True).start()
        elif state != self.closed_loop_state_value and previous_state == self.closed_loop_state_value:
            self._closed_loop_buzzed = False

    def _timer_callback(self):
        self._publish_telemetry()
        self._update_and_publish_steering()

    def _publish_telemetry(self):
        current_time = self.get_clock().now().to_msg()

        with self._rosmaster_lock:
            battery_voltage = float(self.robot.get_battery_voltage())
            ax, ay, az = self.robot.get_accelerometer_data()
            gx, gy, gz = self.robot.get_gyroscope_data()
            mx, my, mz = self.robot.get_magnetometer_data()

        battery_msg = Float32()
        battery_msg.data = battery_voltage

        if 5.0 < battery_msg.data < self.min_battery_voltage:
            self.get_logger().warn(
                f'¡ALERTA BATERÍA BAJA! Voltaje actual: {battery_msg.data:.2f}V '
                f'(Umbral: {self.min_battery_voltage}V)',
                throttle_duration_sec=2.0,
            )
            self._trigger_buzzer(100)

        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        mag_msg = MagneticField()
        mag_msg.header.stamp = current_time
        mag_msg.header.frame_id = 'imu_link'
        mag_msg.magnetic_field.x = mx
        mag_msg.magnetic_field.y = my
        mag_msg.magnetic_field.z = mz

        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)
        self.voltage_pub.publish(battery_msg)

    def _update_and_publish_steering(self):
        v = float(self.target_physical_velocity)
        omega = float(self.cmd_vel_msg.twist.angular.z)

        delta_efectivo = self._cmd_to_delta(v, omega)
        steering_pwm = self._delta_to_pwm(delta_efectivo)

        with self._rosmaster_lock:
            self.robot.set_pwm_servo(self.steering_servo_channel, steering_pwm)

        steering_msg = Float32()
        steering_msg.data = float(delta_efectivo)
        self.steering_cmd_pub.publish(steering_msg)

    def _cmd_to_delta(self, v: float, omega: float) -> float:
        # Use the configured maximum steer angle but don't exceed previously
        # configured steering limits.
        max_allowed_angle = min(self.max_steer_angle_rad, self.max_steer_angle_rad * self.steering_limit_percent)

        # If we have zero physical forward/backward speed, avoid division by zero
        # and directly apply a max steer angle proportional to the sign of omega.
        if abs(v) == 0.0:
            if abs(omega) > 0.0:
                return math.copysign(max_allowed_angle, omega)
            return 0.0

        delta = math.atan((omega * self.wheelbase) / v)
        return max(-max_allowed_angle, min(delta, max_allowed_angle))

    def _delta_to_pwm(self, delta: float) -> float:
        effective_angle = self.max_steer_angle_rad * self.steering_limit_percent
        return self._map_and_clamp(
            delta,
            -effective_angle,
            effective_angle,
            self.steering_pwm_min,
            self.steering_pwm_max,
        )

    def _map_and_clamp(self, x, in_min, in_max, out_min, out_max):
        x = max(in_min, min(x, in_max))
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def _trigger_buzzer(self, duration_ms: int):
        with self._rosmaster_lock:
            self.robot.set_beep(duration_ms)

    def _beep_closed_loop_sequence(self):
        for pulse_index in range(self.buzzer_pulse_count):
            self._trigger_buzzer(self.buzzer_pulse_ms)
            if pulse_index < self.buzzer_pulse_count - 1 and self.buzzer_pause_ms > 0:
                time.sleep(self.buzzer_pause_ms / 1000.0)

    def destroy_node(self):
        self.get_logger().info('Interceptando apagado (Ctrl+C). Centrando el servo por seguridad...')
        try:
            with self._rosmaster_lock:
                self.robot.set_pwm_servo(self.steering_servo_channel, self.steering_pwm_center)
                self.robot.set_beep(200)
        except Exception as exc:
            self.get_logger().error(f'Error al cerrar el nodo de hardware: {exc}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YbEbExternalEscNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Señal de interrupción de teclado recibida.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()