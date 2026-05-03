#!/usr/bin/env python3
"""
Motor Startup Control Node

This node performs a controlled motor startup sequence by:
1. Subscribing to joystick (Joy) messages and ESC state (CFOC state)
2. Detecting when a configured button is pressed
3. Publishing fixed velocity commands at 20 Hz to drive the motor through startup
4. Stopping when CLOSED_LOOP state is reached

Configurable behavior:
- hold_button_required: If true, button must be held throughout startup
  (releasing aborts). If false, single press is sufficient.
- joy_button_idx: Which joystick button triggers startup (default: 0 for button A)
- startup_cmd_value: The fixed linear velocity command during startup (default: 0.3)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import time


class StartupNode(Node):
    """
    State machine for motor startup control via joystick.
    
    States:
    - IDLE: Waiting for button press
    - STARTING: Button pressed, publishing startup commands
    - STARTED: Motor reached CLOSED_LOOP state
    - ABORTED: Button released during startup (only if hold_button_required=true)
    """
    
    # State constants
    STATE_IDLE = "IDLE"
    STATE_STARTING = "STARTING"
    STATE_STARTED = "STARTED"
    STATE_ABORTED = "ABORTED"
    
    def __init__(self):
        super().__init__('startup_node')
        
        # ===== PARAMETERS =====
        self.declare_parameter('joy_button_idx', 0)  # Button A = 0
        self.declare_parameter('hold_button_required', True)
        self.declare_parameter('startup_cmd_value', 0.3)
        self.declare_parameter('startup_timeout_s', 6.0)
        
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cfoc_state_topic', '/esc/cfoc_state')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_start')
        
        self.declare_parameter('publish_hz', 20)
        
        # Get parameters
        self.joy_button_idx = self.get_parameter('joy_button_idx').value
        self.hold_button_required = self.get_parameter('hold_button_required').value
        self.startup_cmd_value = self.get_parameter('startup_cmd_value').value
        self.startup_timeout_s = self.get_parameter('startup_timeout_s').value
        
        self.joy_topic = self.get_parameter('joy_topic').value
        self.cfoc_state_topic = self.get_parameter('cfoc_state_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        self.publish_hz = self.get_parameter('publish_hz').value
        
        # ===== STATE VARIABLES =====
        self.current_state = self.STATE_IDLE
        self.button_pressed = False
        self.cfoc_state = "IDLE"
        self.startup_start_time = None
        self.last_button_state = False
        self._last_publish_log_time = 0.0
        
        # ===== PUBLISHERS & SUBSCRIBERS =====
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self._joy_callback, 10)
        self.cfoc_sub = self.create_subscription(String, self.cfoc_state_topic, 
                                                  self._cfoc_state_callback, 10)
        
        # Timer for publishing commands at 20 Hz
        self.publish_interval = 1.0 / self.publish_hz
        self.timer = self.create_timer(self.publish_interval, self._timer_callback)
        
        # Log startup
        self.get_logger().info(
            f"Startup Node initialized:\n"
            f"  Button Index: {self.joy_button_idx}\n"
            f"  Hold Required: {self.hold_button_required}\n"
            f"  Startup Command Value: {self.startup_cmd_value} m/s\n"
            f"  Publish Rate: {self.publish_hz} Hz\n"
            f"  Timeout: {self.startup_timeout_s} s"
        )
    
    # ===== CALLBACKS =====
    
    def _joy_callback(self, msg: Joy):
        """
        Handle joystick messages.
        Detects button press and manages state transitions.
        """
        # Check if button is available in message
        if len(msg.buttons) <= self.joy_button_idx:
            self.get_logger().warn(
                f"Joy message has {len(msg.buttons)} buttons, "
                f"but requested button index is {self.joy_button_idx}"
            )
            return
        
        button_is_pressed = bool(msg.buttons[self.joy_button_idx])
        
        # Detect rising edge (button press)
        if button_is_pressed and not self.last_button_state:
            if self.current_state in [self.STATE_IDLE, self.STATE_ABORTED, self.STATE_STARTED]:
                self.get_logger().info(f"Button {self.joy_button_idx} pressed - Starting motor startup")
                self.current_state = self.STATE_STARTING
                self.startup_start_time = time.time()
            elif self.current_state == self.STATE_STARTING:
                self.get_logger().info(
                    f"Button {self.joy_button_idx} pressed, startup already running"
                )
        
        # Handle button release
        elif not button_is_pressed and self.last_button_state:
            self.get_logger().info(f"Button {self.joy_button_idx} released")
            
            if self.hold_button_required and self.current_state == self.STATE_STARTING:
                # In hold mode, releasing button aborts startup
                self.get_logger().warn("Hold mode: Button released during startup - ABORTING")
                self.current_state = self.STATE_ABORTED
                # Stop publishing immediately
                self._send_stop_command()
            elif not self.hold_button_required and self.current_state == self.STATE_STARTING:
                # In non-hold mode, button release doesn't stop startup
                self.get_logger().info("Press mode: Button released, but continuing startup sequence")
        
        self.button_pressed = button_is_pressed
        self.last_button_state = button_is_pressed
    
    def _cfoc_state_callback(self, msg: String):
        """
        Monitor CFOC state from ESC telemetry.
        Detect CLOSED_LOOP state to mark startup as complete.
        """
        old_state = self.cfoc_state
        self.cfoc_state = msg.data
        
        # Log state transitions
        if old_state != self.cfoc_state:
            self.get_logger().info(f"CFOC State: {old_state} → {self.cfoc_state}")
        
        # Check if we reached CLOSED_LOOP during startup
        if self.current_state == self.STATE_STARTING and self.cfoc_state == "CLOSED_LOOP":
            self.get_logger().info("Motor reached CLOSED_LOOP state - Startup sequence complete!")
            self.current_state = self.STATE_STARTED
            # Stop publishing
            self._send_stop_command()
    
    def _timer_callback(self):
        """
        Publish motor commands at configured frequency (20 Hz).
        Only publishes when in STARTING state.
        """
        # Check for timeout
        if self.current_state == self.STATE_STARTING and self.startup_start_time is not None:
            elapsed = time.time() - self.startup_start_time
            if elapsed > self.startup_timeout_s:
                self.get_logger().error(
                    f"Startup timeout ({self.startup_timeout_s}s) - "
                    f"CFOC state never reached CLOSED_LOOP. Aborting."
                )
                self.current_state = self.STATE_ABORTED
                self._send_stop_command()
                return
        
        # Publish command only if in STARTING state
        if self.current_state == self.STATE_STARTING:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            
            # Fixed velocity for startup (forward only)
            msg.twist.linear.x = self.startup_cmd_value
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = 0.0
            
            self.cmd_pub.publish(msg)

            # Periodic visibility log to confirm active publishing
            now = time.time()
            if now - self._last_publish_log_time > 1.0:
                self.get_logger().info(
                    f"Publishing startup command on {self.cmd_vel_topic}: "
                    f"linear.x={self.startup_cmd_value:.3f}"
                )
                self._last_publish_log_time = now
    
    def _send_stop_command(self):
        """
        Send a stop command (zero velocity) to the motor.
        Used when aborting or startup is complete.
        """
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StartupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
