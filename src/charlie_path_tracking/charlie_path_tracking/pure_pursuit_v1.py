#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path

import tf2_ros
from tf2_ros import TransformException

# Requires: ros-<distro>-tf2-geometry-msgs
from tf2_geometry_msgs import do_transform_pose, do_transform_pose_stamped


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class PurePursuitNode(Node):
    """
    Minimal but robust Pure Pursuit for cmd_vel (diff-drive style):
      - subscribes: nav_msgs/Path
      - publishes: geometry_msgs/Twist
      - uses TF: path.header.frame_id -> base_link
    """

    def __init__(self):
        super().__init__("pure_pursuit_node")

        # ---- Parameters
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")

        self.declare_parameter("base_frame", "base_link")

        # Control
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("v_nominal", 0.5)           # m/s
        self.declare_parameter("max_speed", 1.0)            # m/s
        self.declare_parameter("max_omega", 1.5)            # rad/s
        self.declare_parameter("goal_tolerance", 0.25)      # m

        # Lookahead: Ld = clamp(L0 + k*v, Lmin, Lmax)
        self.declare_parameter("lookahead_L0", 0.6)         # m
        self.declare_parameter("lookahead_kv", 0.0)         # s  (0 disables adaptation)
        self.declare_parameter("lookahead_min", 0.4)        # m
        self.declare_parameter("lookahead_max", 2.0)        # m

        # Small eps to avoid division by zero
        self.declare_parameter("eps", 1e-6)

        # TF timeout
        self.declare_parameter("tf_timeout_sec", 0.2)

        # ---- Read parameters
        self.path_topic = self.get_parameter("path_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.base_frame = self.get_parameter("base_frame").value

        self.rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.v_nominal = float(self.get_parameter("v_nominal").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_omega = float(self.get_parameter("max_omega").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance").value)

        self.L0 = float(self.get_parameter("lookahead_L0").value)
        self.kv = float(self.get_parameter("lookahead_kv").value)
        self.Lmin = float(self.get_parameter("lookahead_min").value)
        self.Lmax = float(self.get_parameter("lookahead_max").value)

        self.eps = float(self.get_parameter("eps").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)

        # ---- ROS interfaces
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, 10)

        # ---- TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- State
        self.path: List[PoseStamped] = []
        self.path_frame: Optional[str] = None
        self.has_path = False
        self.last_target_index = 0  # progressing index

        # ---- Timer
        dt = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(dt, self.on_timer)

        self.get_logger().info(f"Pure Pursuit listening on {self.path_topic}, publishing {self.cmd_topic}")

    def on_path(self, msg: Path) -> None:
        self.path = list(msg.poses)
        self.path_frame = msg.header.frame_id if msg.header.frame_id else None
        self.has_path = len(self.path) > 0 and self.path_frame is not None
        self.last_target_index = 0  # reset; you can improve by finding closest point here

        if not self.has_path:
            self.get_logger().warn("Received empty path or missing frame_id.")
        else:
            self.get_logger().info(f"Received path with {len(self.path)} poses in frame '{self.path_frame}'.")

    def on_timer(self) -> None:
        if not self.has_path:
            self.publish_stop()
            return

        # 1) Get transform base_link <- path_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.path_frame,
                rclpy.time.Time(),  # latest
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed ({self.path_frame} -> {self.base_frame}): {ex}")
            self.publish_stop()
            return

        # 2) Check goal distance (transform last pose)
        goal_pose_b = self.transform_pose_to_base(self.path[-1], tf)
        if goal_pose_b is None:
            self.publish_stop()
            return

        goal_dx = goal_pose_b.pose.position.x
        goal_dy = goal_pose_b.pose.position.y
        goal_dist = math.hypot(goal_dx, goal_dy)

        if goal_dist <= self.goal_tol:
            self.publish_stop()
            return

        # 3) Compute lookahead (optionally speed-adaptive)
        v_cmd = clamp(self.v_nominal, 0.0, self.max_speed)
        Ld = clamp(self.L0 + self.kv * abs(v_cmd), self.Lmin, self.Lmax)

        # 4) Select target point in base_link frame
        target = self.find_lookahead_target(tf, Ld)
        if target is None:
            # No valid "ahead" target; safest is stop
            self.publish_stop()
            return

        bx, by, idx = target
        self.last_target_index = idx

        # 5) Pure Pursuit curvature and angular velocity
        # kappa = 2*by / Ld^2
        kappa = (2.0 * by) / (Ld * Ld + self.eps)
        omega = v_cmd * kappa

        # Clamp omega
        omega = clamp(omega, -self.max_omega, self.max_omega)

        # 6) Publish cmd_vel
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(v_cmd)
        cmd.twist.angular.z = float(omega)
        self.cmd_pub.publish(cmd)


    def transform_pose_to_base(self, pose_st, tf):
        """
        Transform PoseStamped from path_frame into base_frame using TF2.
        """
        try:
            return do_transform_pose_stamped(pose_st, tf)
        except Exception as ex:
            self.get_logger().warn(f"Pose transform failed: {ex}")
            return None

    def find_lookahead_target(self, tf, Ld: float) -> Optional[Tuple[float, float, int]]:
        """
        Scan forward from last_target_index, find first point that:
          - is in front of robot (bx > 0)
          - has distance >= Ld
        Returns (bx, by, index) in base_link frame.
        """
        n = len(self.path)
        start = clamp(self.last_target_index, 0, n - 1)

        best_fallback = None  # last pose ahead, in case none reaches Ld

        for i in range(int(start), n):
            pose_b = self.transform_pose_to_base(self.path[i], tf)
            if pose_b is None:
                continue

            bx = pose_b.pose.position.x
            by = pose_b.pose.position.y

            if bx <= 0.0:
                continue  # behind or exactly sideways

            d = math.hypot(bx, by)

            # Keep a fallback "ahead" point
            best_fallback = (bx, by, i)

            if d >= Ld:
                return (bx, by, i)

        # If we never reached Ld, use the farthest ahead point we saw
        return best_fallback

    def publish_stop(self) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()