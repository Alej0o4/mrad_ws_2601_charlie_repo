#!/usr/bin/env python3

import math
import threading
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import tf2_geometry_msgs
from scipy.spatial import cKDTree


def quaternion_from_yaw(yaw: float) -> Quaternion:
    half_yaw = 0.5 * yaw
    q = Quaternion()
    q.w = math.cos(half_yaw)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half_yaw)
    return q


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_pose(first_pose, second_pose):
    x1, y1, theta1 = first_pose
    x2, y2, theta2 = second_pose
    cos_theta = math.cos(theta1)
    sin_theta = math.sin(theta1)
    return (
        x1 + cos_theta * x2 - sin_theta * y2,
        y1 + sin_theta * x2 + cos_theta * y2,
        normalize_angle(theta1 + theta2),
    )


def transform_points(points: np.ndarray, pose) -> np.ndarray:
    x, y, theta = pose
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    rotation = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]], dtype=np.float64)
    translation = np.array([x, y], dtype=np.float64)
    return points @ rotation.T + translation


def estimate_rigid_transform_2d(source_points: np.ndarray, target_points: np.ndarray):
    if source_points.shape[0] < 2:
        raise ValueError("At least two correspondences are required")

    source_centroid = np.mean(source_points, axis=0)
    target_centroid = np.mean(target_points, axis=0)

    source_centered = source_points - source_centroid
    target_centered = target_points - target_centroid
    covariance = source_centered.T @ target_centered

    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T

    translation = target_centroid - rotation @ source_centroid
    theta = math.atan2(rotation[1, 0], rotation[0, 0])
    return (float(translation[0]), float(translation[1]), float(theta))


class LaserICPNode(Node):
    def __init__(self):
        super().__init__("laser_icp_node")

        # --- Parámetros de entrada/salida ---
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("motion_prior_topic", "/odom")
        self.declare_parameter("odom_topic", "/odom_laser")
        self.declare_parameter("valid_topic", "/odom_laser/valid")
        self.declare_parameter("score_topic", "/odom_laser/score")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("laser_frame", "lidar_link")
        
        # --- Offsets del sensor (TF estático) ---
        self.declare_parameter("lidar_offset_x", 0.0)
        self.declare_parameter("lidar_offset_y", 0.0)
        
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("publish_twist", True)
        self.declare_parameter("use_motion_seed", True)
        self.declare_parameter("update_rate_hz", 30.0)
        self.declare_parameter("process_every_n_scans", 1)
        self.declare_parameter("scan_timeout_s", 0.5)
        self.declare_parameter("crop_min_angle", -math.pi)
        self.declare_parameter("crop_max_angle", math.pi)
        self.declare_parameter("scan_stride", 2)
        self.declare_parameter("max_scan_points", 240)
        self.declare_parameter("min_valid_points", 80)
        self.declare_parameter("min_angular_span", 1.0)
        self.declare_parameter("max_iterations", 20)
        self.declare_parameter("max_correspondence_distance", 0.25)
        self.declare_parameter("transformation_epsilon", 1e-4)
        self.declare_parameter("fitness_epsilon", 1e-4)
        self.declare_parameter("min_fitness_score", 0.35)
        self.declare_parameter("keyframe_translation_thresh", 0.20)
        self.declare_parameter("keyframe_rotation_thresh", 0.15)
        self.declare_parameter("pose_covariance_xy", 0.03)
        self.declare_parameter("pose_covariance_yaw", 0.10)
        self.declare_parameter("covariance_scale_from_fitness", 2.0)
        self.declare_parameter("covariance_scale_from_motion", 0.5)

        # Lectura de parámetros
        self.scan_topic = self.get_parameter("scan_topic").value
        self.motion_prior_topic = self.get_parameter("motion_prior_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.valid_topic = self.get_parameter("valid_topic").value
        self.score_topic = self.get_parameter("score_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        self.lidar_offset_x = float(self.get_parameter("lidar_offset_x").value)
        self.lidar_offset_y = float(self.get_parameter("lidar_offset_y").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_twist = bool(self.get_parameter("publish_twist").value)
        self.use_motion_seed = bool(self.get_parameter("use_motion_seed").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.process_every_n_scans = max(1, int(self.get_parameter("process_every_n_scans").value))
        self.scan_timeout_s = float(self.get_parameter("scan_timeout_s").value)
        self.crop_min_angle = float(self.get_parameter("crop_min_angle").value)
        self.crop_max_angle = float(self.get_parameter("crop_max_angle").value)
        self.scan_stride = max(1, int(self.get_parameter("scan_stride").value))
        self.max_scan_points = max(10, int(self.get_parameter("max_scan_points").value))
        self.min_valid_points = max(10, int(self.get_parameter("min_valid_points").value))
        self.min_angular_span = float(self.get_parameter("min_angular_span").value)
        self.max_iterations = max(1, int(self.get_parameter("max_iterations").value))
        self.max_correspondence_distance = float(self.get_parameter("max_correspondence_distance").value)
        self.transformation_epsilon = float(self.get_parameter("transformation_epsilon").value)
        self.fitness_epsilon = float(self.get_parameter("fitness_epsilon").value)
        self.min_fitness_score = float(self.get_parameter("min_fitness_score").value)
        self.keyframe_translation_thresh = float(self.get_parameter("keyframe_translation_thresh").value)
        self.keyframe_rotation_thresh = float(self.get_parameter("keyframe_rotation_thresh").value)
        self.pose_covariance_xy = float(self.get_parameter("pose_covariance_xy").value)
        self.pose_covariance_yaw = float(self.get_parameter("pose_covariance_yaw").value)
        self.covariance_scale_from_fitness = float(self.get_parameter("covariance_scale_from_fitness").value)
        self.covariance_scale_from_motion = float(self.get_parameter("covariance_scale_from_motion").value)

        # --- QoS ---
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # --- Suscripciones y publicaciones ---
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, sensor_qos)
        self.motion_prior_sub = None
        if self.motion_prior_topic:
            self.motion_prior_sub = self.create_subscription(
                Odometry,
                self.motion_prior_topic,
                self._motion_prior_callback,
                reliable_qos,
            )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, reliable_qos)
        self.valid_pub = self.create_publisher(Bool, self.valid_topic, reliable_qos)
        self.score_pub = self.create_publisher(Float32, self.score_topic, reliable_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- TF Listener para transformación estática ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._lidar_to_base_rotation = 0.0
        self._lidar_to_base_offset_x = self.lidar_offset_x
        self._lidar_to_base_offset_y = self.lidar_offset_y
        self._tf_initialized = False

        timer_period = 1.0 / self.update_rate_hz if self.update_rate_hz > 0.0 else 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self._publish_loop)

        self._lock = threading.Lock()
        self._scan_counter = 0
        self._last_processed_scan_counter = 0
        self._latest_scan_points = None
        self._latest_scan_stamp = None
        self._latest_scan_frame = self.laser_frame
        self._last_scan_receive_time = self.get_clock().now()
        self._previous_scan_stamp = None
        self._timestamp_buffer = deque(maxlen=3)

        self._keyframe_points = None
        self._keyframe_tree = None
        self._keyframe_pose_global = (0.0, 0.0, 0.0)
        self._current_pose_global = (0.0, 0.0, 0.0)
        self._last_relative_pose = (0.0, 0.0, 0.0)
        self._last_motion_prior = (0.0, 0.0)
        self._last_motion_prior_time = None
        self._last_accepted_time = self.get_clock().now()
        self._last_twist = (0.0, 0.0)
        self._last_score = 0.0
        self._last_valid = False

        self.get_logger().info(
            "Laser ICP node iniciado. "
            f"scan_topic={self.scan_topic}, odom_topic={self.odom_topic}, "
            f"publish_tf={self.publish_tf}, motion_seed={self.use_motion_seed}, "
            f"lidar_offset=({self.lidar_offset_x:.3f}, {self.lidar_offset_y:.3f})"
        )

    def _try_initialize_tf(self):
        """Intenta obtener la transformación estática base_link -> lidar_link."""
        if self._tf_initialized:
            return

        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.laser_frame, rclpy.time.Time())
            self._lidar_to_base_offset_x = float(transform.transform.translation.x)
            self._lidar_to_base_offset_y = float(transform.transform.translation.y)
            # Extraer yaw del quaternion
            quat = transform.transform.rotation
            self._lidar_to_base_rotation = math.atan2(
                2 * (quat.w * quat.z + quat.x * quat.y),
                1 - 2 * (quat.y * quat.y + quat.z * quat.z),
            )
            self._tf_initialized = True
            self.get_logger().info(
                f"TF inicializado: lidar offset=({self._lidar_to_base_offset_x:.4f}, "
                f"{self._lidar_to_base_offset_y:.4f}), rot={math.degrees(self._lidar_to_base_rotation):.1f}°"
            )
        except Exception as e:
            self.get_logger().debug(f"TF no disponible aún: {e}")

    def _motion_prior_callback(self, msg: Odometry):
        with self._lock:
            self._last_motion_prior = (
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.angular.z),
            )
            self._last_motion_prior_time = self.get_clock().now()

    def _scan_callback(self, msg: LaserScan):
        points = self._laser_scan_to_points(msg)
        with self._lock:
            self._scan_counter += 1
            self._latest_scan_points = points
            self._latest_scan_stamp = msg.header.stamp
            self._latest_scan_frame = msg.header.frame_id or self.laser_frame
            self._last_scan_receive_time = self.get_clock().now()
            self._timestamp_buffer.append(msg.header.stamp)

    def _laser_scan_to_points(self, msg: LaserScan) -> np.ndarray:
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if ranges.size == 0:
            return None

        angles = msg.angle_min + np.arange(ranges.size, dtype=np.float64) * msg.angle_increment
        valid_mask = np.isfinite(ranges)
        valid_mask &= ranges >= max(msg.range_min, 0.0)
        valid_mask &= ranges <= msg.range_max
        valid_mask &= angles >= self.crop_min_angle
        valid_mask &= angles <= self.crop_max_angle

        if self.scan_stride > 1:
            stride_mask = (np.arange(ranges.size) % self.scan_stride) == 0
            valid_mask &= stride_mask

        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        if valid_ranges.size == 0:
            return None

        if valid_ranges.size > self.max_scan_points:
            step = max(1, valid_ranges.size // self.max_scan_points)
            valid_ranges = valid_ranges[::step]
            valid_angles = valid_angles[::step]

        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        points = np.column_stack((x, y)).astype(np.float64)
        return points if points.shape[0] >= 2 else None

    def _scan_span(self, points: np.ndarray) -> float:
        if points is None or points.shape[0] < 2:
            return 0.0
        angles = np.arctan2(points[:, 1], points[:, 0])
        return float(np.max(angles) - np.min(angles))

    def _nearest_neighbors_kdtree(self, source_points: np.ndarray, target_tree: cKDTree) -> tuple:
        """Busca vecinos más cercanos usando KD-Tree. O(N log N) en lugar de O(N^2)."""
        if source_points.size == 0 or target_tree is None:
            return None, None

        distances, indices = target_tree.query(source_points, distance_upper_bound=self.max_correspondence_distance)
        valid_mask = np.isfinite(distances)

        if not np.any(valid_mask):
            return None, None

        source_corr = source_points[valid_mask]
        target_corr = target_tree.data[indices[valid_mask]]
        return source_corr, target_corr

    def _run_icp(self, source_points: np.ndarray, target_tree: cKDTree, initial_pose):
        """ICP con KD-Tree para búsqueda rápida de correspondencias."""
        pose = initial_pose
        previous_rmse = None
        correspondence_count = 0

        for iteration in range(self.max_iterations):
            transformed_source = transform_points(source_points, pose)
            source_corr, target_corr = self._nearest_neighbors_kdtree(transformed_source, target_tree)
            if source_corr is None or target_corr is None or source_corr.shape[0] < 2:
                return pose, float("inf"), 0, False

            correspondence_count = int(source_corr.shape[0])
            delta_pose = estimate_rigid_transform_2d(source_corr, target_corr)
            pose = compose_pose(delta_pose, pose)

            aligned_source = transform_points(source_points, pose)
            source_corr, target_corr = self._nearest_neighbors_kdtree(aligned_source, target_tree)
            if source_corr is None or target_corr is None or source_corr.shape[0] < 2:
                return pose, float("inf"), correspondence_count, False

            residuals = target_corr - source_corr
            rmse = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))

            translation_delta = math.hypot(delta_pose[0], delta_pose[1])
            rotation_delta = abs(delta_pose[2])
            converged = (
                translation_delta < self.transformation_epsilon
                and rotation_delta < self.transformation_epsilon
            )
            stable_error = previous_rmse is not None and abs(previous_rmse - rmse) < self.fitness_epsilon

            previous_rmse = rmse
            if converged or stable_error:
                return pose, rmse, correspondence_count, True

        aligned_source = transform_points(source_points, pose)
        source_corr, target_corr = self._nearest_neighbors_kdtree(aligned_source, target_tree)
        if source_corr is None or target_corr is None or source_corr.shape[0] < 2:
            return pose, float("inf"), correspondence_count, False

        residuals = target_corr - source_corr
        rmse = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
        return pose, rmse, int(source_corr.shape[0]), True

    def _compute_motion_seed(self, dt_seconds: float):
        if not self.use_motion_seed:
            return 0.0, 0.0, 0.0

        vx, wz = self._last_motion_prior
        if vx == 0.0 and wz == 0.0:
            return self._last_relative_pose

        delta_x = -vx * dt_seconds
        delta_theta = -wz * dt_seconds
        return delta_x, 0.0, delta_theta

    def _validate_timestamp(self, timestamp) -> bool:
        """Valida que el timestamp sea válido y progresivo."""
        if self._previous_scan_stamp is None:
            return True

        dt_ns = timestamp.sec * 1e9 + timestamp.nanosec - (
            self._previous_scan_stamp.sec * 1e9 + self._previous_scan_stamp.nanosec
        )

        if dt_ns < 0:
            self.get_logger().warn("Timestamp retroactivo detectado. Ignorando escaneo.")
            return False

        if dt_ns > 1e9:  # Más de 1 segundo
            self.get_logger().warn(f"Salto de timestamp > 1s detectado: {dt_ns / 1e9:.2f}s")

        return True

    def _calculate_dt_from_timestamps(self) -> float:
        """Calcula dt usando timestamps de ROS en lugar del reloj del SO."""
        if self._previous_scan_stamp is None:
            return 0.033  # Default 30 Hz

        dt_ns = self._latest_scan_stamp.sec * 1e9 + self._latest_scan_stamp.nanosec - (
            self._previous_scan_stamp.sec * 1e9 + self._previous_scan_stamp.nanosec
        )
        dt_seconds = max(1e-3, dt_ns / 1e9)
        return dt_seconds

    def _apply_lidar_offset(self, icp_pose):
        """Aplica la transformación estática del LiDAR para obtener la pose del base_link."""
        x, y, theta = icp_pose
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        # Rotar el offset del LiDAR según la orientación actual
        rotated_offset_x = cos_theta * self._lidar_to_base_offset_x - sin_theta * self._lidar_to_base_offset_y
        rotated_offset_y = sin_theta * self._lidar_to_base_offset_x + cos_theta * self._lidar_to_base_offset_y

        # Corregir la posición
        corrected_x = x - rotated_offset_x
        corrected_y = y - rotated_offset_y
        corrected_theta = normalize_angle(theta - self._lidar_to_base_rotation)

        return (corrected_x, corrected_y, corrected_theta)

    def _process_latest_scan(self):
        self._try_initialize_tf()

        with self._lock:
            scan_points = self._latest_scan_points
            scan_stamp = self._latest_scan_stamp
            scan_counter = self._scan_counter
            keyframe_points = self._keyframe_points
            keyframe_tree = self._keyframe_tree
            keyframe_pose_global = self._keyframe_pose_global
            last_processed_scan_counter = self._last_processed_scan_counter

        if scan_points is None or scan_stamp is None:
            return

        if scan_counter == last_processed_scan_counter:
            return

        if scan_counter - last_processed_scan_counter < self.process_every_n_scans:
            return

        # Validar timestamp
        if not self._validate_timestamp(scan_stamp):
            with self._lock:
                self._last_processed_scan_counter = scan_counter
            return

        if keyframe_points is None:
            with self._lock:
                self._keyframe_points = scan_points.copy()
                self._keyframe_tree = cKDTree(scan_points)
                self._keyframe_pose_global = self._current_pose_global
                self._last_relative_pose = (0.0, 0.0, 0.0)
                self._last_processed_scan_counter = scan_counter
                self._last_score = 1.0
                self._last_valid = False
                self._previous_scan_stamp = scan_stamp
            return

        # Calcular dt usando timestamps de ROS (no del SO)
        dt_seconds = self._calculate_dt_from_timestamps()

        initial_pose = self._compute_motion_seed(dt_seconds)
        if initial_pose == (0.0, 0.0, 0.0):
            initial_pose = self._last_relative_pose

        estimated_pose, rmse, correspondence_count, converged = self._run_icp(
            scan_points,
            keyframe_tree,
            initial_pose,
        )

        # Aplicar corrección del brazo de palanca del LiDAR
        estimated_pose_corrected = self._apply_lidar_offset(estimated_pose)

        span_ok = self._scan_span(scan_points) >= self.min_angular_span
        enough_points = scan_points.shape[0] >= self.min_valid_points
        score = 0.0 if not math.isfinite(rmse) else max(0.0, 1.0 - (rmse / max(self.max_correspondence_distance, 1e-6)))
        valid = (
            enough_points
            and span_ok
            and correspondence_count >= self.min_valid_points
            and converged
            and math.isfinite(rmse)
            and rmse <= self.max_correspondence_distance
            and score >= self.min_fitness_score
        )

        current_global_pose = compose_pose(keyframe_pose_global, estimated_pose_corrected)

        if valid:
            delta_global_x = current_global_pose[0] - self._current_pose_global[0]
            delta_global_y = current_global_pose[1] - self._current_pose_global[1]
            delta_global_theta = normalize_angle(current_global_pose[2] - self._current_pose_global[2])
            self._last_twist = (
                math.hypot(delta_global_x, delta_global_y) / max(dt_seconds, 1e-3),
                delta_global_theta / max(dt_seconds, 1e-3),
            )
            self._current_pose_global = current_global_pose
            self._last_relative_pose = estimated_pose_corrected
            self._last_score = score
            self._last_valid = True

            motion_translation = math.hypot(estimated_pose_corrected[0], estimated_pose_corrected[1])
            motion_rotation = abs(estimated_pose_corrected[2])
            if (
                motion_translation >= self.keyframe_translation_thresh
                or motion_rotation >= self.keyframe_rotation_thresh
            ):
                with self._lock:
                    self._keyframe_points = scan_points.copy()
                    self._keyframe_tree = cKDTree(scan_points)
                    self._keyframe_pose_global = self._current_pose_global
                    self._last_relative_pose = (0.0, 0.0, 0.0)
        else:
            self._last_valid = False
            self._last_score = score

        with self._lock:
            self._last_processed_scan_counter = scan_counter
            self._previous_scan_stamp = scan_stamp

    def _publish_odometry(self, scan_stamp):
        """Publica odometría usando el timestamp del sensor (no del SO)."""
        odom_msg = Odometry()
        odom_msg.header.stamp = scan_stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        with self._lock:
            pose_x, pose_y, pose_theta = self._current_pose_global
            valid = self._last_valid
            score = self._last_score
            linear_v, angular_w = self._last_twist

        odom_msg.pose.pose.position.x = pose_x
        odom_msg.pose.pose.position.y = pose_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quaternion_from_yaw(pose_theta)

        if self.publish_twist:
            odom_msg.twist.twist.linear.x = linear_v
            odom_msg.twist.twist.angular.z = angular_w

        pose_cov_scale = self.pose_covariance_xy
        yaw_cov_scale = self.pose_covariance_yaw
        if valid:
            motion_scale = 1.0 + self.covariance_scale_from_motion * min(5.0, abs(linear_v) + abs(angular_w))
            fitness_scale = 1.0 + self.covariance_scale_from_fitness * max(0.0, 1.0 - score)
            pose_cov_scale *= motion_scale * fitness_scale
            yaw_cov_scale *= motion_scale * fitness_scale
        else:
            pose_cov_scale *= 25.0
            yaw_cov_scale *= 25.0

        pose_covariance = [0.0] * 36
        pose_covariance[0] = pose_cov_scale
        pose_covariance[7] = pose_cov_scale
        pose_covariance[35] = yaw_cov_scale
        odom_msg.pose.covariance = pose_covariance

        twist_covariance = [0.0] * 36
        twist_covariance[0] = 0.05 if valid else 5.0
        twist_covariance[35] = 0.10 if valid else 5.0
        odom_msg.twist.covariance = twist_covariance

        self.odom_pub.publish(odom_msg)
        self.valid_pub.publish(Bool(data=valid))
        self.score_pub.publish(Float32(data=float(score)))

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = scan_stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = pose_x
            tf_msg.transform.translation.y = pose_y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = quaternion_from_yaw(pose_theta)
            self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_loop(self):
        current_time = self.get_clock().now()

        with self._lock:
            dt_since_scan = (current_time - self._last_scan_receive_time).nanoseconds / 1e9
            if dt_since_scan > self.scan_timeout_s:
                self._last_valid = False
            latest_scan_stamp = self._latest_scan_stamp

        self._process_latest_scan()

        # Publicar solo si tenemos timestamp válido del sensor
        if latest_scan_stamp is not None:
            self._publish_odometry(latest_scan_stamp)


def main(args=None):
    rclpy.init(args=args)
    node = LaserICPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
