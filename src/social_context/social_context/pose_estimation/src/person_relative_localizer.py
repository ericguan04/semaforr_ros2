#!/usr/bin/env python3
from social_context.pose_estimation.config import (CAMERA_INFO_TOPIC, LIDAR_TOPIC, CAMERA_2D_POSE_DATA_TOPIC,
                    MAX_SYNC_DELAY, PERSON_RELATIVE_LOCALIZER_OUTPUT_TOPIC)
import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseArray, Pose, PointStamped, TransformStamped
from sensor_msgs.msg import LaserScan, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs


class PersonRelativeLocalizer(Node):
    """Estimates relative positions of people in the robot's local coordinate frame
    using LiDAR data and 2D pose detection data from OpenPose"""

    def __init__(self):
        super().__init__('person_relative_localizer')

        self.declare_parameter('camera_info_topic', CAMERA_INFO_TOPIC)
        self.declare_parameter('lidar_topic', LIDAR_TOPIC)
        self.declare_parameter('poses_2d_topic', CAMERA_2D_POSE_DATA_TOPIC)
        self.declare_parameter('max_sync_delay', MAX_SYNC_DELAY)
        self.declare_parameter('min_detection_confidence', 0.3)
        self.declare_parameter('max_detection_range', 10.0)  # meters
        self.declare_parameter('lidar_search_window', 5)  # rays to search
        self.declare_parameter('person_height_estimate', 1.7)  # meters
        self.declare_parameter('min_person_width', 0.3)  # meters
        self.declare_parameter('max_person_width', 0.8)  # meters

        self.declare_parameter('camera_frame', 'rgb_camera_optical_frame') # In HuNav it's called 'rgb_camera_optical_frame'
        self.declare_parameter('lidar_frame', 'base_laser_link')
        self.declare_parameter('output_frame', 'base_laser_link')
        self.declare_parameter('tf_timeout', 0.5)

        self.camera_info = None
        self.focal_x = None
        self.focal_y = None
        self.center_x = None
        self.center_y = None

        # Transform Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Store transform from camera to lidar (computed once and cached)
        self.camera_to_lidar_transform = None
        self.transform_last_updated = None

        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        poses_2d_topic = self.get_parameter('poses_2d_topic').get_parameter_value().string_value
        lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        max_sync_delay = self.get_parameter('max_sync_delay').get_parameter_value().double_value

        self.camera_frame = self.get_parameter('camera_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.output_frame = self.get_parameter('output_frame').value
        self.tf_timeout = self.get_parameter('tf_timeout').value

        self.pose_3d_pub = self.create_publisher(
            PoseArray,
            PERSON_RELATIVE_LOCALIZER_OUTPUT_TOPIC,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )

        # Synchronized subscribers for pose and LiDAR
        self.pose_sub = Subscriber(self, PoseArray, poses_2d_topic)
        self.lidar_sub = Subscriber(self, LaserScan, lidar_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.pose_sub, self.lidar_sub],
            queue_size=10,
            slop=max_sync_delay,
        )
        self.sync.registerCallback(self.synchronized_callback)

        self.get_logger().info('Person Relative Localizer Node started (WITH TF TRANSFORMS)')
        self.get_logger().info(f'Camera frame: {self.camera_frame}')
        self.get_logger().info(f'LiDAR frame: {self.lidar_frame}')
        self.get_logger().info(f'Output frame: {self.output_frame}')
        self.get_logger().info(f'Subscribing to camera info: {camera_info_topic}')
        self.get_logger().info(f'Subscribing to 2D poses: {poses_2d_topic}')
        self.get_logger().info(f'Subscribing to LiDAR: {lidar_topic}')
        self.get_logger().info(f'  Output: {PERSON_RELATIVE_LOCALIZER_OUTPUT_TOPIC}')

    def camera_info_callback(self, msg):
        """Store camera calibration parameters from camera_info topic."""
        if self.camera_info is None:
            self.camera_info = msg

            # Extract intrinsic parameters from K matrix: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
            self.focal_x = msg.k[0]
            self.focal_y = msg.k[4]
            self.center_x = msg.k[2]
            self.center_y = msg.k[5]

            self.get_logger().info(
                f'Camera calibration received: '
                f'fx={self.focal_x:.1f}, fy={self.focal_y:.1f}, '
                f'cx={self.center_x:.1f}, cy={self.center_y:.1f}'
            )

    def get_camera_to_lidar_transform(self, timestamp):
        """
        Get transform from camera frame to LiDAR frame.
        Caches the transform for efficiency.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.lidar_frame,
                self.camera_frame,
                timestamp,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout)
            )

            self.camera_to_lidar_transform = transform
            self.transform_last_updated = timestamp

            return transform

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {self.camera_frame} to {self.lidar_frame}: {ex}',
                throttle_duration_sec=5.0
            )
            return None

    def transform_camera_ray_to_lidar(self, pixel_x, pixel_y, timestamp):
        """
        Transform a camera ray (from pixel) to a direction in the LiDAR frame.
        Returns: (angle, success) - angle in LiDAR frame, and whether transform succeeded
        """
        # Get or use cached transform
        if self.camera_to_lidar_transform is None:
            transform = self.get_camera_to_lidar_transform(timestamp)
        else:
            transform = self.camera_to_lidar_transform

        if transform is None:
            return None, False

        # Create a point along the camera ray at unit depth (1 meter)
        # Camera frame convention: +Z forward, +X right, +Y down
        camera_angle_h = math.atan2(pixel_x - self.center_x, self.focal_x)
        camera_angle_v = math.atan2(self.center_y - pixel_y, self.focal_y)

        # Point at 1m depth in camera frame
        point_camera = PointStamped()
        point_camera.header.frame_id = self.camera_frame
        point_camera.header.stamp = timestamp.to_msg()

        # In camera optical frame: Z is forward, X is right, Y is down
        point_camera.point.x = math.tan(camera_angle_h)  # Right
        point_camera.point.y = -math.tan(camera_angle_v)  # Up (negative Y)
        point_camera.point.z = 1.0  # Forward

        try:
            # Transform to LiDAR frame
            point_lidar = tf2_geometry_msgs.do_transform_point(point_camera, transform)

            # Calculate angle in LiDAR frame (typically: +X forward, +Y left)
            # Angle from +X axis in XY plane
            lidar_angle = math.atan2(point_lidar.point.y, point_lidar.point.x)

            return lidar_angle, True

        except Exception as e:
            self.get_logger().error(f'Error transforming point: {e}')
            return None, False

    def synchronized_callback(self, pose_msg: PoseArray, lidar_msg: LaserScan):
        """Main callback that fuses 2D poses with LiDAR data."""

        self.get_logger().info(f'=== SYNC: Received {len(pose_msg.poses)} poses ===')

        if self.camera_info is None:
            self.get_logger().warn('Camera info not yet received', throttle_duration_sec=5.0)
            return

        if len(pose_msg.poses) == 0:
            self.get_logger().debug('No people detected in this frame')
            return

        try:
            poses_3d = PoseArray()
            poses_3d.header = lidar_msg.header
            poses_3d.header.frame_id = self.output_frame

            min_confidence = self.get_parameter('min_detection_confidence').value
            max_range = self.get_parameter('max_detection_range').value

            timestamp = Time.from_msg(lidar_msg.header.stamp)

            for i, pose_2d in enumerate(pose_msg.poses):
                pixel_x = pose_2d.position.x
                pixel_y = pose_2d.position.y
                confidence = pose_2d.position.z

                self.get_logger().info(f'Person {i}: pixel=({pixel_x:.1f}, {pixel_y:.1f}), conf={confidence:.3f}')

                if confidence < min_confidence:
                    self.get_logger().debug(f'Skipping person {i}: confidence {confidence:.2f} < {min_confidence}')
                    continue

                lidar_angle, success = self.transform_camera_ray_to_lidar(pixel_x, pixel_y, timestamp)

                if not success or lidar_angle is None:
                    self.get_logger().debug(f'  Failed to transform to LiDAR frame')
                    continue

                # Get LiDAR range at this angle
                lidar_range = self.get_lidar_range_at_angle(lidar_angle, lidar_msg)

                if lidar_range is None:
                    self.get_logger().debug(
                        f'No valid LiDAR range for person {i} at angle {math.degrees(lidar_angle):.1f}°')
                    continue

                if lidar_range > max_range:
                    self.get_logger().debug(f'Person {i} too far: {lidar_range:.2f}m > {max_range}m')
                    continue

                if not self.validate_person_detection(lidar_angle, lidar_range, lidar_msg):
                    self.get_logger().debug(f'Person {i} failed validation checks')
                    continue

                pose_3d = self.create_3d_pose_in_lidar_frame(lidar_angle, lidar_range)

                if pose_3d is not None:
                    poses_3d.poses.append(pose_3d)
                    self.get_logger().info(
                        f'  ✓ Localized at ({pose_3d.position.x:.2f}, {pose_3d.position.y:.2f})m, '
                        f'range={lidar_range:.2f}m, angle={math.degrees(lidar_angle):.1f}°'
                    )

            self.pose_3d_pub.publish(poses_3d)

            if len(poses_3d.poses) > 0:
                self.get_logger().info(f'Published {len(poses_3d.poses)} 3D person positions')

        except Exception as e:
            self.get_logger().error(f'Error in synchronized callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def get_lidar_range_at_angle(self, target_angle, lidar_msg):
        """Get LiDAR range measurement closest to the target angle."""

        if target_angle < lidar_msg.angle_min or target_angle > lidar_msg.angle_max:
            return None

        # Find corresponding LiDAR ray index
        angle_from_min = target_angle - lidar_msg.angle_min
        ray_index = int(round(angle_from_min / lidar_msg.angle_increment))
        ray_index = max(0, min(ray_index, len(lidar_msg.ranges) - 1))

        search_window = self.get_parameter('lidar_search_window').value

        valid_ranges = []
        for offset in range(-search_window, search_window + 1):
            idx = ray_index + offset
            if 0 <= idx < len(lidar_msg.ranges):
                range_val = lidar_msg.ranges[idx]
                if (lidar_msg.range_min <= range_val <= lidar_msg.range_max and
                        not math.isinf(range_val) and not math.isnan(range_val)):
                    valid_ranges.append(range_val)

        if valid_ranges:
            return float(np.median(valid_ranges))

        return None

    def validate_person_detection(self, angle: float, range_val: float, lidar_msg: LaserScan) -> bool:
        """
        Validate that the detection corresponds to a person-like object.
        Returns True if detection appears valid
        """

        # Skip validation for very close objects
        if range_val < 1.0:
            return True

        min_width = self.get_parameter('min_person_width').value
        max_width = self.get_parameter('max_person_width').value

        angle_from_min = angle - lidar_msg.angle_min
        center_idx = int(round(angle_from_min / lidar_msg.angle_increment))

        if center_idx < 0 or center_idx >= len(lidar_msg.ranges):
            return False

        # Estimate angular width
        expected_angular_width = math.atan2(min_width / 2, range_val) * 2
        rays_to_check = max(3, int(expected_angular_width / lidar_msg.angle_increment))

        # Check for consistent ranges
        consistent_count = 0
        for offset in range(-rays_to_check, rays_to_check + 1):
            idx = center_idx + offset
            if 0 <= idx < len(lidar_msg.ranges):
                r = lidar_msg.ranges[idx]
                if (lidar_msg.range_min <= r <= lidar_msg.range_max and
                        not math.isinf(r) and not math.isnan(r)):
                    if abs(r - range_val) < 1.0:  # More lenient
                        consistent_count += 1

        # Require at least 2 consistent rays
        return consistent_count >= 2

    def create_3d_pose_in_lidar_frame(self, angle: float, distance: float) -> Pose:
        """
        Create 3D pose from angles and distance.

        Args:
            angle: Angle in LiDAR frame
            distance: Range measurement

        Returns:
            3D pose in robot frame
        """
        try:
            # LiDAR frame convention (typically): +X forward, +Y left, +Z up
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            z = 0.0 # Assume ground level

            pose_3d = Pose()
            pose_3d.position.x = float(x)
            pose_3d.position.y = float(y)
            pose_3d.position.z = float(z)

            # Orientation: face towards robot
            yaw = math.atan2(-y, -x)  # Opposite direction = facing robot

            # Convert yaw to quaternion (only rotating around z-axis)
            pose_3d.orientation.x = 0.0
            pose_3d.orientation.y = 0.0
            pose_3d.orientation.z = math.sin(yaw / 2.0)
            pose_3d.orientation.w = math.cos(yaw / 2.0)

            return pose_3d

        except Exception as e:
            self.get_logger().error(f'Error computing local person position: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PersonRelativeLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()












