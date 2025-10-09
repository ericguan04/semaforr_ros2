#!/usr/bin/env python3
from config import (CAMERA_INFO_TOPIC, LIDAR_TOPIC, OPENPOSE_OUTPUT_TOPIC,
                    MAX_SYNC_DELAY, PERSON_RELATIVE_LOCALIZER_OUTPUT_TOPIC)
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import LaserScan, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber


class PersonRelativeLocalizer(Node):
    """Estimates relative positions of people in the robot's local coordinate frame
    using LiDAR data and 2D pose detection data from OpenPose

    SIMPLIFIED VERSION: Assumes camera and LiDAR are co-located and aligned.
    This is a functional prototype - add TF transforms later for accuracy.
    """

    def __init__(self):
        super().__init__('person_relative_localizer')

        self.declare_parameter('camera_info_topic', CAMERA_INFO_TOPIC)
        self.declare_parameter('lidar_topic', LIDAR_TOPIC)
        self.declare_parameter('poses_2d_topic', OPENPOSE_OUTPUT_TOPIC)
        self.declare_parameter('max_sync_delay', MAX_SYNC_DELAY)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('max_detection_range', 10.0)  # meters
        self.declare_parameter('lidar_search_window', 5)  # rays to search
        self.declare_parameter('person_height_estimate', 1.7)  # meters
        self.declare_parameter('min_person_width', 0.3)  # meters
        self.declare_parameter('max_person_width', 0.8)  # meters

        self.camera_info = None
        self.focal_x = None
        self.focal_y = None
        self.center_x = None
        self.center_y = None

        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        poses_2d_topic = self.get_parameter('poses_2d_topic').get_parameter_value().string_value
        lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        max_sync_delay = self.get_parameter('max_sync_delay').get_parameter_value().double_value

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

        self.get_logger().info('Person Relative Localizer Node started (SIMPLE MODE - no TF)')
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

    def synchronized_callback(self, pose_msg: PoseArray, lidar_msg: LaserScan):
        """
        Main callback that fuses 2D poses with LiDAR data.

        Args:
            pose_msg: Array of 2D detected poses from OpenPose
            lidar_msg: LiDAR scan data
        """
        if self.camera_info is None:
            self.get_logger().warn('Camera info not yet received', throttle_duration_sec=5.0)
            return

        if len(pose_msg.poses) == 0:
            self.get_logger().debug('No people detected in this frame')
            return

        try:
            poses_3d = PoseArray()
            poses_3d.header = lidar_msg.header
            poses_3d.header.frame_id = lidar_msg.header.frame_id

            min_confidence = self.get_parameter('min_detection_confidence').get_parameter_value().double_value
            max_range = self.get_parameter('max_detection_range').get_parameter_value().double_value

            for i, pose_2d in enumerate(pose_msg.poses):
                pixel_x = pose_2d.position.x
                pixel_y = pose_2d.position.y
                confidence = pose_2d.position.z # OpenPose should store confidence in z

                if confidence < min_confidence:
                    self.get_logger().debug(f'Skipping person {i}: confidence {confidence:.2f} < {min_confidence}')
                    continue

                # Convert pixel to camera angles
                h_angle = self.pixel_to_camera_angle_horizontal(pixel_x)
                v_angle = self.pixel_to_camera_angle_vertical(pixel_y)

                # Get LiDAR range at this angle
                lidar_range = self.get_lidar_range_at_angle(h_angle, lidar_msg)

                if lidar_range is None:
                    self.get_logger().debug(
                        f'No valid LiDAR range for person {i} at angle {math.degrees(h_angle):.1f}°')
                    continue

                if lidar_range > max_range:
                    self.get_logger().debug(f'Person {i} too far: {lidar_range:.2f}m > {max_range}m')
                    continue

                if not self.validate_person_detection(h_angle, lidar_range, lidar_msg):
                    self.get_logger().debug(f'Person {i} failed validation checks')
                    continue

                pose_3d = self.create_3d_pose(h_angle, v_angle, lidar_range, confidence)

                if pose_3d is not None:
                    poses_3d.poses.append(pose_3d)
                    self.get_logger().debug(
                        f'Person {i} localized at ({pose_3d.position.x:.2f}, '
                        f'{pose_3d.position.y:.2f}) m, range={lidar_range:.2f}m'
                    )

            self.pose_3d_pub.publish(poses_3d)

            if len(poses_3d.poses) > 0:
                self.get_logger().info(f'Published {len(poses_3d.poses)} 3D person positions')

        except Exception as e:
            self.get_logger().error(f'Error in synchronized callback: {str(e)}', throttle_duration_sec=1.0)

    def pixel_to_camera_angle_horizontal(self, pixel_x: float) -> float:
        """Utilizes camera information to calculate horizontal camera angle for provided pixel x"""
        angle = math.atan2(pixel_x - self.center_x, self.focal_x) # Positive angle = right, negative = left
        return angle

    def pixel_to_camera_angle_vertical(self, pixel_y: float) -> float:
        """Utilizes camera information to calculate vertical camera angle for provided pixel y"""
        angle = math.atan2(self.center_y - pixel_y, self.focal_y) # Positive angle = right, negative = left
        return angle

    def get_lidar_range_at_angle(self, target_angle, lidar_msg):
        """Get LiDAR range measurement closest to the target camera angle."""
        lidar_angle = target_angle # Convert camera angle to LiDAR angle coordinate system (this assumes camera and LiDAR are roughly aligned)

        if lidar_angle < lidar_msg.angle_min or lidar_angle > lidar_msg.angle_max:
            return None

        # Find corresponding LiDAR ray index
        angle_from_min = lidar_angle - lidar_msg.angle_min
        ray_index = int(round(angle_from_min / lidar_msg.angle_increment))

        ray_index = max(0, min(ray_index, len(lidar_msg.ranges) - 1))

        search_window = self.get_parameter('lidar_search_window').get_parameter_value().integer_value

        valid_ranges = []
        for offset in range(-search_window, search_window + 1):
            idx = ray_index + offset
            if 0 <= idx < len(lidar_msg.ranges):
                range_val = lidar_msg.ranges[idx]
                if (lidar_msg.range_min <= range_val <= lidar_msg.range_max and
                        not math.isinf(range_val) and not math.isnan(range_val)):
                    valid_ranges.append(range_val)

        if valid_ranges:
            # Return median of valid ranges to reduce noise
            return float(np.median(valid_ranges))

        return None

    def validate_person_detection(self, angle: float, range_val: float, lidar_msg: LaserScan) -> bool:
        """
        Validate that the detection corresponds to a person-like object.

        Args:
            angle: Horizontal angle to detected person
            range_val: Range to detected person
            lidar_msg: Full LiDAR scan

        Returns:
            True if detection appears valid
        """
        min_width = self.get_parameter('min_person_width').get_parameter_value().double_value
        max_width = self.get_parameter('max_person_width').get_parameter_value().double_value

        # Find ray index for this angle
        angle_from_min = angle - lidar_msg.angle_min
        center_idx = int(round(angle_from_min / lidar_msg.angle_increment))

        if center_idx < 0 or center_idx >= len(lidar_msg.ranges):
            return False

        # Estimate angular width of person at this range
        expected_angular_width = math.atan2(max_width / 2, range_val) * 2
        rays_to_check = int(expected_angular_width / lidar_msg.angle_increment / 2)

        # Check for consistent ranges around the detection
        consistent_count = 0
        for offset in range(-rays_to_check, rays_to_check + 1):
            idx = center_idx + offset
            if 0 <= idx < len(lidar_msg.ranges):
                r = lidar_msg.ranges[idx]
                if (lidar_msg.range_min <= r <= lidar_msg.range_max and
                        not math.isinf(r) and not math.isnan(r)):
                    # Check if range is similar (within 0.5m)
                    if abs(r - range_val) < 0.5:
                        consistent_count += 1

        # Require at least 30% of rays to be consistent
        min_consistent_rays = max(2, int(rays_to_check * 0.3))
        return consistent_count >= min_consistent_rays

    def create_3d_pose(self, h_angle: float, v_angle: float, distance: float, confidence: float) -> Pose:
        """
        Create 3D pose from angles and distance.

        Args:
            h_angle: Horizontal angle (yaw)
            v_angle: Vertical angle (pitch)
            distance: Range measurement
            confidence: Detection confidence

        Returns:
            3D pose in robot frame
        """
        try:
            x = distance * math.cos(h_angle)
            y = distance * math.sin(h_angle)
            z = 0.0 # Assume ground level

            person_height = self.get_parameter('person_height_estimate').get_parameter_value().double_value
            # z = distance * math.tan(v_angle)

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












