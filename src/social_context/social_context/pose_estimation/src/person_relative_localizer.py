#!/usr/bin/env python3
from config import CAMERA_INFO_TOPIC, LIDAR_TOPIC, OPENPOSE_OUTPUT_TOPIC, MAX_SYNC_DELAY, PERSON_RELATIVE_LOCALIZER_OUTPUT_TOPIC
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import LaserScan, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber

class PersonRelativeLocalizer(Node):
    """
    Estimates relative positions of people in the robot's local coordinate frame using LiDAR data and 2D pose detection data from OpenPose
    """

    def __init__(self):
        super().__init__('person_relative_localizer')
        self.declare_parameters('camera_info_topic', CAMERA_INFO_TOPIC)
        self.declare_parameters('lidar_topic', LIDAR_TOPIC)
        self.declare_parameters('poses_2d_topic', OPENPOSE_OUTPUT_TOPIC)
        self.declare_parameters('max_sync_delay', MAX_SYNC_DELAY)
        self.camera_info = None

        # Note: We use the `Subscriber` class instead of the `create_subscriber` method because the `Subscriber` class provides us with synchronization
        self.pose_3d_pub = self.create_publisher(PoseArray, PERSON_RELATIVE_LOCALIZER_OUTPUT_TOPIC, 10)
        self.camera_info_sub = self.create_subscriber(CameraInfo, self.get_parameters('camera_info_topic').get_parameter_values().string_value, self.camera_info_callback, 10)
        self.pose_sub = Subscriber(self, PoseArray, self.get_parameters('poses_2d_topic').get_parameter_value().string_value())
        self.lidar_sub = Subscriber(self, LaserScan, self.get_parameters('lidar_topic').get_parameter_value().string_value())

        self.sync = ApproximateTimeSynchronizer(
            [self.pose_sub, self.lidar_sub],
            queue_size=10,
            slop=self.get_parameter('max_sync_delay').get_parameter_value().double_value
        )
        self.sync.registerCallback(self.person_relative_localizer_callback)

        self.get_logger().info('Person Relative Localizer Node started')

    def camera_info_callback(self, msg):
        """Capture and store the camera's calibration parameters when they become available"""
        self.camera_info = msg
        self.get_logger().info('Camera information received')

    def person_relative_localizer_callback(self, pose_msg, lidar_msg):
        """
        Utilizes 2D human pose detection data with LiDAR data to get 3D positions
        """
        if self.camera_info is None:
            self.get_logger().warn('Camera info not received')
            return

        try:
            poses_3d = PoseArray()
            poses_3d.header = pose_msg.header
            poses_3d.header.frame_id = 'base_laser_link'

            for pose_2d in pose_msg.pose:
                pixel_x = pose_2d.position.x

                camera_angle = self.pixel_to_camera_angle(pixel_x)
                lidar_range = self.get_lidar_range_at_angle(camera_angle, lidar_msg)

                if lidar_range is not None:
                    pose_3d = self.create_3d_pose(camera_angle, lidar_range)

                    if pose_3d is not None:
                        poses_3d.pose.append(pose_3d)

            self.pose_3d_pub.publish(poses_3d)

            if len(poses_3d.pose) > 0:
                self.get_logger().info('3D positions received')

        except Exception as e:
            self.get_logger().error(f'Error in person relative localizer callback: {str(e)}')

    def pixel_to_camera_angle(self, pixel_x):
        """Utilizes camera information to calculate horizontal camera angle for provided pixel x"""
        focal_x = self.camera_info.k[0]
        center_x = self.camera_info.k[2]
        angle = math.atan2(pixel_x - center_x, focal_x) # Positive angle = right, negative = left
        return angle

    def get_lidar_range_at_angle(self, target_angle, lidar_msg):
        """Get LiDAR range measurement closest to the target camera angle."""
        lidar_angle = target_angle # Convert camera angle to LiDAR angle coordinate system (this assumes camera and LiDAR are roughly aligned)

        if lidar_angle < lidar_msg.angle_min or lidar_angle > lidar_msg.angle_max:
            return None

        pass

    def compute_local_person_position(self, angle, distance):
        """
        Convert polar coordinates (angle, distance) to 3D pose in robot's local frame.

        Args:
            angle: Horizontal angle in radians (positive = right, negative = left)
            distance: Distance from LiDAR in meters

        Returns:
            Pose: 3D position with person assumed at ground level, or None if error
        """
        try:
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            pose_3d = Pose()
            pose_3d.position.x = float(x)
            pose_3d.position.y = float(y)
            pose_3d.position.z = 0.0 # Assume ground level

            # Identity quaternion (no rotation)
            pose_3d.orientation.w = 1.0
            pose_3d.orientation.x = 0.0
            pose_3d.orientation.y = 0.0
            pose_3d.orientation.z = 0.0

            return pose_3d

        except Exception as e:
            self.get_logger().error(f'Error computing local person position: {str(e)}')
            return None








