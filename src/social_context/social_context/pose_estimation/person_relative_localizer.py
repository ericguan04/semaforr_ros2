#!/usr/bin/env python3
from config import CAMERA_INFO_TOPIC, LIDAR_TOPIC, OPENPOSE_OUTPUT_TOPIC, MAX_SYNC_DELAY, PERSON_RELATIVE_LOCALIZER_OUTPUT_TOPIC
import rclpy
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
        self.camera_info = msg
        self.get_logger().info('Camera information received')

    def person_relative_localizer_callback(self, pose_msg, lidar_msg):
        pass
