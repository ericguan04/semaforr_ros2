#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np

"""
3D Pose Localizer Node using MediaPipe.

This node processes camera images and outputs 3D human poses directly using MediaPipe's 
BlazePose model. The poses include real-world 3D coordinates suitable for robot navigation 
and social interaction applications.

MediaPipe provides 33 3D landmarks with coordinates in meters, with origin at the center 
between hips. This is perfect for spatial reasoning in robotics applications.

Prerequisites:
- pip install mediapipe opencv-python

Topics:
- Subscribes to: /camera/image_raw (sensor_msgs/Image)
- Publishes to: /human_poses_3d (geometry_msgs/PoseArray)
"""


class PoseLocalizer3DNode(Node):
    def __init__(self):
        super().__init__('pose_localizer_3d')

        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('model_complexity', 1)
        self.declare_parameter('smooth_landmarks', True)
        self.declare_parameter('enable_segmentation', False)
        self.declare_parameter('publish_all_landmarks', False)  # For debugging/visualization

        min_detection_conf = self.get_parameter('min_detection_confidence').get_parameter_value().double_value
        min_tracking_conf = self.get_parameter('min_tracking_confidence').get_parameter_value().double_value
        model_complexity = self.get_parameter('model_complexity').get_parameter_value().integer_value
        smooth_landmarks = self.get_parameter('smooth_landmarks').get_parameter_value().bool_value
        enable_segmentation = self.get_parameter('enable_segmentation').get_parameter_value().bool_value
        self.publish_all_landmarks = self.get_parameter('publish_all_landmarks').get_parameter_value().bool_value

        # Initialize MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,  # Process video stream
            model_complexity=model_complexity,  # Balance speed vs accuracy
            smooth_landmarks=smooth_landmarks,  # Temporal smoothing
            enable_segmentation=enable_segmentation,  # Background segmentation
            min_detection_confidence=min_detection_conf,
            min_tracking_confidence=min_tracking_conf
        )

        self.mp_drawing = mp.solutions.drawing_utils

        self.bridge = CvBridge()
        self.pose_3d_pub = self.create_publisher(PoseArray, 'human_poses_3d', 10)

        if self.publish_all_landmarks:
            self.landmarks_pub = self.create_publisher(PoseArray, 'human_landmarks_3d', 10)

        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Landmark indices for key body parts
        self.LEFT_HIP = self.mp_pose.PoseLandmark.LEFT_HIP.value
        self.RIGHT_HIP = self.mp_pose.PoseLandmark.RIGHT_HIP.value
        self.LEFT_SHOULDER = self.mp_pose.PoseLandmark.LEFT_SHOULDER.value
        self.RIGHT_SHOULDER = self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value
        self.NOSE = self.mp_pose.PoseLandmark.NOSE.value

        self.get_logger().info('3D Pose Localizer Node initialized successfully')
        self.get_logger().info(f'Model complexity: {model_complexity} (0=lite, 1=full, 2=heavy)')
        self.get_logger().info(f'Detection confidence: {min_detection_conf}')
        self.get_logger().info(f'Tracking confidence: {min_tracking_conf}')

    def image_callback(self, msg):
        """Process incoming camera images and estimate 3D poses"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            results = self.pose.process(rgb_image)

            pose_array = PoseArray()
            pose_array.header = msg.header
            pose_array.header.frame_id = "camera_frame"

            if results.pose_landmarks and results.pose_world_landmarks:
                main_pose = self.extract_main_pose(results.pose_world_landmarks)
                if main_pose:
                    pose_array.poses.append(main_pose)

                if self.publish_all_landmarks:
                    all_landmarks = self.extract_all_landmarks(results.pose_world_landmarks, msg.header)
                    self.landmarks_pub.publish(all_landmarks)

            # Publish main poses
            self.pose_3d_pub.publish(pose_array)

            # Log detection status
            if results.pose_landmarks:
                self.get_logger().debug('3D pose detected and published')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def extract_main_pose(self, pose_world_landmarks):
        """
        Extract the main pose representation for robot navigation.
        Uses hip center as the primary position for the person.
        """
        try:
            landmarks = pose_world_landmarks.landmark

            left_hip = landmarks[self.LEFT_HIP]
            right_hip = landmarks[self.RIGHT_HIP]

            if left_hip.visibility < 0.5 or right_hip.visibility < 0.5:
                return None

            center_x = (left_hip.x + right_hip.x) / 2
            center_y = (left_hip.y + right_hip.y) / 2
            center_z = (left_hip.z + right_hip.z) / 2

            pose = Pose()

            pose.position = Point(
                x=float(center_x),  # Meters
                y=float(center_y),  # Meters
                z=float(center_z)  # Meters (depth from camera)
            )

            orientation = self.calculate_body_orientation(landmarks)
            pose.orientation = orientation

            return pose

        except Exception as e:
            self.get_logger().error(f'Error extracting main pose: {str(e)}')
            return None

    def calculate_body_orientation(self, landmarks):
        """
        Calculate body orientation from shoulder line.
        Returns a quaternion representing the person's facing direction.
        """
        try:
            left_shoulder = landmarks[self.LEFT_SHOULDER]
            right_shoulder = landmarks[self.RIGHT_SHOULDER]
            left_hip = landmarks[self.LEFT_HIP]
            right_hip = landmarks[self.RIGHT_HIP]

            # Calculate shoulder vector (left to right)
            shoulder_vec = np.array([
                right_shoulder.x - left_shoulder.x,
                right_shoulder.y - left_shoulder.y,
                right_shoulder.z - left_shoulder.z
            ])

            # Calculate hip vector (left to right)
            hip_vec = np.array([
                right_hip.x - left_hip.x,
                right_hip.y - left_hip.y,
                right_hip.z - left_hip.z
            ])

            body_vec = (shoulder_vec + hip_vec) / 2

            yaw = np.arctan2(body_vec[1], body_vec[0])

            quat = Quaternion()
            quat.z = float(np.sin(yaw / 2))
            quat.w = float(np.cos(yaw / 2))
            quat.x = 0.0
            quat.y = 0.0

            return quat

        except Exception as e:
            self.get_logger().debug(f'Could not calculate orientation: {str(e)}')

            quat = Quaternion()
            quat.w = 1.0
            return quat

    def extract_all_landmarks(self, pose_world_landmarks, header):
        """
        Extract all 33 landmarks for visualization or detailed analysis.
        Each landmark becomes a pose in the array.
        """
        landmarks_array = PoseArray()
        landmarks_array.header = header
        landmarks_array.header.frame_id = "camera_frame"

        for i, landmark in enumerate(pose_world_landmarks.landmark):
            if landmark.visibility > 0.1: 
                pose = Pose()
                pose.position = Point(
                    x=float(landmark.x),
                    y=float(landmark.y),
                    z=float(landmark.z)
                )
                pose.orientation.w = 1.0
                landmarks_array.poses.append(pose)

        return landmarks_array

    def get_landmark_name(self, index):
        """Helper function to get landmark name from index (for debugging)"""
        try:
            return self.mp_pose.PoseLandmark(index).name
        except:
            return f"LANDMARK_{index}"


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PoseLocalizer3DNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()