#!/usr/bin/env python3
"""
ROS2 node for 2D human pose detection from camera images.

This node is detector-agnostic - it works with any AbstractPoseDetector
implementation (MediaPipe, OpenPose, etc.).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge

from social_context.pose_estimation.config import CAMERA_IMAGE_DATA_TOPIC
from social_context.pose_estimation.src.pose_detectors.abstract import AbstractPoseDetector


class Camera2DPoseDetectionNode(Node):
    """
    Generic ROS2 node for 2D human pose detection.

    Uses dependency injection - accepts any detector that implements
    the AbstractPoseDetector interface.

    Subscribes to: camera images
    Publishes to: /human_poses (PoseArray with 2D detections)
    """

    def __init__(self, detector: AbstractPoseDetector):
        """
        Initializes the pose detection node.

        Args:
            detector (AbstractPoseDetector): AbstractPoseDetector implementation
        """
        super().__init__('camera_2d_pose_detection_node')

        self.detector = detector

        # Parameters
        self.declare_parameter('camera_frame_id', 'rgb_camera_optical_frame')
        self.camera_frame_id = self.get_parameter('camera_frame_id').value

        self.get_logger().info(f'Initializing {self.detector.get_name()} detector...')
        if not self.detector.initialize():
            self.get_logger().error(f'{self.detector.get_name()} initialization failed!')
            raise RuntimeError("Detector initialization failed")

        self.bridge = CvBridge()

        self.pose_pub = self.create_publisher(PoseArray, 'human_poses', 10)
        self.image_sub = self.create_subscription(
            Image,
            CAMERA_IMAGE_DATA_TOPIC,
            self.image_callback,
            10
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Camera 2D Pose Node Ready!')
        self.get_logger().info(f'  Detector: {self.detector.get_name()}')
        self.get_logger().info(f'  Camera frame: {self.camera_frame_id}')
        self.get_logger().info(f'  Subscribing to: {CAMERA_IMAGE_DATA_TOPIC}')
        self.get_logger().info(f'  Publishing to: /human_poses')
        self.get_logger().info('=' * 60)

    def image_callback(self, msg: Image):
        """
        Process incoming camera images.

        Args:
            msg: ROS Image message from camera
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            detections = self.detector.detect(cv_image)

            pose_array = PoseArray()
            pose_array.header = msg.header
            pose_array.header.frame_id = self.camera_frame_id

            for detection in detections:
                pose = Pose()
                pose.position.x = detection.pixel_x
                pose.position.y = detection.pixel_y
                pose.position.z = detection.confidence  # Store confidence in z
                pose_array.poses.append(pose)

            self.pose_pub.publish(pose_array)

            if detections:
                self.get_logger().debug(
                    f'Detected {len(detections)} person(s) '
                    f'[conf: {[f"{d.confidence:.2f}" for d in detections]}]'
                )

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        self.get_logger().info(f'Shutting down {self.detector.get_name()} detector...')
        self.detector.cleanup()
        super().destroy_node()

# =============================================================================
# Entry Points (one for each detector type)
# =============================================================================

def main_mediapipe(args=None):
    """Launch node with MediaPipe detector."""
    from social_context.pose_estimation.src.pose_detectors.mediapipe_pose_detector import MediaPipePoseDetector

    rclpy.init(args=args)

    detector = MediaPipePoseDetector(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        model_complexity=1  # 0=Lite, 1=Full, 2=Heavy
    )

    node = Camera2DPoseDetectionNode(detector)
    detector.logger = node.get_logger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_openpose(args=None):
    """Launch node with OpenPose detector."""
    from social_context.pose_estimation.src.pose_detectors.openpose_pose_detector import OpenPosePoseDetector

    rclpy.init(args=args)

    # Create OpenPose detector
    detector = OpenPosePoseDetector(
        num_gpu=0,  # Set to 1 if GPU available
        model_pose="BODY_25",
        confidence_threshold=0.1
    )

    # Create node with detector
    node = Camera2DPoseDetectionNode(detector)
    detector.logger = node.get_logger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main_mediapipe()




