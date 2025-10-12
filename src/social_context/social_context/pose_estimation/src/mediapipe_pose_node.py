# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import PoseArray, Pose
# from cv_bridge import CvBridge
# import cv2
# import mediapipe as mp
# import numpy as np
#
# from social_context.pose_estimation.config import CAMERA_IMAGE_DATA_TOPIC
#
# class MediaPipePoseNode(Node):
#     """
#     ROS2 node for human pose detection using Google MediaPipe.
#
#     Alternative to OpenPose - can be much faster and works well on CPU!
#     Publishes 2D pose detections (hip position + confidence) for fusion with LiDAR.
#     """
#
#     def __init__(self):
#         super().__init__('mediapipe_pose_node')
#
#         self.declare_parameter('min_detection_confidence', 0.5)
#         self.declare_parameter('min_tracking_confidence', 0.5)
#         self.declare_parameter('model_complexity', 1)  # 0=Lite, 1=Full, 2=Heavy
#         self.declare_parameter('camera_frame_id', 'camera_rgb_optical_frame')
#
#         min_detection_conf = self.get_parameter('min_detection_confidence').value
#         min_tracking_conf = self.get_parameter('min_tracking_confidence').value
#         model_complexity = self.get_parameter('model_complexity').value
#         self.camera_frame_id = self.get_parameter('camera_frame_id').value
#
#         self.get_logger().info('Initializing MediaPipe Pose...')
#
#         self.mp_pose = mp.solutions.pose
#         self.pose = self.mp_pose.Pose(
#             static_image_mode=False,  # Video mode for better tracking
#             model_complexity=model_complexity,
#             smooth_landmarks=True,
#             min_detection_confidence=min_detection_conf,
#             min_tracking_confidence=min_tracking_conf
#         )
#
#         self.get_logger().info(
#             f'MediaPipe initialized: '
#             f'model_complexity={model_complexity}, '
#             f'min_detection_conf={min_detection_conf}, '
#             f'min_tracking_conf={min_tracking_conf}'
#         )
#
#         self.bridge = CvBridge()
#
#         self.pose_pub = self.create_publisher(PoseArray, 'human_poses', 10)
#         self.image_sub = self.create_subscription(
#             Image,
#             CAMERA_IMAGE_DATA_TOPIC,
#             self.image_callback,
#             10
#         )
#
#         self.get_logger().info(f'Subscribed to: {CAMERA_IMAGE_DATA_TOPIC}')
#         self.get_logger().info('Publishing to: /human_poses')
#         self.get_logger().info('MediaPipe Pose Node ready!')
#
#     def image_callback(self, msg):
#         """Process incoming images and detect human poses."""
#         try:
#             # Convert ROS Image to OpenCV format
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#             rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) # Convert BGR to RGB (MediaPipe uses RGB)
#
#             results = self.pose.process(rgb_image)
#
#             pose_array = PoseArray()
#             pose_array.header = msg.header
#             pose_array.header.frame_id = self.camera_frame_id
#
#             # Extract poses if detected
#             if results.pose_landmarks:
#                 # MediaPipe detects one person at a time in Pose mode
#                 # For multi-person, would need to use BlazePose or run detection multiple times
#
#                 landmarks = results.pose_landmarks.landmark
#                 image_height, image_width = cv_image.shape[:2]
#
#                 # Get hip center position (average of left and right hip)
#                 # MediaPipe landmark indices:
#                 # 23 = LEFT_HIP, 24 = RIGHT_HIP
#                 left_hip = landmarks[23]
#                 right_hip = landmarks[24]
#
#                 if left_hip.visibility > 0.5 and right_hip.visibility > 0.5:
#                     hip_x = ((left_hip.x + right_hip.x) / 2) * image_width
#                     hip_y = ((left_hip.y + right_hip.y) / 2) * image_height
#
#                     confidence = (left_hip.visibility + right_hip.visibility) / 2
#
#                     pose = Pose()
#                     pose.position.x = float(hip_x)
#                     pose.position.y = float(hip_y)
#                     pose.position.z = float(confidence)
#
#                     pose_array.poses.append(pose)
#
#                     self.get_logger().debug(
#                         f'Detected person at pixel ({hip_x:.0f}, {hip_y:.0f}), '
#                         f'confidence={confidence:.2f}'
#                     )
#
#             self.pose_pub.publish(pose_array)
#
#         except Exception as e:
#             self.get_logger().error(f"Error processing image: {str(e)}")
#
#     def destroy_node(self):
#         """Clean up MediaPipe resources."""
#         self.pose.close()
#         super().destroy_node()
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = MediaPipePoseNode()
#
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#
#
#
#
#
#
#
