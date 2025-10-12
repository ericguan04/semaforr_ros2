# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import PoseArray, Pose
# from cv_bridge import CvBridge
# import sys
# import os
#
# from social_context.pose_estimation.config import CAMERA_IMAGE_DATA_TOPIC
#
# OPENPOSE_PATH = os.environ.get('OPENPOSE_PATH','')
# sys.path.append(f'{OPENPOSE_PATH}/build/python/openpose')
# import pyopenpose as op
#
#
# class OpenPoseNode(Node):
#     def __init__(self):
#         super().__init__('openpose_node')
#
#         self.declare_parameter('model_folder', f'{OPENPOSE_PATH}/models/')
#         self.declare_parameter('confidence_threshold', 0.1)
#         self.declare_parameter('camera_frame_id', 'camera_rgb_optical_frame')
#
#         params = {
#             "model_folder": self.get_parameter('model_folder').get_parameter_value().string_value,
#             "model_pose": "BODY_25",
#         }
#
#         self.confidence_threshold = self.get_parameter('confidence_threshold').value
#         self.camera_frame_id = self.get_parameter('camera_frame_id').value
#
#         self.get_logger().info('Initializing OpenPose...')
#         self.opWrapper = op.WrapperPython()
#         self.opWrapper.configure(params)
#         self.opWrapper.start()
#         self.get_logger().info('OpenPose initialized successfully')
#
#         self.bridge = CvBridge()
#         self.pose_pub = self.create_publisher(PoseArray, 'human_poses', 10)
#         self.image_sub = self.create_subscription(Image, CAMERA_IMAGE_DATA_TOPIC, self.image_callback, 10)
#
#     def image_callback(self, msg):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#             datum = op.Datum()
#             datum.cvInputData = cv_image
#             self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))
#
#             pose_array = PoseArray()
#             pose_array.header = msg.header
#             pose_array.header.frame_id = self.camera_frame_id
#
#             if datum.poseKeypoints is not None and \
#                     not isinstance(datum.poseKeypoints, (tuple, float, int)) and \
#                     len(datum.poseKeypoints) > 0:
#
#                 for person in datum.poseKeypoints:
#                     if len(person) > 8 and len(person[8]) >= 3:
#                         if person[8, 2] > self.confidence_threshold:
#                             pose = Pose()
#                             pose.position.x = float(person[8, 0])
#                             pose.position.y = float(person[8, 1])
#                             pose.position.z = float(person[8, 2]) # Confidence score
#                             pose_array.poses.append(pose)
#
#             self.pose_pub.publish(pose_array)
#
#         except Exception as e:
#             self.get_logger().error(f'Error processing image: {str(e)}')
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = OpenPoseNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()