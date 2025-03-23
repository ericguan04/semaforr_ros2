#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import sys
import os

"""
This node depends on OpenPose being installed on your system at the path specified in the OPENPOSE_PATH environment variable (or the default path below).

OpenPose comes with a Python API module that this node interacts with.   

Additionally throughout the code I added comments for my own understanding.
"""

OPENPOSE_PATH = os.environ.get('OPENPOSE_PATH', '') # Path to openpose installation. TODO: Change to actual expected installation
sys.path.append(f'{OPENPOSE_PATH}/build/python/openpose')
import pyopenpose as op

class OpenPoseNode(Node):
    def __init__(self):
        super().__init__('openpose_node')

        # Parameters
        self.declare_parameter('model_folder', f'{OPENPOSE_PATH}/models/')

        # Initialize OpenPose
        params = {
        "model_folder": self.get_parameter('model_folder').get_parameter_value().string_value
        }

        self.get_logger().info('Initializing OpenPose...')
        self.opWrapper = op.WrapperPython()  # Creates the openpose wrapper object
        self.opWrapper.configure(params)   # Configures it with the parameters
        self.opWrapper.start()    # Starts the openpose processing
        self.get_logger().info('OpenPose initialized successfully')

        # ROS interfaces
        self.bridge = CvBridge()
        self.pose_pub = self.create_publisher(PoseArray, 'human_poses', 10)
        self.image_sub = self.create_subscription(
            Image, 
            'camera/image_raw',   # TODO: Change topic name to the actual topic we use
            self.image_callback, 
            10
        )

    def image_callback(self, msg):
        """
        Function to establish communication pattern: subscribe to images, process them, publish poses.
        """
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            datum = op.Datum()  # Create datum object which is OpenPose's container for data
            datum.cvInputData = cv_image  # Set the input image 

            self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))  # emplaceAndPop method processes the image and gets results stored in datum object

            # Create pose message
            pose_array = PoseArray()
            pose_array.header = msg.header

            # If people were detected
            if datum.poseKeypoints is not None and not isinstance(datum.poseKeypoints, tuple):
                for person in datum.poseKeypoints:
                    # Use mid-hip as person position (keypoint 8 in COCO model)
                    if person[8, 2] > 0.0:  # If confidence is good
                        pose = Pose()
                        pose.position.x = float(person[8, 0])
                        pose.position.y = float(person[8, 1])
                        pose.position.z = 0.0
                        pose_array.poses.append(pose)


            # Publish pose array
            self.pose_pub.publish(pose_array)
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = OpenPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()