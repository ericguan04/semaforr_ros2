#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraListener(Node):
    def __init__(self):
        super().__init__('camera_listener')
        
        """
        TEMPORARY CODE
        Could be camera, but could also use LIDAR
        """
        # Subscription to the raw camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adjust topic name as needed
            self.image_callback,
            10
        )
        
        # Publisher for the processed image topic
        """
        TEMPORARY CODE
        Will send pedestrian ID and the (x, y) coordinates
        """
        self.publisher = self.create_publisher(
            Image,
            '/camera/processed_image',  # Topic name for processed image
            10
        )
        
        self.bridge = CvBridge()
        self.get_logger().info("Camera listener and processor node has been started.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process the image (for example, convert to grayscale)
            processed_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            self.get_logger().info("Image processed!")
            
            # Publish the processed image
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="mono8")
            self.publisher.publish(processed_msg)
            
            # Optionally display the processed image
            cv2.imshow("Processed Camera Feed", processed_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    camera_listener = CameraListener()
    
    try:
        rclpy.spin(camera_listener)
    except KeyboardInterrupt:
        camera_listener.get_logger().info("Shutting down camera listener and processor.")
    finally:
        camera_listener.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
