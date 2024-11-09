#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

"""
self.image_data_structure will be our social context model
spin will cause this node to constantly receive processed data from the sensor
update social context model function occurs inside the call_back function. 
This allows it to run over and over again and update the model with new data
Update this depending on our needs
"""

class ProcessedImageListener(Node):
    def __init__(self):
        super().__init__('processed_image_listener')
        self.subscription = self.create_subscription(
            Image,
            '/camera/processed_image',
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        
        # Initialize the data structure for storing images
        self.image_data_structure = []  # Can be replaced with any structure
        
        self.get_logger().info("Processed image listener node with data structure has been started.")

    def image_callback(self, msg):
        try:
            # Convert the processed image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            
            # Update data structure with the new image
            self.update_data_structure(cv_image)
            
            # Log or perform additional operations if needed
            self.get_logger().info(f"Data structure updated. Total frames stored: {len(self.image_data_structure)}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to process received image: {e}")

    def update_data_structure(self, image):
        # Example of adding the latest image to the list
        self.image_data_structure.append(image)
        
        # Optional: Limit the size of the data structure to avoid memory issues
        if len(self.image_data_structure) > 100:  # Limit to 100 images for example
            self.image_data_structure.pop(0)  # Remove oldest image to keep data structure size manageable

def main(args=None):
    rclpy.init(args=args)
    processed_image_listener = ProcessedImageListener()
    
    try:
        rclpy.spin(processed_image_listener)
    except KeyboardInterrupt:
        processed_image_listener.get_logger().info("Shutting down processed image listener with data structure.")
    finally:
        processed_image_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
