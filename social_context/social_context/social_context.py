#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from collections import defaultdict # might need to use default dict
from trajectory_prediction.pipeline import pipeline

"""
Defines ROS2 node for listening to pedestrian coordinates and updating a social context model:
Classes:
    CoordinateListener(Node): A ROS2 node that subscribes to a topic to receive pedestrian coordinates,
                              updates a social context model, and makes trajectory predictions.
Functions:
    main(args=None): Initializes the ROS2 system, creates a CoordinateListener node, and keeps it spinning.

CoordinateListener:
    Methods:
        __init__(self): Initializes the CoordinateListener node, sets up the subscription, and initializes the social context model.
        
        coordinate_callback(self, msg): Callback function that processes received coordinates and updates the social context model.
            msg: The received message containing pedestrian ID and coordinates.
        
        update_social_context(self, id, coordinate): Updates the social context model with new pedestrian coordinates.
            id: The pedestrian ID.
            coordinate: The (x, y) coordinates of the pedestrian.
        
        make_predictions(self): Generates trajectory predictions based on the social context model.
        
        social_context_to_raw_data(social_context_model): Converts the social context model to raw data format for the prediction pipeline.
            social_context_model: The social context model containing pedestrian IDs, coordinates, and predictions.
"""

class CoordinateListener(Node):
    def __init__(self):
        super().__init__('coordinate_listener')
        
        # Subscriber node will receive pedestrian ID and (x, y) coordinates
        # Update this when sensor_listener is ready
        self.subscription = self.create_subscription(
            Image,
            '/camera/processed_image',
            # Every time a new message is received, call the image_callback function
            self.coordinate_callback,
            10
        )
        
        self.bridge = CvBridge()
        
        # Social context model will store pedestrian data in the environment
        self.social_context_model = {}
        
        self.get_logger().info("Coordinate listener node and social context model is running")

    def coordinate_callback(self, msg):
        try:
            # Unpack the msg to get pedestrian ID and (x, y) coordinates
            id, obs_coordinate = msg.id, msg.obs_coordinate
            
            # Update the social context model
            self.update_social_context(id, obs_coordinate)
            
            # Log info
            self.get_logger().info(f"Social context model updated")
            
        except Exception as e:
            self.get_logger().error(f"Failed to process received coordinates: {e}")

    def update_social_context(self, id, coordinate):
        if id not in self.social_context_model:
            # Initialize a new entry for the pedestrian ID
            self.social_context_model[id] = {"history": [], "predictions": []}

        # Add ID and coordinate to the social context model
        self.social_context_model[id]["history"].append(coordinate)
        
        # Need some way to know when to make predictions: needs some discussion.

    def make_predictions(self):
        # Could call this in the main function actually
        
        # Turn data into the correct format for the prediction pipeline
        raw_data = self.social_context_to_raw_data(self.social_context_model)
        traj_preds = pipeline(raw_data, 3, 3)

        for key, value in traj_preds.items():
            for coord in value:
                self.social_context_model[key]["predictions"].append(coord)

    def social_context_to_raw_data(social_context_model):
        raw_data = []
        for key, value in social_context_model.items():
            raw_data.append({'id': key, 'coords': value['history']})
        return [raw_data]


def main(args=None):
    rclpy.init(args=args)
    coordinate_listener = CoordinateListener()
    
    try:
        rclpy.spin(coordinate_listener)
    except KeyboardInterrupt:
        coordinate_listener.get_logger().info("Shutting down coordinate listener and social context model")
    finally:
        coordinate_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()