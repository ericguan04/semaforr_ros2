#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_listener')
        
        # Subscribe to the LiDAR scan data topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Common topic name for 2D LiDAR, adjust as needed
            self.lidar_callback,
            10
        )
        
        # Publisher for observed positions using PoseStamped
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/pedestrian_observations',
            10
        )
        
        self.get_logger().info("LiDAR listener node has been started.")

    def lidar_callback(self, msg):
        try:
            # Process the LiDAR data here
            # For example, finding the minimum distance in the scan ranges
            min_distance = min(msg.ranges)
            self.get_logger().info(f"Closest object distance: {min_distance} meters")
            
            # Placeholder for publishing processed data
            # Here we could modify the LaserScan message or publish custom data
            # Example: simply republishing the received data
            observed_position = self.calculate_position(msg)
            self.publisher.publish(observed_position)

        except Exception as e:
            self.get_logger().error(f"Failed to process LiDAR data: {e}")
    
    def calculate_position(self, msg):
        # Placeholder for calculating position based on LiDAR data
        pass

def main(args=None):
    rclpy.init(args=args)
    lidar_listener = LidarListener()
    
    try:
        rclpy.spin(lidar_listener)
    except KeyboardInterrupt:
        lidar_listener.get_logger().info("Shutting down LiDAR listener.")
    finally:
        lidar_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
