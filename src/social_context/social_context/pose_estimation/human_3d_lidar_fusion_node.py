#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import math
from message_filters import ApproximateTimeSynchronizer, Subscriber


class Human3DLidarFusionNode(Node):
    """
    Node that fuses 2D human pose detections with LiDAR data to produce 3D human positions.
    Designed for real-world robots with RGB cameras + LiDAR (no depth camera required).
    """

    def __init__(self):
        super().__init__('human_3d_lidar_fusion')

        # Parameters
        self.declare_parameter('camera_info_topic', '/rgb_camera_frame_sensor/camera_info')
        self.declare_parameter('lidar_topic', '/scan_raw')
        self.declare_parameter('poses_2d_topic', 'human_poses')
        self.declare_parameter('max_sync_delay', 0.1)  # 100ms max delay between messages

        self.camera_info = None

        # Publishers
        self.pose_3d_pub = self.create_publisher(PoseArray, 'human_poses_3d', 10)

        # Camera info subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').get_parameter_value().string_value,
            self.camera_info_callback,
            10
        )

        # Synchronized subscribers for poses and LiDAR scan
        self.pose_sub = Subscriber(
            self,
            PoseArray,
            self.get_parameter('poses_2d_topic').get_parameter_value().string_value
        )
        self.lidar_sub = Subscriber(
            self,
            LaserScan,
            self.get_parameter('lidar_topic').get_parameter_value().string_value
        )

        # Synchronize topics
        max_delay = self.get_parameter('max_sync_delay').get_parameter_value().double_value
        self.sync = ApproximateTimeSynchronizer(
            [self.pose_sub, self.lidar_sub],
            queue_size=10,
            slop=max_delay
        )
        self.sync.registerCallback(self.fusion_callback)

        self.get_logger().info('Human 3D LiDAR Fusion Node started')

    def camera_info_callback(self, msg):
        """Store camera calibration parameters"""
        self.camera_info = msg
        self.get_logger().info('Camera calibration received')

    def fusion_callback(self, pose_msg, lidar_msg):
        """
        Fuse 2D human poses with LiDAR data to get 3D positions.
        """
        if self.camera_info is None:
            self.get_logger().warn("No camera calibration received yet")
            return

        try:
            # Create output message
            poses_3d = PoseArray()
            poses_3d.header = pose_msg.header
            poses_3d.header.frame_id = 'base_laser_link'  # LiDAR frame

            # Process each detected person
            for pose_2d in pose_msg.poses:
                # Convert 2D pixel coordinates to camera ray
                pixel_x = pose_2d.position.x
                pixel_y = pose_2d.position.y

                # Convert pixel to horizontal camera angle
                camera_angle = self.pixel_to_camera_angle(pixel_x)

                # Find corresponding LiDAR range measurement
                lidar_range = self.get_lidar_range_at_angle(camera_angle, lidar_msg)

                if lidar_range is not None:
                    # Convert to 3D point in LiDAR frame
                    pose_3d = self.create_3d_pose(camera_angle, lidar_range)

                    if pose_3d is not None:
                        poses_3d.poses.append(pose_3d)

            # Publish 3D poses
            self.pose_3d_pub.publish(poses_3d)

            if len(poses_3d.poses) > 0:
                self.get_logger().info(f'Published {len(poses_3d.poses)} 3D human poses')

        except Exception as e:
            self.get_logger().error(f'Error in LiDAR fusion: {str(e)}')

    def pixel_to_camera_angle(self, pixel_x):
        """
        Convert pixel x-coordinate to horizontal camera angle.
        Assumes camera and LiDAR are roughly aligned horizontally.
        """
        # Camera intrinsic parameters
        fx = self.camera_info.k[0]  # Focal length x
        cx = self.camera_info.k[2]  # Principal point x

        # Convert pixel to horizontal angle (radians)
        # Positive angle = right, negative = left
        angle = math.atan2(pixel_x - cx, fx)

        return angle

    def get_lidar_range_at_angle(self, target_angle, lidar_msg):
        """
        Get LiDAR range measurement closest to the target camera angle.

        Args:
            target_angle: Horizontal angle from camera (radians)
            lidar_msg: LaserScan message

        Returns:
            Range value (meters) or None if no valid measurement
        """
        # Convert camera angle to LiDAR angle coordinate system
        # This assumes camera and LiDAR are roughly aligned
        # You may need to adjust this transformation based on your robot setup
        lidar_angle = target_angle

        # Check if angle is within LiDAR field of view
        if lidar_angle < lidar_msg.angle_min or lidar_angle > lidar_msg.angle_max:
            return None

        # Find corresponding LiDAR ray index
        angle_from_min = lidar_angle - lidar_msg.angle_min
        ray_index = int(round(angle_from_min / lidar_msg.angle_increment))

        # Ensure index is valid
        ray_index = max(0, min(ray_index, len(lidar_msg.ranges) - 1))

        # Get range value
        range_val = lidar_msg.ranges[ray_index]

        # Check if range is valid
        if (lidar_msg.range_min <= range_val <= lidar_msg.range_max and
                not math.isinf(range_val) and not math.isnan(range_val)):
            return range_val

        # If exact ray is invalid, try nearby rays
        for offset in [1, -1, 2, -2]:
            nearby_index = ray_index + offset
            if 0 <= nearby_index < len(lidar_msg.ranges):
                nearby_range = lidar_msg.ranges[nearby_index]
                if (lidar_msg.range_min <= nearby_range <= lidar_msg.range_max and
                        not math.isinf(nearby_range) and not math.isnan(nearby_range)):
                    return nearby_range

        return None

    def create_3d_pose(self, angle, distance):
        """
        Create 3D pose from horizontal angle and distance.

        Args:
            angle: Horizontal angle (radians)
            distance: Distance from LiDAR (meters)

        Returns:
            Pose message with 3D position
        """
        try:
            # Convert polar to Cartesian coordinates
            # LiDAR coordinate system: x=forward, y=left, z=up
            x = distance * math.cos(angle)  # Forward distance
            y = distance * math.sin(angle)  # Lateral distance
            z = 0.0  # Assume humans are on ground plane

            # Create pose message
            pose_3d = Pose()
            pose_3d.position.x = float(x)
            pose_3d.position.y = float(y)
            pose_3d.position.z = float(z)

            # Set orientation (no orientation info from 2D pose)
            pose_3d.orientation.w = 1.0
            pose_3d.orientation.x = 0.0
            pose_3d.orientation.y = 0.0
            pose_3d.orientation.z = 0.0

            return pose_3d

        except Exception as e:
            self.get_logger().error(f'Error creating 3D pose: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = Human3DLidarFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        self.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()