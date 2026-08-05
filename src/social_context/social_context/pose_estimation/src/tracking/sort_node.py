#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from .sort_tracker import SortTracker
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .feature_extractor import FeatureExtractor
from social_context_msgs.msg import TrackedPerson, TrackedPersonArray, LocalizedPersonArray

class SortNode(Node):
    """
    ROS2 node for SORT tracking.

    Subscribes to: /human_poses_3d_global (LocalizedPersonArray with 3D detections)
    Publishes to: /tracked_human_poses_global (TrackedPersonArray with tracked 3D positions)
    """

    def __init__(self):
        """
        Initialize the SORT node.
        """

        super().__init__('sort_node')

        # Parameters
        self.declare_parameter('distance_threshold', 2.0)
        self.declare_parameter('missed_threshold', 15)
        self.declare_parameter('tentative_missed_threshold', 7)
        self.declare_parameter('hits_threshold', 2)
        self.declare_parameter('combined_cost_threshold', 0.8)

        distance_threshold = self.get_parameter('distance_threshold').value
        missed_threshold = self.get_parameter('missed_threshold').value
        tentative_missed_threshold = self.get_parameter('tentative_missed_threshold').value
        hits_threshold = self.get_parameter('hits_threshold').value
        combined_cost_threshold = self.get_parameter('combined_cost_threshold').value

        self.tracker = SortTracker(
            distance_threshold = distance_threshold, 
            missed_threshold = missed_threshold, 
            tentative_missed_threshold = tentative_missed_threshold, 
            hits_threshold = hits_threshold, 
            combined_cost_threshold = combined_cost_threshold)

        self.feature_extractor = FeatureExtractor()
        self.bridge = CvBridge()
        self.latest_image = None

        self.pose_sub = self.create_subscription(
            LocalizedPersonArray,
            'human_poses_3d_global',
            self.pose_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/rgb_camera_frame_sensor/image_raw',
            self.image_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            TrackedPersonArray, 
            'human_poses_3d_tracked_global', 
            10
        )



        self.get_logger().info('=' * 60)
        self.get_logger().info(f'SORT Node Ready!')
        self.get_logger().info(f'  Distance threshold: {distance_threshold}')
        self.get_logger().info(f'  Missed threshold: {missed_threshold}')
        self.get_logger().info(f'  Tentative missed threshold: {tentative_missed_threshold}')
        self.get_logger().info(f'  Hits threshold: {hits_threshold}')
        self.get_logger().info(f'  Combined cost threshold: {combined_cost_threshold}')
        self.get_logger().info(f'  Subscribing to: /human_poses_3d_global')
        self.get_logger().info(f'  Subscribing to: /rgb_camera_frame_sensor/image_raw')
        self.get_logger().info(f'  Publishing to: /human_poses_3d_tracked_global')
        self.get_logger().info('=' * 60)


    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            self.latest_image = None


    def pose_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        detections = []
        for person in msg.people:
            x = person.x
            y = person.y
            confidence = person.confidence
            orientation = (
                person.orientation_x,
                person.orientation_y,
                person.orientation_z,
                person.orientation_w
            )
            pixel_x = person.pixel_x
            pixel_y = person.pixel_y
            detections.append((x, y, confidence, orientation, pixel_x, pixel_y))

        confirmed_tracks = self.tracker.update(detections, self.latest_image, self.feature_extractor, timestamp)

        tracked_msg = TrackedPersonArray()
        tracked_msg.header = msg.header

        for track in confirmed_tracks:
            person = TrackedPerson()
            person.id = int(track['id'])
            person.confidence = float(track['confidence'])
            person.x = float(track['position'][0])
            person.y = float(track['position'][1])

            person.orientation_x = float(track['orientation'][0])
            person.orientation_y = float(track['orientation'][1])
            person.orientation_z = float(track['orientation'][2])
            person.orientation_w = float(track['orientation'][3])

            person.history_x = [float(p[0]) for p in track['history']]
            person.history_y = [float(p[1]) for p in track['history']]

            tracked_msg.people.append(person)

        self.pose_pub.publish(tracked_msg)

        if confirmed_tracks:
            ids = [track['id'] for track in confirmed_tracks]
            self.get_logger().info(
                f'Tracking {len(confirmed_tracks)} person(s) with IDs: {ids}'
            )
        else:
            self.get_logger().debug('No confirmed tracks in this frame.')
    

def main(args=None):
    rclpy.init(args=args)
    node = SortNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()