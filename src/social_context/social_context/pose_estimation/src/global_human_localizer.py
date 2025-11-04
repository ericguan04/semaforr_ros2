#!/usr/bin/env python3
"""
Global Human Localizer Node

Transforms human positions from robot-relative coordinates (local)
to global map coordinates using TF transforms.

Input:  /human_poses_3d (PoseArray in robot frame)
Output: /human_poses_global (PoseArray in map frame)
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseArray, PoseStamped
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs

class GlobalHumanLocalizer(Node):
    """
    Transforms human positions from robot frame to global map frame.

    Uses TF2 to look up the robot's position in the map and transform
    each detected person's position accordingly.
    """

    def __init__(self):
        super().__init__('global_human_localizer')

        self.declare_parameter('local_poses_topic', '/human_poses_3d')
        self.declare_parameter('global_poses_topic', '/human_poses_3d_global')
        self.declare_parameter('robot_frame', 'base_laser_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tf_timeout', 0.5)
        self.declare_parameter('publish_rate_limit', 10.0)  # Hz

        local_topic = self.get_parameter('local_poses_topic').value
        global_topic = self.get_parameter('global_poses_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        self.publish_rate_limit = self.get_parameter('publish_rate_limit').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.global_pose_pub = self.create_publisher(PoseArray, global_topic, 10)
        self.local_poses_sub = self.create_subscription(PoseArray, local_topic, self.local_poses_callback, 10)

        self.last_publish_time = self.get_clock().now()
        self.min_publish_interval = 1.0 / self.publish_rate_limit

        self.transform_success_count = 0
        self.transform_fail_count = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('Global Human Localizer Node Ready!')
        self.get_logger().info(f'  Robot frame: {self.robot_frame}')
        self.get_logger().info(f'  Map frame: {self.map_frame}')
        self.get_logger().info(f'  Subscribing to: {local_topic}')
        self.get_logger().info(f'  Publishing to: {global_topic}')
        self.get_logger().info('=' * 60)

    def local_poses_callback(self, msg: PoseArray):
        """
        Callback for local pose detections.
        Transforms each pose from robot frame to map frame.
        """
        # Rate limiting
        current_time = self.get_clock().now()
        time_since_last_publish = (current_time - self.last_publish_time).nanoseconds / 1e9

        if time_since_last_publish < self.min_publish_interval:
            return  # Skip this message to avoid overwhelming the system

        if len(msg.poses) == 0:
            return

        try:
            timestamp = Time.from_msg(msg.header.stamp)

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.robot_frame,
                    # rclpy.time.Time(),
                    timestamp,
                    timeout=rclpy.duration.Duration(seconds=self.tf_timeout)
                )
            except TransformException as ex:
                self.get_logger().warn(
                    f'Could not transform {self.robot_frame} to {self.map_frame}: {ex}',
                    throttle_duration_sec=5.0
                )
                self.transform_fail_count += 1
                return

            global_poses = PoseArray()
            global_poses.header.stamp = msg.header.stamp
            global_poses.header.frame_id = self.map_frame

            for local_pose in msg.poses:
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = msg.header.stamp
                pose_stamped.header.frame_id = self.robot_frame
                pose_stamped.pose = local_pose

                global_pose_stamped = tf2_geometry_msgs.do_transform_pose_stamped(
                    pose_stamped,
                    transform
                )

                global_poses.poses.append(global_pose_stamped.pose)

            self.global_pose_pub.publish(global_poses)
            self.last_publish_time = current_time
            self.transform_success_count += 1

            self.get_logger().debug(
                f'Transformed {len(global_poses.poses)} person(s) to global frame'
            )

            # Log statistics periodically
            if self.transform_success_count % 100 == 0:
                total = self.transform_success_count + self.transform_fail_count
                success_rate = 100.0 * self.transform_success_count / total if total > 0 else 0
                self.get_logger().info(
                    f'Transform statistics: {self.transform_success_count} success, '
                    f'{self.transform_fail_count} failed ({success_rate:.1f}% success rate)'
                )

        except Exception as e:
            self.get_logger().error(f'Error transforming poses: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = GlobalHumanLocalizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()







