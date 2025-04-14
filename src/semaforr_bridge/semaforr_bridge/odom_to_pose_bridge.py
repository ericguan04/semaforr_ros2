import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomToPoseBridge(Node):
    def __init__(self):
        super().__init__('odom_to_pose_bridge')

        # Subscribe to Odometry from robot controller
        self.subscription = self.create_subscription(
            Odometry,
            '/mobile_base_controller/odom',
            self.odom_callback,
            10
        )

        # Publish PoseStamped for SemaFORR
        self.publisher = self.create_publisher(
            PoseStamped,
            '/pose',
            10
        )

    def odom_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header  # preserves timestamp and frame_id (e.g., 'odom')
        pose_stamped.pose = msg.pose.pose  # extract just the pose

        self.publisher.publish(pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
