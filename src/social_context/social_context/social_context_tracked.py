import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from social_context_msgs.msg import TrackedPersonArray
import time

from .trajectory_prediction.pipeline import pipeline

# Uses the real tracker output (TrackedPersonArray on /human_poses_3d_tracked_global) instead of ground-truth /human_states

class TrackedPersonListener(Node):
    def __init__(self):
        super().__init__('tracked_person_listener')

        # How often to run a prediction cycle
        self._read_interval = 5.0
        self._last_read_time = 0.0

        self._prediction_steps = 5

        # Subscribe to the tracker's global-frame output
        self.tracked_sub = self.create_subscription(
            TrackedPersonArray,
            '/human_poses_3d_tracked_global',
            self.tracked_callback,
            10
        )

        # Publisher for PoseStamped predictions
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/pedestrian_predictions_tracked',
            10
        )

        self.get_logger().info("TrackedPersonListener listening to /human_poses_3d_tracked_global")

    def tracked_callback(self, msg):
        timestamp = time.time()
        # If it's been less than _read_interval seconds since last read, skip
        if timestamp - self._last_read_time < self._read_interval:
            return
        self._last_read_time = timestamp

        self.get_logger().info(f"processing /human_poses_3d_tracked_global at {timestamp:.1f}")

        raw_data = []
        for person in msg.people:
            if len(person.history_x) < self._prediction_steps or len(person.history_y) < self._prediction_steps:
                continue  # this tracker hasn't accumulated enough history yet

            coords = list(zip(
                person.history_x[-self._prediction_steps:],
                person.history_y[-self._prediction_steps:]
            ))
            raw_data.append({'id': str(person.id), 'coords': coords})

        # Skip if no tracker has enough history yet
        if not raw_data:
            self.get_logger().info("Not enough tracker history to make predictions")
            return

        self.make_predictions(raw_data)

    def make_predictions(self, raw_data):
        # With the processed raw_data, make predictions
        predictions = pipeline([raw_data], self._prediction_steps, self._prediction_steps)

        for tracker_id, coords in predictions.items():
            for i, coord in enumerate(coords):
                # Publish each prediction as a PoseStamped message
                self.publish_pose_stamped(tracker_id, coord, i)

    def publish_pose_stamped(self, tracker_id, coord, step_index):
        # Convert the predicted coordinates to a PoseStamped message and publish
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = f"{tracker_id}_pred_{step_index}"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = coord[0]
        pose_msg.pose.position.y = coord[1]
        pose_msg.pose.position.z = 0.0

        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f"Published prediction for tracker {tracker_id}, step {step_index}")


def main(args=None):
    rclpy.init(args=args)
    node = TrackedPersonListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TrackedPersonListener")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
