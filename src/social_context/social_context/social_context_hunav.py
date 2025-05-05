import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from hunav_msgs.msg import Agents
from collections import deque
import time

import sys
sys.path.append('/root/semaforr_ros2/src/social_context/social_context')
from trajectory_prediction.pipeline import pipeline

# Specifically for HunNavSim - this is for testing the social context model
# Directly uses hunav_msgs/Agents message for human poses

class CoordinateListener(Node):
    def __init__(self):
        super().__init__('coordinate_listener')

        # How often to read the /human_states topic
        self._read_interval = 5.0
        self._last_read_time = 0.0

        # Number of time steps before making a prediction
        self._prediction_steps = 3        

        # Subscribe to /human_states
        self.human_states_subscriber = self.create_subscription(
            Agents,
            '/human_states',
            self.human_states_callback,
            10
        )

        # Publisher for PoseStamped predictions
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/pedestrian_predictions',
            10
        )

        # The Social Context Model: stores pose history + predictions for each agent
        self.social_context_model = {}

        self.get_logger().info("CoordinateListener listening to /human_states")

    def human_states_callback(self, msg):
        timestamp = time.time()
        # If it's been less than _read_interval seconds since last read, skip
        if timestamp - self._last_read_time < self._read_interval:
            return
        self._last_read_time = timestamp

        self.get_logger().info(f"processing /human_states at {timestamp:.1f}")

        # Process the Agents message and add to the social context model
        for agent in msg.agents:
            agent_id = str(agent.id)
            pos = (agent.position.position.x, agent.position.position.y)

            if agent_id not in self.social_context_model:
                self.social_context_model[agent_id] = {
                    "history": deque(maxlen=self._prediction_steps), # automatically removes oldest entry (limit of 3)
                    "timestamps": deque(maxlen=self._prediction_steps), # automatically removes oldest entry (limit of 3) / timestamps can be removed (just for debugging)
                    "predictions": []
                }

            self.social_context_model[agent_id]["history"].append(pos)
            self.social_context_model[agent_id]["timestamps"].append(timestamp)
        
        # Debugging: print the social context model
        print(self.social_context_model)
        
        # Make predictions if we have enough data
        self.make_predictions()

    def make_predictions(self):
        # Convert social_context_model to a list of dictionaries (input format for deep learning model) if we have enough data
        raw_data = []

        for agent_id, data in self.social_context_model.items():
            if len(data["history"]) == self._prediction_steps:
                raw_data.append({
                    'id': agent_id,
                    'coords': list(data["history"])
                })

        # Skip if not enough data
        if not raw_data:
            self.get_logger().info("Not enough data to make predictions")
            return

        # With the processed raw_data, make predictions
        predictions = pipeline([raw_data], self._prediction_steps, self._prediction_steps)

        # Save predictions back to social_context_model
        for agent_id, coords in predictions.items():
            self.social_context_model[agent_id]["predictions"] = coords

            for coord in coords:
                # Publish each prediction as a PoseStamped message
                self.publish_pose_stamped(agent_id, coord)

    def publish_pose_stamped(self, agent_id, coord):
        # Convert the predicted coordinates to a PoseStamped message and publish
        # Change this to anything else if you want to publish something else
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = str(agent_id)
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = coord[0]
        pose_msg.pose.position.y = coord[1]
        pose_msg.pose.position.z = 0.0

        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f"Published prediction for agent {agent_id}")


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CoordinateListener")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
