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

        # Internal model: stores pose history + predictions per agent
        self.social_context_model = {}

        # Timer: checks every 2s if it's time to predict
        self.create_timer(2.0, self.make_predictions)

        self.get_logger().info("CoordinateListener listening to /human_states")

    def human_states_callback(self, msg):
        timestamp = time.time()

        for agent in msg.agents:
            agent_id = str(agent.id)
            pos = (agent.position.position.x, agent.position.position.y)

            if agent_id not in self.social_context_model:
                self.social_context_model[agent_id] = {
                    "history": deque(maxlen=3),
                    "timestamps": deque(maxlen=3),
                    "predictions": []
                }

            self.social_context_model[agent_id]["history"].append(pos)
            self.social_context_model[agent_id]["timestamps"].append(timestamp)

    def make_predictions(self):
        raw_data = []

        for agent_id, data in self.social_context_model.items():
            if len(data["history"]) == 3 and self.valid_intervals(data["timestamps"]):
                raw_data.append({
                    'id': agent_id,
                    'coords': list(data["history"])
                })

        if not raw_data:
            return  # No valid sequences yet

        # Call DL model here (replace with your real pipeline)
        predictions = pipeline([raw_data], input_len=3, pred_len=3)

        for agent_id, coords in predictions.items():
            self.social_context_model[agent_id]["predictions"] = coords

            for coord in coords:
                self.publish_pose_stamped(agent_id, coord)

    def valid_intervals(self, timestamps):
        """Ensure samples are ~2s apart"""
        if len(timestamps) != 3:
            return False
        t0, t1, t2 = timestamps
        return (1.5 <= t1 - t0 <= 2.5) and (1.5 <= t2 - t1 <= 2.5)

    def publish_pose_stamped(self, agent_id, coord):
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
