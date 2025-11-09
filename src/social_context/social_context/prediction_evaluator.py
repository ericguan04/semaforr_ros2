import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from hunav_msgs.msg import Agents
import math
from collections import deque

# --- ASSUMPTIONS TO BE FILLED IN ---
# !!! This is the critical value from Question 1 !!!
# (e.g., if your model predicts 5.0s, 10.0s, 15.0s, this is 5.0)
MODEL_PREDICTION_TIMESTEP_SEC = 5.0 # <--- REPLACE THIS

# This is from Question 2. How close in time do the ground truth
# and prediction need to be to be considered a "match"?
# (e.g., if /human_states is 10Hz, a 0.1s tolerance is good)
TIMESTAMP_MATCH_TOLERANCE_SEC = 0.2 # <--- REPLACE THIS
# ---

def l2_distance(pos1, pos2):
    """Calculates the Euclidean (L2) distance between two (x,y) tuples."""
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

class PredictionEvaluator(Node):
    def __init__(self):
        super().__init__('prediction_evaluator')

        # This stores predictions we've received but haven't found a matching ground truth for yet
        # Format: [{'agent_id': str, 'target_time': rclpy.time.Time, 'pred_pos': (x,y)}]
        self.pending_evaluations = deque(maxlen=200) # Store last 200 pending

        # This stores all the calculated errors
        # Format: [error_dist_1, error_dist_2, ...]
        self.completed_errors = []

        # 1. Subscriber for the predictions from your CoordinateListener
        self.prediction_subscriber = self.create_subscription(
            PoseStamped,
            '/pedestrian_predictions',
            self.prediction_callback,
            10
        )

        # 2. Subscriber for the ground truth data
        self.human_states_subscriber = self.create_subscription(
            Agents,
            '/human_states',
            self.ground_truth_callback,
            10
        )
        
        self.get_logger().info("PredictionEvaluator node started. Waiting for data...")

    def prediction_callback(self, msg):
        # This callback receives a prediction
        try:
            # Parse the frame_id (e.g., "1_pred_0")
            agent_id, _, step_index_str = msg.header.frame_id.rpartition('_')
            step_index = int(step_index_str) # This will be 0, 1, 2
        except ValueError:
            self.get_logger().warn(f"Could not parse frame_id: {msg.header.frame_id}")
            return
            
        prediction_made_at = rclpy.time.Time.from_msg(msg.header.stamp)
        
        # Calculate the *target time* this prediction is FOR
        # (e.g., if step_index is 2, it's for 3 * 5.0s = 15s in the future)
        time_offset_sec = (step_index + 1) * MODEL_PREDICTION_TIMESTEP_SEC
        target_time = prediction_made_at + Duration(seconds=time_offset_sec)

        # Store this pending evaluation
        self.pending_evaluations.append({
            'agent_id': agent_id,
            'target_time': target_time,
            'pred_pos': (msg.pose.position.x, msg.pose.position.y),
            'step_index': step_index
        })

    def ground_truth_callback(self, msg):
        # This callback receives the *actual* human states
        # We check if this ground truth message matches any of our pending predictions
        
        # Use the timestamp from the message header
        current_time = rclpy.time.Time.from_msg(msg.header.stamp) 
        
        matched_indices = []

        # Iterate through all agents in the current ground truth message
        for agent in msg.agents:
            gt_agent_id = str(agent.id)
            gt_pos = (agent.position.position.x, agent.position.position.y)

            # Check all pending evaluations to see if this agent matches
            # We iterate backwards so we can safely remove items
            for i in range(len(self.pending_evaluations) - 1, -1, -1):
                eval_task = self.pending_evaluations[i]

                if eval_task['agent_id'] != gt_agent_id:
                    continue # Not the right agent

                # Check if the timestamps match within our tolerance
                time_diff_sec = abs((eval_task['target_time'] - current_time).nanoseconds / 1e9)

                if time_diff_sec < TIMESTAMP_MATCH_TOLERANCE_SEC:
                    # --- WE HAVE A MATCH! ---
                    
                    # 1. Calculate the error
                    error = l2_distance(eval_task['pred_pos'], gt_pos)
                    
                    # 2. Store the error
                    self.completed_errors.append(error)
                    
                    # 3. Log it
                    self.get_logger().info(
                        f"MATCH: Agent {gt_agent_id} [Step {eval_task['step_index']}] "
                        f"Error: {error:.3f}m"
                    )
                    
                    # 4. Mark this task for removal
                    matched_indices.append(i)

        # Remove all matched evaluations from the pending deque
        # (Use a set for efficiency and iterate from high to low to avoid index shifting)
        for i in sorted(list(set(matched_indices)), reverse=True):
            del self.pending_evaluations[i]


    def print_statistics(self):
        # This function will be called on shutdown
        if not self.completed_errors:
            self.get_logger().info("No prediction errors were calculated.")
            return

        mean_error = sum(self.completed_errors) / len(self.completed_errors)
        max_error = max(self.completed_errors)
        
        self.get_logger().info("--- PREDICTION PERFORMANCE ---")
        self.get_logger().info(f"Total Predictions Evaluated: {len(self.completed_errors)}")
        self.get_logger().info(f"Mean Displacement Error (ADE): {mean_error:.3f}m")
        self.get_logger().info(f"Max Error: {max_error:.3f}m")


def main(args=None):
    rclpy.init(args=args)
    evaluator_node = PredictionEvaluator()

    try:
        rclpy.spin(evaluator_node)
    except KeyboardInterrupt:
        evaluator_node.get_logger().info("Shutting down...")
    finally:
        # On shutdown, print the final stats
        evaluator_node.print_statistics()
        evaluator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()