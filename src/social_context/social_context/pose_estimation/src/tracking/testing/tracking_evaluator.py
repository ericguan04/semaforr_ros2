# FOR TESTING AND EVALUATION PURPOSES ONLY
#!/usr/bin/env python3
"""
Tracking Evaluator Node

Subscribes to HuNav's ground-truth agent positions (/human_states) and
your tracker's output (/human_poses_3d_tracked_global), matches them to
measure:
  - ID switch count (how often a ground-truth person's tracker ID
    actually changes, i.e. their previous tracker either disappeared or
    moved out of range and a different existing tracker had to be
    picked up instead)
  - Position error (distance between ground-truth and matched tracker
    position)

Matching is identity-preserving, not a fresh nearest-neighbor solve every
frame: a gt person keeps their current tracker as long as it's still
present and within max_match_distance, even if a different tracker is
briefly closer. A pure per-frame nearest-neighbor re-match (the previous
approach) falsely reports an "ID switch" whenever two real people simply
cross paths -- each tracker keeps following the same person the whole
time, but naive re-matching flips which tracker is "closest" to which gt
person mid-crossing and misreports it as a switch. Only gt people whose
current tracker is no longer valid get re-matched via Hungarian
assignment against the remaining unclaimed trackers, and only that
re-match counts as a genuine switch.
"""

import rclpy
from rclpy.node import Node
from social_context_msgs.msg import TrackedPersonArray
from hunav_msgs.msg import Agents
import csv
import numpy as np
from scipy.optimize import linear_sum_assignment

class TrackingEvaluator(Node):
    def __init__(self):
        super().__init__('tracking_evaluator')

        self.latest_ground_truth = {}    # {gt_id: (x, y)}
        self.gt_to_tracker_id = {}       # {gt_id : tracker_id}
        self.id_switch_count = 0
        self.position_errors = []
        self.records = []

        self.max_match_distance = 3.0 

        self.gt_sub = self.create_subscription(
            Agents, '/human_states', self.ground_truth_callback, 10
        )

        self.tracked_sub = self.create_subscription(
            TrackedPersonArray, '/human_poses_3d_tracked_global', self.tracked_callback, 10
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('Tracking Evaluator Ready')
        self.get_logger().info('  Ground truth: /human_states')
        self.get_logger().info('  Tracker output: /human_poses_3d_tracked_global')
        self.get_logger().info('=' * 60)

    def ground_truth_callback(self, msg: Agents):
        for agent in msg.agents:
            self.latest_ground_truth[agent.id] = (agent.position.position.x, agent.position.position.y)

    def tracked_callback(self, msg: TrackedPersonArray):
        if not self.latest_ground_truth:
            return

        if len(msg.people) == 0:
            return

        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        gt_ids = list(self.latest_ground_truth.keys())
        tracker_positions = {person.id: (person.x, person.y) for person in msg.people}

        resolved = {}  # gt_id -> (tracker_id, dist)
        claimed_trackers = set()
        unresolved_gt_ids = []

        for gt_id in gt_ids:
            prev_tracker_id = self.gt_to_tracker_id.get(gt_id)
            if prev_tracker_id is not None and prev_tracker_id in tracker_positions and prev_tracker_id not in claimed_trackers:
                dist = np.linalg.norm(
                    np.array(self.latest_ground_truth[gt_id]) - np.array(tracker_positions[prev_tracker_id])
                )
                if dist <= self.max_match_distance:
                    resolved[gt_id] = (prev_tracker_id, dist)
                    claimed_trackers.add(prev_tracker_id)
                    continue
            unresolved_gt_ids.append(gt_id)

        remaining_tracker_ids = [tid for tid in tracker_positions if tid not in claimed_trackers]
        if unresolved_gt_ids and remaining_tracker_ids:
            cost_matrix = np.zeros((len(unresolved_gt_ids), len(remaining_tracker_ids)))
            for i, gt_id in enumerate(unresolved_gt_ids):
                for j, tracker_id in enumerate(remaining_tracker_ids):
                    cost_matrix[i, j] = np.linalg.norm(
                        np.array(self.latest_ground_truth[gt_id]) - np.array(tracker_positions[tracker_id])
                    )

            row_indices, col_indices = linear_sum_assignment(cost_matrix)
            for i, j in zip(row_indices, col_indices):
                dist = cost_matrix[i, j]
                if dist > self.max_match_distance:
                    continue  # no reasonable match for this gt person this frame

                gt_id = unresolved_gt_ids[i]
                tracker_id = remaining_tracker_ids[j]
                resolved[gt_id] = (tracker_id, dist)

        for gt_id, (tracker_id, dist) in resolved.items():
            self.position_errors.append(dist)

            previous_tracker_id = self.gt_to_tracker_id.get(gt_id)
            if previous_tracker_id is not None and previous_tracker_id != tracker_id:
                self.id_switch_count += 1
                self.get_logger().warn(
                    f'ID SWITCH: ground-truth person {gt_id} was tracker '
                    f'{previous_tracker_id}, now tracker {tracker_id} '
                    f'(dist={dist:.3f}m)'
                )

            self.gt_to_tracker_id[gt_id] = tracker_id

            self.records.append({
                'time': msg_time,
                'gt_id': gt_id,
                'tracker_id': tracker_id,
                'position_error': dist
            })


    def save_results(self):
        output_path = '/tmp/tracking_eval_results.csv'
        try:
            with open(output_path, 'w', newline='') as f:
                writer = csv.DictWriter(
                    f, fieldnames=['time', 'gt_id', 'tracker_id', 'position_error']
                )
                writer.writeheader()
                writer.writerows(self.records)
        except Exception as e:
            self.get_logger().error(f'Failed to save CSV: {e}')

        avg_error = (
            sum(self.position_errors) / len(self.position_errors)
            if self.position_errors else 0.0
        )
        max_error = max(self.position_errors) if self.position_errors else 0.0

        self.get_logger().info('=' * 60)
        self.get_logger().info('EVALUATION SUMMARY')
        self.get_logger().info(f'  Total frames logged: {len(self.records)}')
        self.get_logger().info(f'  ID switches: {self.id_switch_count}')
        self.get_logger().info(f'  Average position error: {avg_error:.3f}m')
        self.get_logger().info(f'  Max position error: {max_error:.3f}m')
        self.get_logger().info(f'  Results saved to: {output_path}')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()