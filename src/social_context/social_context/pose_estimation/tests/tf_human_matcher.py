#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import tf2_ros
from tf2_ros import TransformException

AGENT_NAMES = [f"agent{i}" for i in range(1, 16)]  # adjust max agents if needed


def dist2d(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.hypot(dx, dy)


class TFHumanMatcher(Node):
    def __init__(self):
        super().__init__('tf_human_matcher')

        self.declare_parameter('det_topic', '/human_poses_3d_global')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('match_threshold', 0.8)

        self.det_topic = self.get_parameter('det_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.match_threshold = self.get_parameter('match_threshold').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseArray, self.det_topic, self.det_cb, 10)

        self.get_logger().info(
            f"Matching detections on {self.det_topic} against TF agents in {self.map_frame}"
        )

    def det_cb(self, msg: PoseArray):
        detections = [(p.position.x, p.position.y) for p in msg.poses]
        if not detections:
            return

        # collect all available agents from TF
        agents = []
        for name in AGENT_NAMES:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, name, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.05)
                )
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                agents.append((name, (x, y)))
            except TransformException:
                continue

        if not agents:
            self.get_logger().warn("No agent TFs found")
            return

        matched = 0
        errors = []

        for i, det in enumerate(detections):
            best_name = None
            best_dist = float('inf')
            for name, pos in agents:
                d = dist2d(det, pos)
                if d < best_dist:
                    best_dist = d
                    best_name = name

            if best_dist < self.match_threshold:
                matched += 1
                errors.append(best_dist)
            else:
                self.get_logger().warn(
                    f"Detection #{i} at ({det[0]:.2f}, {det[1]:.2f}) has no agent within "
                    f"{self.match_threshold} m (closest was {best_dist:.2f} m: {best_name})"
                )

        if errors:
            mean_err = sum(errors) / len(errors)
            max_err = max(errors)
            self.get_logger().info(
                f"Detections: {len(detections)}, matched: {matched}, "
                f"mean err: {mean_err:.3f} m, max err: {max_err:.3f} m"
            )
        else:
            self.get_logger().info(f"Detections: {len(detections)}, matched: {matched}")

def main(args=None):
    rclpy.init(args=args)
    node = TFHumanMatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
