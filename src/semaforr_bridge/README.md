# SemaFORR_Bridge Package
This package is meant to bridge incompatible topics and message between ROS systems

How to use:
* `ros2 run semaforr_bridge (node)`

Available Nodes:
* `odom_to_pose_bridge`: This node bridges the Odometry message from the robot controller (PMB2) to a PoseStamped message for SemaFORR. It subscribes to the Odometry topic and publishes the PoseStamped message