
# SemaFORR Bridge Package

A ROS2 package for bridging incompatible topics and messages between ROS systems, enabling seamless integration for Social-SemaFORR navigation.

## Overview

SemaFORR Bridge provides nodes that convert and relay messages between different ROS topics and message types, ensuring compatibility between robot controllers and the SemaFORR navigation system.

## Features

- Bridges Odometry messages to PoseStamped for SemaFORR
- Facilitates communication between robot controllers (e.g., PMB2) and navigation nodes
- Simple ROS2 node interface

## Usage

### Build

Make sure your workspace is built and sourced:

```bash
colcon build
source install/setup.bash
```

### Run Bridge Node

Launch the bridge node to convert Odometry to PoseStamped:

```bash
ros2 run semaforr_bridge odom_to_pose_bridge
```

## Node

- **odom_to_pose_bridge**:  
	- Subscribes to: Odometry topic from robot controller  
	- Publishes: PoseStamped message for SemaFORR
