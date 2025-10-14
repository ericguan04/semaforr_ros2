
# Social-SemaFORR ROS2

A cognitively-inspired system for social robot navigation, built on ROS2 Humble. Social-SemaFORR integrates navigation and social context modeling for navigating in dynamic, human-crowded environments.

## Features

- **Cognitive Navigation**: SemaFORR navigation system for intelligent path planning.
- **Social Context Modeling**: Deep learning-based human trajectory prediction.
- **ROS2 Integration**: Bridge nodes for message compatibility.
- **Docker Support**: Easy setup and reproducibility.

## Quick Start

### 1. Clone & Build

```bash
git clone https://github.com/ericguan04/semaforr_ros2.git
cd semaforr_ros2
colcon build
```

### 2. Run with Docker

Build and start the ROS2 Humble container:

```bash
docker-compose up --build
```

This will launch a container with all dependencies pre-installed.

### 3. Source Workspace

After building, source your workspace:

```bash
source install/setup.bash
```

## Packages

- **semaforr**: Core navigation system.  
	_Run:_  
	`ros2 run semaforr semaforr_node --ros-args -p semaforr_path:=... [other params]`

- **social_context**: Social context model and trajectory prediction.  
	_Run:_  
	`ros2 run social_context social_context_hunav`

- **semaforr_bridge**: Bridges Odometry to PoseStamped for SemaFORR.  
	_Run:_  
	`ros2 run semaforr_bridge odom_to_pose_bridge`

See each package’s README in `src/<package>/README.md` for details and example commands.

## Dependencies

- [SemaFORR](https://github.com/ericguan04/semaforr)
- GST Trajectory Prediction (see `src/social_context/README.md`)
- ROS2 Humble
- Python 3.10 (for social context model)

## Social Context Model Setup

1. Create a Python 3.10 virtual environment (recommended: conda).
2. Install dependencies:
	 ```bash
	 pip install -r src/social_context/trajectory_prediction/requirements.txt
	 ```
