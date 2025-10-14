
# Social Context Package

Provides all nodes, functions, and models for the social context module in the Social-SemaFORR navigation system.

## Overview

The Social Context package enables robots to understand and predict human movement in dynamic environments using deep learning. It integrates with HuNavSim and SemaFORR to enhance socially-aware navigation.

## Features

- Deep learning-based human trajectory prediction
- ROS2 node for real-time social context updates
- Integration with HuNavSim and SemaFORR
- Configurable and extensible model support

## Usage

### Build

Make sure your workspace is built and sourced:

```bash
colcon build
source install/setup.bash
```

### Run Social Context Node

Launch the main node for social context prediction:

```bash
ros2 run social_context social_context_hunav
```

## Node

- **social_context_hunav**:  WIP - WILL BE CHANGED
	- Subscribes to: `/human_states` topic from HuNavSim (Agents message)  
	- Runs: Deep learning prediction function  
	- Publishes: Predicted pose (PoseStamped message)

## Virtual Environment & Dependencies

Running the social context model requires specific Python dependencies.

1. Create a Python 3.10 virtual environment (recommended: conda).
2. Install dependencies:

	 ```bash
	 pip install -r src/social_context/trajectory_prediction/requirements.txt
	 ```
