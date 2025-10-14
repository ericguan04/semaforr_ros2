
# SemaFORR Navigation Package

The core navigation system for Social-SemaFORR, providing cognitively-inspired robot navigation in ROS2.

## Overview

SemaFORR is designed to enable robots to navigate complex environments using cognitive principles. It leverages a set of advisors and configurable parameters for flexible, intelligent path planning.

## Features

- Cognitive navigation algorithms
- Highly configurable via parameters and advisor files
- ROS2 node integration
- Example configuration files for quick setup

## Usage

### Build

Make sure your workspace is built:

```bash
colcon build
source install/setup.bash
```

### Run SemaFORR Node

Launch the main navigation node with required parameters (edit paths as needed):

```bash
ros2 run semaforr semaforr_node --ros-args \
	-p semaforr_path:="/root/semaforr_ros2/src/semaforr" \
	-p target_set:="/root/semaforr_ros2/src/semaforr/config/stage_tutorial/target.conf" \
	-p map_config:="/root/semaforr_ros2/src/semaforr/config/stage_tutorial/stage_tutorialS.xml" \
	-p map_dimensions:="/root/semaforr_ros2/src/semaforr/config/stage_tutorial/dimensions.conf" \
	-p advisors:="/root/semaforr_ros2/src/semaforr/config/advisors.conf" \
	-p params:="/root/semaforr_ros2/src/semaforr/config/params.conf"
```

### Node

- **semaforr_node**: Main entry point for navigation. Requires 6 parameters for configuration.

## Configuration

- `target_set`: List of navigation targets
- `map_config`: XML map configuration
- `map_dimensions`: Map size and boundaries
- `advisors`: Advisor configuration file
- `params`: General parameters for navigation

Example configuration files are provided in the `config/` directory.
