# Pose Estimation Module

This module contains code for estimating human poses from sensor data (camera, lidar, etc.) and will be used in conjunction with the trajectory prediction system to create a comprehensive social context model.

## Overview

Currently, the pose estimation system centers around the **OpenPose Node**, which processes camera images and detects human body position poses using the OpenPose library. 

**Core Component:**
- **OpenPose Node**: Processes live camera data and publishes human pose detections

**Testing Component:**
- **Image Publisher**: A testing utility that publishes static test images to simulate camera input during development

## Prerequisites

### System Requirements
- ROS2 Humble (or compatible version)
- Python 3.10
- OpenCV
- Test images from MPII Human Pose Dataset

### OpenPose Installation
You must have OpenPose installed on your system. Clone it from the official repository:
```bash
git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git
cd openpose
# Follow OpenPose installation instructions for your system
```

## Testing Setup

The following instructions are for **testing the pose estimation system with static images**. In production, the OpenPose node will receive live camera data from actual camera nodes.

### 1. Prepare Test Data (Testing Only)
Download images from the [MPII Human Pose Dataset](http://human-pose.mpi-inf.mpg.de/) and place them in the test images directory:
```bash
# Create test_images directory if it doesn't exist
mkdir -p src/social_context/social_context/pose_estimation/test_images/

# Place your downloaded .jpg and .png files in this directory
```

### 2. Build the Package
Navigate to your workspace root and build the social context package:
```bash
# Navigate to workspace root
cd ~/yourdirectory/semaforr_ros2

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Build the social context package
colcon build --packages-select social_context
```

### 3. Configure OpenPose Path
Locate your OpenPose installation directory:
```bash
# Check common installation paths
ls /usr/local/openpose/build/python/ 2>/dev/null || echo "Not found in /usr/local/openpose"
ls /opt/openpose/build/python/ 2>/dev/null || echo "Not found in /opt/openpose"

# Or search for pyopenpose files
find /home -name "pyopenpose*" 2>/dev/null
```

**Example output interpretation:**
```bash
# If you see output like:
/home/developer2/openpose/build/python/openpose/pyopenpose.cpython-310-x86_64-linux-gnu.so

# Then your OPENPOSE_PATH should be:
export OPENPOSE_PATH="/home/developer2/openpose"
```

## Testing the Pose Estimation System

### Terminal 1: Image Publisher (Testing Only)
```bash
# Navigate to workspace root
cd ~/yourdirectory/semaforr_ros2

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run image publisher with test images
ros2 run social_context image_publisher --ros-args -p image_dir:="$(pwd)/src/social_context/social_context/pose_estimation/test_images/"
```

### Terminal 2: OpenPose Node
```bash
# Navigate to workspace root
cd ~/yourdirectory/semaforr_ros2

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Set OpenPose path (adjust to your actual installation)
export OPENPOSE_PATH="/home/yourusername/openpose"

# Run OpenPose node
ros2 run social_context openpose_node
```

### Terminal 3: Monitor Output
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Monitor pose detection output
ros2 topic echo /human_poses

# Check topic publishing rates
ros2 topic hz /camera/image_raw
ros2 topic hz /human_poses

# List all active topics
ros2 topic list
```

## Expected Output

### Successful Operation
- **Image Publisher Terminal**: Should display messages like "Published image X/Y: path/to/image.jpg"
- **OpenPose Terminal**: Should show "OpenPose initialized successfully" 
- **Monitor Terminal**: Should display `PoseArray` messages containing detected human pose coordinates

### Sample Output
```bash
# Image Publisher
[INFO] [image_publisher]: Found 10 images to publish
[INFO] [image_publisher]: Published image 1/10: test_images/person1.jpg

# OpenPose Node
[INFO] [openpose_node]: Initializing OpenPose...
[INFO] [openpose_node]: OpenPose initialized successfully

# Topic Monitor
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: camera
poses:
- position:
    x: 320.5
    y: 240.8
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

## Troubleshooting

### OpenPose Import Errors
```bash
# Verify OpenPose path
echo $OPENPOSE_PATH
ls $OPENPOSE_PATH/build/python/

# Test manual import
python3 -c "import sys; sys.path.append('/path/to/openpose/build/python'); import pyopenpose"
```

### No Images Found
```bash
# Check test images directory
ls src/social_context/social_context/pose_estimation/test_images/

# Verify image formats (.jpg, .png)
file src/social_context/social_context/pose_estimation/test_images/*
```

### Build Issues
```bash
# Clean rebuild
colcon build --packages-select social_context --cmake-clean-cache
source install/setup.bash
```

