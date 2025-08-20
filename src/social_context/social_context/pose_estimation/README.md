# Pose Estimation Module

This module contains code for estimating human poses from sensor data (camera, lidar, etc.) and will be used in conjunction with the trajectory prediction system to create a comprehensive social context model.

## Overview

Currently, the pose estimation system centers around the **OpenPose Node**, which processes camera images and detects human body position poses using the OpenPose library. 

**Core Component:**
- **OpenPose Node**: Processes live camera data and publishes human pose detections in 2D pixel coordinates
- **Human 3D LiDAR Fusion Node**: Combines 2D pose detections with LiDAR data to produce 3D human positions for navigation
- **Image Publisher**: A testing utility that publishes static test images to simulate camera input during development (lightweight alternative to using HuNav simulator)

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

## Working with HuNav Simulator 

The system has been integrated with the HuNav (Human Navigation) simulator for realistic testing and development of social navigation algorithms.

```
┌─────────────────────────────────────────────────────────────┐
│                    Docker Container                        │
│  ┌─────────────────┐    ┌─────────────────┐               │
│  │  HuNav Simulator│    │  ROS2 Topics    │               │
│  │  - Gazebo 11    │───▶│  - /rgb_camera_ │               │
│  │  - Human agents │    │    frame_sensor │               │
│  │  - PMB2 robot   │    │  - /scan_raw    │               │
│  └─────────────────┘    └─────────────────┘               │
└─────────────────────────────────────────────────────────────┘
                                   │
                          ROS2 Domain Communication
                                   │
┌─────────────────────────────────────────────────────────────┐
│                    Local Development                       │
│  ┌─────────────────┐    ┌─────────────────┐               │
│  │   OpenPose      │    │   LiDAR Fusion  │               │
│  │   Node          │───▶│   Node          │               │
│  │  - 2D poses     │    │  - 3D positions │               │
│  └─────────────────┘    └─────────────────┘               │
└─────────────────────────────────────────────────────────────┘
```

### HuNav Setup

1. **Launch HuNav Simulator** (in Docker container):

```bash
git clone https://github.com/robotics-upo/hunavsim_containers
cd hunavsim_containers
```

Please follow the instructions at https://github.com/robotics-upo/hunavsim_containers. 

We used Option 1 - HuNavSim + Gazebo Classic 11 + ROS 2 Humble + PAL PMB2 robot.

2. **Verify Available Topics**:
```bash
ros2 topic list
# Key topics:
# - /rgb_camera_frame_sensor/image_raw (RGB camera)
# - /rgb_camera_frame_sensor/camera_info (camera calibration)
# - /scan_raw (LiDAR data)
```

3. **Run Complete Pipeline**:
```bash
# Terminal 1: OpenPose node
export OPENPOSE_PATH="/path/to/your/openpose"
ros2 run social_context openpose_node

# Terminal 2: 3D fusion node
ros2 run social_context human_3d_lidar_fusion_node

# Terminal 3: Monitor 3D output
ros2 topic echo /human_poses_3d
```

### Key Topics in HuNav Integration

| Topic | Type | Description |
|-------|------|-------------|
| `/rgb_camera_frame_sensor/image_raw` | sensor_msgs/Image | RGB camera feed from simulator |
| `/rgb_camera_frame_sensor/camera_info` | sensor_msgs/CameraInfo | Camera calibration parameters |
| `/scan_raw` | sensor_msgs/LaserScan | 2D LiDAR data |
| `/human_poses` | geometry_msgs/PoseArray | 2D human detections (pixels) |
| `/human_poses_3d` | geometry_msgs/PoseArray | 3D human positions (meters) |

### Expected Behavior

- **Empty poses when no humans visible**: `poses: []` is normal output if there are no humans in the camera's view
- **2D detections**: Pixel coordinates (e.g., x: 320.5, y: 240.8)  
- **3D positions**: World coordinates in meters (e.g., x: 2.5, y: -1.2, z: 0.0)

## Prerequisites

### System Requirements
- ROS2 Humble (or compatible version)
- Python 3.10
- OpenCV
- NumPy < 2.0 (important for cv_bridge compatibility)
- Test images from MPII Human Pose Dataset

