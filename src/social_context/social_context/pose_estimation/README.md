# Pose Estimation Module

This module contains code for estimating human poses from sensor data (camera, lidar, etc.) and will be used in conjunction with the trajectory prediction system to create a comprehensive social context model.

## Overview

Currently, the pose estimation system consists of an OpenPose node for 2D pose detection (using OpenPose to detect pose information in the camera's view):

**Alternative Component:**
- **OpenPose Node**: Processes camera images and detects detailed 2D human body keypoints using OpenPose. Useful for downstream tasks.

**Testing Component:**
- **Image Publisher**: A testing utility that publishes static test images to simulate camera input during development

## Prerequisites

### System Requirements
- ROS2 Humble (or compatible version)
- Python 3.10
- OpenCV
- Test images from MPII Human Pose Dataset

### MediaPipe Installation (Recommended)
```bash
pip install mediapipe opencv-python
```

### OpenPose Installation (Optional)
You must have OpenPose installed on your system if you want to use the 2D pose detection features:
```bash
git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git
cd openpose
# Follow OpenPose installation instructions for your system
```

## Testing Setup

The following instructions are for **testing the pose estimation system with static images**. In actual use, the pose nodes will receive live camera data from actual camera nodes.

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

### 3. Configure OpenPose Path (Only if using OpenPose)
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

### Option A: 3D Pose Estimation with MediaPipe (Recommended)

#### Terminal 1: Image Publisher (Testing Only)
```bash
# Navigate to workspace root
cd ~/yourdirectory/semaforr_ros2

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run image publisher with test images
ros2 run social_context image_publisher --ros-args -p image_dir:="$(pwd)/src/social_context/social_context/pose_estimation/test_images/"
```

#### Terminal 2: 3D Pose Localizer (MediaPipe)
```bash
# Navigate to workspace root
cd ~/yourdirectory/semaforr_ros2

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run 3D pose localizer node
ros2 run social_context 3d_pose_localizer

# Optional: Run with custom parameters for better accuracy
ros2 run social_context 3d_pose_localizer --ros-args \
  -p model_complexity:=2 \
  -p min_detection_confidence:=0.7 \
  -p publish_all_landmarks:=true
```

#### Terminal 3: Monitor 3D Output
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Monitor 3D pose detection output
ros2 topic echo /human_poses_3d

# Optional: Monitor all 33 landmarks (if enabled)
ros2 topic echo /human_landmarks_3d

# Check topic publishing rates
ros2 topic hz /camera/image_raw
ros2 topic hz /human_poses_3d

# List all active topics
ros2 topic list
```

### Option B: 2D Pose Estimation with OpenPose

#### Terminal 1: Image Publisher (Testing Only)
```bash
# Navigate to workspace root
cd ~/yourdirectory/semaforr_ros2

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run image publisher with test images
ros2 run social_context image_publisher --ros-args -p image_dir:="$(pwd)/src/social_context/social_context/pose_estimation/test_images/"
```

#### Terminal 2: OpenPose Node
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

#### Terminal 3: Monitor 2D Output
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Monitor 2D pose detection output
ros2 topic echo /human_poses

# Monitor detailed keypoint data
ros2 topic echo /human_keypoints_2d

# Check topic publishing rates
ros2 topic hz /camera/image_raw
ros2 topic hz /human_poses

# List all active topics
ros2 topic list
```

## Expected Output

### MediaPipe 3D Pose Localizer
- **Image Publisher Terminal**: Should display messages like "Published image X/Y: path/to/image.jpg"
- **3D Pose Localizer Terminal**: Should show "3D Pose Localizer Node initialized successfully"
- **Monitor Terminal**: Should display `PoseArray` messages containing 3D coordinates in meters

#### Sample 3D Output
```bash
# 3D Pose Localizer
[INFO] [pose_localizer_3d]: 3D Pose Localizer Node initialized successfully
[INFO] [pose_localizer_3d]: Model complexity: 1 (0=lite, 1=full, 2=heavy)
[INFO] [pose_localizer_3d]: Detection confidence: 0.5

# Topic Monitor (/human_poses_3d)
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: camera_frame
poses:
- position:
    x: 0.12     # meters right of camera center
    y: -0.05    # meters below camera center
    z: 1.8      # meters in front of camera
  orientation:
    x: 0.0
    y: 0.0
    z: 0.342    # facing direction (yaw rotation)
    w: 0.940
```

### OpenPose 2D Detection
- **Image Publisher Terminal**: Should display messages like "Published image X/Y: path/to/image.jpg"
- **OpenPose Terminal**: Should show "OpenPose initialized successfully" 
- **Monitor Terminal**: Should display `PoseArray` messages containing 2D pixel coordinates

#### Sample 2D Output
```bash
# OpenPose Node
[INFO] [openpose_node]: Initializing OpenPose...
[INFO] [openpose_node]: OpenPose initialized successfully

# Topic Monitor (/human_poses)
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: camera
poses:
- position:
    x: 320.5    # pixel x coordinate
    y: 240.8    # pixel y coordinate
    z: 0.0      # (not used in 2D)
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

## Understanding the Output

### MediaPipe 3D Coordinates
The 3D pose localizer outputs **real-world coordinates in meters**:
- **Position x**: Left/right from camera center (+ = right, - = left)
- **Position y**: Up/down from camera center (+ = up, - = down)  
- **Position z**: Distance from camera (+ = in front of camera)
- **Orientation**: Which direction the person is facing (quaternion)

### OpenPose 2D Coordinates
The OpenPose node outputs **pixel coordinates**:
- **Position x**: Horizontal pixel position in image
- **Position y**: Vertical pixel position in image
- **Position z**: Always 0.0 (not used for 2D poses)

## Troubleshooting

### MediaPipe Issues
```bash
# Test MediaPipe installation
python3 -c "import mediapipe as mp; print('MediaPipe version:', mp.__version__)"

# If import fails
pip install --upgrade mediapipe opencv-python
```

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

### Performance Tuning

**For faster processing (MediaPipe):**
```bash
ros2 run social_context 3d_pose_localizer --ros-args -p model_complexity:=0
```

**For higher accuracy (MediaPipe):**
```bash
ros2 run social_context 3d_pose_localizer --ros-args \
  -p model_complexity:=2 \
  -p min_detection_confidence:=0.8
```
