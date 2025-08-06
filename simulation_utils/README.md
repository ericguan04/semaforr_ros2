# Simulation Utilities for SEMAFORR_ROS2

This folder contains utilities for running Gazebo Fortress simulations to generate sensor data for testing computer vision nodes (like OpenPose) without requiring physical hardware.

## Directory Structure

```
simulation_utils/
├── worlds/              # Gazebo world definitions
│   └── sensor_world.sdf # Simple world with camera and walking humans
├── launch/              # ROS2 launch files
│   └── sensor_bridge.launch.py # Launches Gazebo + ROS2 bridge
├── scripts/             # Helper scripts
│   ├── start_sim.sh     # Start simulation (headless or GUI)
│   └── test_camera.sh   # Test camera connectivity
└── README.md           # This file
```

## Files Explained

### worlds/sensor_world.sdf
- **Purpose**: Defines the simulated environment
- **Contains**:
  - Ground plane and basic lighting
  - Camera sensor positioned to view the scene
  - Two animated human actors walking in loops
- **Why**: Provides visual data of moving humans for pose detection testing

### launch/sensor_bridge.launch.py
- **Purpose**: Launch file to start all components together
- **Starts**:
  - Gazebo with the world file
  - ROS2-Gazebo bridge for camera data
  - Remaps camera topic to `/camera/image_raw` (OpenPose standard)
- **Why**: Simplifies startup and ensures consistent configuration

### scripts/start_sim.sh
- **Purpose**: Bash script for easy simulation startup
- **Modes**:
  - `headless`: Run without GUI (for WSL2/servers)
  - `gui`: Run with GUI (if display available)
  - `launch`: Use ROS2 launch file
- **Why**: Provides flexibility for different environments

### scripts/test_camera.sh
- **Purpose**: Verify camera data is flowing
- **Checks**:
  - Gazebo topics
  - ROS2 topics
  - Data publishing rate
- **Why**: Quick diagnostics without running full pipeline

## Usage

### Quick Start (Headless Mode - Recommended for WSL2)

```bash
cd ~/Git/semaforr_ros2
# Don't use sudo!
bash simulation_utils/scripts/start_sim.sh headless
```

### With GUI (if you have X11 forwarding)

```bash
cd ~/Git/semaforr_ros2
bash simulation_utils/scripts/start_sim.sh gui
```

### Using Launch File

```bash
cd ~/Git/semaforr_ros2
source install/setup.bash
ros2 launch simulation_utils/launch/sensor_bridge.launch.py
```

### Manual Start (for debugging)

Terminal 1 - Start Gazebo (headless):
```bash
cd ~/Git/semaforr_ros2
ign gazebo -s simulation_utils/worlds/sensor_world_headless.sdf  # -s for server/headless
```

Terminal 2 - Bridge camera to ROS2:
```bash
cd ~/Git/semaforr_ros2
source install/setup.bash
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@ignition.msgs.Image --ros-args -r /camera:=/camera/image_raw
```

Terminal 3 - Run OpenPose (or any image processing node):
```bash
cd ~/Git/semaforr_ros2
source install/setup.bash
export OPENPOSE_PATH=/path/to/openpose
ros2 run [your_package] openpose_node
```

## Testing

Verify camera is publishing:
```bash
ros2 topic hz /camera/image_raw
```

Check if humans are detected:
```bash
ros2 topic echo /human_poses
```

Visualize camera feed (if you have X11 forwarding):
```bash
ros2 run rqt_image_view rqt_image_view
# Select /camera/image_raw from dropdown
```

## Troubleshooting

### OpenGL/Ogre Errors (Common in WSL2)
- **Error**: `Ogre::UnimplementedException` or `libEGL warning`
- **Solution**: Use headless mode with `-s` flag
- This is normal in WSL2 - the simulation works fine without GUI

### No camera data
1. Check Gazebo is running: `ps aux | grep ign`
2. Check Gazebo topics: `ign topic -l | grep camera`
3. Check bridge is running: `ps aux | grep ros_gz_bridge`
4. Check ROS2 topics: `ros2 topic list | grep camera`

### Gazebo won't start
- Kill old processes: `pkill -f gazebo && pkill -f ign`
- Check for port conflicts: `netstat -tulpn | grep 11345`
- Try verbose mode: `ign gazebo -s -v 4 [world_file]`

### Bridge errors
- Ensure ros_gz_bridge is installed: `sudo apt install ros-humble-ros-gz-bridge`
- Check message types match between Gazebo and ROS2
- Try without remapping first to isolate issues

### Simulation runs slowly
- Reduce camera resolution in world file (640x480 → 320x240)
- Lower camera update rate (30Hz → 10Hz)
- Build in Release mode: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`
- Close other applications to free RAM

## Integration with OpenPose

The simulation publishes camera images to `/camera/image_raw` which matches the OpenPose node's expected input topic. The walking humans in the simulation should be detected and published as poses on `/human_poses`.

### Expected Data Flow
1. Gazebo simulates camera sensor → publishes to `/camera` (Ignition topic)
2. Bridge converts and remaps → publishes to `/camera/image_raw` (ROS2 topic)
3. OpenPose node subscribes to `/camera/image_raw`
4. OpenPose detects humans → publishes to `/human_poses`

## WSL2 Specific Notes

### Required Setup
- WSL2 with Ubuntu 20.04 or 22.04