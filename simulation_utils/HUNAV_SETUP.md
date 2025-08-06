# HuNav Human Navigation Simulator Installation Guide

This guide walks you through setting up HuNav (Human Navigation simulator) with Gazebo Fortress and ROS2. 

## Step 1: Install Gazebo Fortress
(This is the core 3D robotics simulator itself that will render our virtual world, handle physics calculations, and simulate sensors like cameras and LiDAR.)

```bash
# Add Gazebo repositories to get the latest stable version
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update package lists and install Gazebo Fortress
sudo apt update
sudo apt install ignition-fortress -y

# Install sensor and development libraries for custom plugins
sudo apt install libignition-gazebo6-dev libignition-sensors6-dev -y

# Verify installation
ign gazebo --version
```

## Step 2: Install ROS2-Gazebo Bridge
(The bridge acts as a translator between ROS2 and Gazebo. Without it, the ROS2 nodes can't receive sensor data from Gazebo or send commands to simulated robots.)

```bash
# Install core bridge packages for ROS2-Gazebo communication
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim ros-humble-ros-gz-image -y

# Install message types for sensors and computer vision
sudo apt install ros-humble-sensor-msgs ros-humble-vision-msgs ros-humble-cv-bridge ros-humble-image-transport -y
```

## Step 3: Install HuNav Simulation Packages
(HuNav provides realistic human pedestrian behavior in the simulations. The wrapper package integrates it specifically with Gazebo Fortress, while the image pipeline helps process camera data.)

```bash
cd ~/semaforr_ros2/src

# Clone HuNav core simulation and Gazebo integration
git clone https://github.com/robotics-upo/hunav_sim.git
git clone https://github.com/robotics-upo/hunav_gazebo_fortress_wrapper.git

# Clone image processing tools for camera sensor data
git clone https://github.com/ros-perception/image_pipeline.git -b humble
```

## Step 4: Install Dependencies and Build
(This step ensures all Python and ROS dependencies are met, then compiles everything into executable packages that ROS2 can run.)

```bash
cd ~/semaforr_ros2  

# Install Python libraries for image processing and numerical operations
pip3 install opencv-python numpy

# Install all ROS package dependencies automatically
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace (compile all packages)
colcon build --packages-select social_context --symlink-install
```

## Step 5: Source and Test
*Purpose: Make the built packages available to your shell*

```bash
# Source the workspace to add it to your ROS2 environment
source ~/semaforr_ros2/install/setup.bash

# Test that everything works
ros2 launch hunav_gazebo_fortress_wrapper hunav_demo.launch.py
```

---

**Quick Summary:** 
1. **Gazebo** = The 3D world and physics engine
2. **Bridge** = Communication layer between ROS2 and Gazebo  
3. **HuNav** = Human crowd simulation add-on
4. **Dependencies** = Required libraries and compilation
5. **Source** = Make everything available to use

Each component is essential - without Gazebo you have no simulation, without the bridge ROS2 can't control anything, and without HuNav you can't simulate realistic human behavior for navigation research.
