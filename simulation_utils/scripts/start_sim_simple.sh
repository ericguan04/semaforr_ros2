#!/bin/bash

WORKSPACE_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

echo "Starting Simulation in WSL2 (Simplified)"
echo "========================================="

# Source ROS2
source /opt/ros/humble/setup.bash
source $WORKSPACE_ROOT/install/setup.bash 2>/dev/null

# Clean up old processes
echo "Cleaning up old processes..."
pkill -f gazebo
pkill -f ign
pkill -f ros_gz_bridge
sleep 2

# IMPORTANT: Don't set conflicting environment variables for WSL2
# Just use the simplest command possible

echo "Starting Gazebo in headless mode (simple)..."

# Use the absolute simplest command - just server mode, no render engine specified
ign gazebo -s $WORKSPACE_ROOT/simulation_utils/worlds/sensor_world_headless.sdf > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!

echo "Waiting for Gazebo to start..."
sleep 5

# Check if Gazebo started successfully
if ps -p $GAZEBO_PID > /dev/null; then
    echo "✓ Gazebo started successfully (PID: $GAZEBO_PID)"
else
    echo "ERROR: Gazebo failed to start"
    echo "Gazebo log output:"
    cat /tmp/gazebo.log
    echo ""
    echo "Trying alternative: empty world test..."

    # Try with empty world to see if Gazebo works at all
    ign gazebo -s empty.sdf > /tmp/gazebo_empty.log 2>&1 &
    GAZEBO_PID=$!
    sleep 3

    if ps -p $GAZEBO_PID > /dev/null; then
        echo "Empty world works! Issue is with the world file."
        kill $GAZEBO_PID
    else
        echo "Gazebo won't start at all. Check your installation."
        cat /tmp/gazebo_empty.log
    fi
    exit 1
fi

# Try to list topics (might fail in headless but worth trying)
echo "Attempting to list Gazebo topics..."
timeout 3 ign topic -l 2>/dev/null | grep -E "(camera|lidar|clock)" || echo "Could not list topics (normal in some headless setups)"

# Start the ROS2 bridge
echo "Starting camera bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    /camera@sensor_msgs/msg/Image@ignition.msgs.Image \
    --ros-args -r /camera:=/camera/image_raw > /tmp/bridge.log 2>&1 &
BRIDGE_PID=$!

sleep 2

# Verify bridge is running
if ps -p $BRIDGE_PID > /dev/null; then
    echo "✓ Bridge started successfully (PID: $BRIDGE_PID)"
else
    echo "ERROR: Bridge failed to start"
    echo "Bridge log:"
    cat /tmp/bridge.log
    kill $GAZEBO_PID 2>/dev/null
    exit 1
fi

# Final status
echo ""
echo "========================================="
echo "System Status:"
echo "  Gazebo PID: $GAZEBO_PID"
echo "  Bridge PID: $BRIDGE_PID"
echo ""
echo "Quick Tests:"
echo "  ros2 topic list | grep camera"
echo "  ros2 topic hz /camera/image_raw"
echo ""
echo "To test camera data:"
echo "  python3 $WORKSPACE_ROOT/simulation_utils/scripts/test_camera_wsl2.py"
echo ""
echo "Press Ctrl+C to stop"

# Cleanup function
cleanup() {
    echo -e "\nShutting down..."
    kill $GAZEBO_PID 2>/dev/null
    kill $BRIDGE_PID 2>/dev/null
    pkill -f gazebo 2>/dev/null
    pkill -f ign 2>/dev/null
    pkill -f ros_gz_bridge 2>/dev/null
    echo "Cleanup complete"
    exit 0
}

trap cleanup INT TERM

# Keep running
wait