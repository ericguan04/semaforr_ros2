#!/bin/bash

WORKSPACE_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

echo "Starting Simulation in WSL2 Headless Mode"
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

# Set environment variables for headless operation
export QT_QPA_PLATFORM=offscreen
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

echo "Starting Gazebo in pure headless mode..."

# Option 1: Try with null rendering first (fastest, no rendering at all)
echo "Attempting to start with null rendering..."
ign gazebo -s --render-engine ogre2 --headless-rendering \
    $WORKSPACE_ROOT/simulation_utils/worlds/sensor_world_headless.sdf \
    > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!

sleep 3

# Check if Gazebo started successfully
if ! ps -p $GAZEBO_PID > /dev/null; then
    echo "Null rendering failed, trying software rendering..."

    # Option 2: Try with software rendering
    export MESA_LOADER_DRIVER_OVERRIDE=swrast
    ign gazebo -s \
        $WORKSPACE_ROOT/simulation_utils/worlds/sensor_world_headless.sdf \
        > /tmp/gazebo.log 2>&1 &
    GAZEBO_PID=$!

    sleep 3

    if ! ps -p $GAZEBO_PID > /dev/null; then
        echo "ERROR: Gazebo failed to start even with software rendering"
        echo "Check /tmp/gazebo.log for details:"
        tail -20 /tmp/gazebo.log
        exit 1
    fi
fi

echo "✓ Gazebo started successfully (PID: $GAZEBO_PID)"

# Check if Gazebo topics are available
echo "Checking Gazebo topics..."
timeout 5 ign topic -l > /tmp/gz_topics.txt 2>&1
if [ $? -eq 0 ]; then
    echo "✓ Gazebo topics available:"
    grep camera /tmp/gz_topics.txt || echo "  Warning: No camera topic found"
else
    echo "Warning: Could not list Gazebo topics (this might be normal in headless mode)"
fi

# Start the ROS2 bridge
echo "Starting camera bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    /camera@sensor_msgs/msg/Image@ignition.msgs.Image \
    --ros-args -r /camera:=/camera/image_raw &
BRIDGE_PID=$!

sleep 2

# Verify bridge is running
if ! ps -p $BRIDGE_PID > /dev/null; then
    echo "ERROR: Bridge failed to start"
    kill $GAZEBO_PID 2>/dev/null
    exit 1
fi

echo "✓ Bridge started successfully (PID: $BRIDGE_PID)"

# Final status
echo ""
echo "========================================="
echo "System Status:"
echo "  Gazebo PID: $GAZEBO_PID"
echo "  Bridge PID: $BRIDGE_PID"
echo ""
echo "Test Commands:"
echo "  List ROS topics:     ros2 topic list"
echo "  Check camera rate:   ros2 topic hz /camera/image_raw"
echo "  Echo camera info:    ros2 topic echo /camera/image_raw --once"
echo ""
echo "To run OpenPose:"
echo "  cd $WORKSPACE_ROOT && source install/setup.bash"
echo "  ros2 run [package] openpose_node"
echo ""
echo "Logs:"
echo "  Gazebo log: /tmp/gazebo.log"
echo ""
echo "Press Ctrl+C to stop all processes"

# Function to cleanup
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $GAZEBO_PID 2>/dev/null
    kill $BRIDGE_PID 2>/dev/null
    sleep 1
    pkill -f gazebo 2>/dev/null
    pkill -f ign 2>/dev/null
    pkill -f ros_gz_bridge 2>/dev/null
    echo "Cleanup complete"
    exit 0
}

trap cleanup INT TERM

# Monitor processes
while true; do
    if ! ps -p $GAZEBO_PID > /dev/null; then
        echo "Warning: Gazebo process died unexpectedly"
        tail -10 /tmp/gazebo.log
        cleanup
    fi
    if ! ps -p $BRIDGE_PID > /dev/null; then
        echo "Warning: Bridge process died"
        cleanup
    fi
    sleep 5
done