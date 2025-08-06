#!/bin/bash

echo "Testing Camera Setup"
echo "==================="

# Check Gazebo topics
echo "Gazebo topics:"
ign topic -l | grep camera

echo ""
echo "ROS2 topics:"
ros2 topic list | grep camera

echo ""
echo "Camera data rate:"
ros2 topic hz /camera/image_raw --window 10

echo ""
echo "To visualize:"
echo "  ros2 run rqt_image_view rqt_image_view"