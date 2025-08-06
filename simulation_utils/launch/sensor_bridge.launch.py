#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to simulation_utils (relative to workspace root)
    workspace_root = os.path.join(os.path.dirname(__file__), '..', '..')
    world_file = os.path.join(workspace_root, 'simulation_utils', 'worlds', 'sensor_world_headless.sdf')

    # Declare launch arguments
    use_openpose = LaunchConfiguration('use_openpose', default='false')

    return LaunchDescription([
        # Launch argument to control OpenPose
        DeclareLaunchArgument(
            'use_openpose',
            default_value='false',
            description='Launch OpenPose node'
        ),

        # Start Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_file],
            output='screen',
            shell=False
        ),

        # Bridge camera topic - remapped to match OpenPose expectation
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            arguments=['/camera@sensor_msgs/msg/Image@ignition.msgs.Image'],
            remappings=[
                ('/camera', '/camera/image_raw')  # Remap to OpenPose's expected topic
            ],
            output='screen'
        ),

        # Optional: Launch OpenPose if requested
        # Uncomment and adjust package name as needed
        # Node(
        #     package='your_openpose_package_name',  # Replace with actual package name
        #     executable='openpose_node',
        #     name='openpose_node',
        #     output='screen',
        #     condition=IfCondition(use_openpose)
        # ),
    ])