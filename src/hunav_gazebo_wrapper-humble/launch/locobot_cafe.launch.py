from os import path, environ, pathsep
from scripts import GazeboRosPaths
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, SetEnvironmentVariable, 
    DeclareLaunchArgument, ExecuteProcess, Shutdown, 
    RegisterEventHandler, TimerAction, LogInfo, SetLaunchConfiguration
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution, LaunchConfiguration, 
    PythonExpression, EnvironmentVariable
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    set_log_level = SetLaunchConfiguration('log_level', 'debug')
    
    # Activate GPU for Gazebo
    use_nvidia_gpu = [
        '__NV_PRIME_RENDER_OFFLOAD=1 ',
        '__GLX_VENDOR_LIBRARY_NAME=nvidia ',
    ]

    # Set environment variables for Gazebo
    set_env_gazebo_model = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), PathJoinSubstitution([FindPackageShare('hunav_gazebo_wrapper'), 'models'])]
    )
    set_env_gazebo_resource = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH', 
        value=[EnvironmentVariable('GAZEBO_RESOURCE_PATH'), PathJoinSubstitution([FindPackageShare('hunav_gazebo_wrapper'), 'models'])]
    )
    set_env_gazebo_plugin = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH', 
        value=[EnvironmentVariable('GAZEBO_PLUGIN_PATH'), GazeboRosPaths.get_paths()[1]]
    )

    # Declare launch arguments
    # Launch argument for world file
    declare_arg_world = DeclareLaunchArgument(
        'base_world', 
        default_value='example_cafe.world',
        description='Specify world file name'
    )
    # Launch argument for agent configuration file
    declare_agent_conf = DeclareLaunchArgument(
        'configuration_file', 
        default_value='agents.yaml',
        description='Specify configuration file name for the simulated humans'
    )
    # Launch argument for metrics configuration file
    declare_metrics_conf_file = DeclareLaunchArgument(
        'metrics_file', 
        default_value='metrics.yaml',
        description='Specify the name of the metrics configuration file in the config directory'
    )

    # Set up paths using launch arguments
    # Load World file path
    world_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        LaunchConfiguration('base_world')
    ])

    # Load HuNav agent manager (for simulated humans) path
    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_agent_manager'),
        'config',
        LaunchConfiguration('configuration_file')
    ])

    # Load HuNav evaluator path
    metrics_file = PathJoinSubstitution([
        FindPackageShare('hunav_evaluator'),
        'config',
        LaunchConfiguration('metrics_file')
    ])

    # Load HuNav Nodes (agent manager, loader, evaluator)
    # Load HuNav agent manager node using the configuration file
    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[agent_conf_file]
    )

    # Load HuNav agent manager behavior
    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Load HuNav evaluator using the metrics configuration file
    hunav_evaluator_node = Node(
        package='hunav_evaluator',
        executable='hunav_evaluator_node',
        output='screen',
        parameters=[metrics_file]
    )


    # TESTING

    config_file_name = 'params.yaml' 
    pkg_dir = get_package_share_directory('hunav_gazebo_wrapper') 
    config_file = path.join(pkg_dir, 'launch', config_file_name) 

    gzserver_cmd = [
        use_nvidia_gpu,
        'gzserver ',
        world_file,
        '-s ', 'libgazebo_ros_init.so',
        '-s ', 'libgazebo_ros_factory.so',
        '-s ', 'libgazebo_ros_state.so',
        '--ros-args',
        '--params-file', config_file,
    ]

    # Launch Gazebo Server
    gzserver_process = ExecuteProcess(
        #cmd=[use_nvidia_gpu, 'gzserver', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        cmd=gzserver_cmd,
        output='screen',
        shell=True,
        on_exit=Shutdown(),
    )
    gzclient_process = ExecuteProcess(
        cmd=[use_nvidia_gpu, 'gzclient'],
        output='screen',
        shell=True,
        on_exit=Shutdown(),
    )

    # Spawn LoCoBot in Gazebo
    # Path for LoCoBot launch file from interbotix workspace
    locobot_gazebo_launch = PathJoinSubstitution([
        FindPackageShare('interbotix_xslocobot_sim'),
        'launch',
        'xslocobot.launch.py'
    ])
    # Launch LoCoBot in Gazebo using path to launch file
    locobot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([locobot_gazebo_launch]),
        launch_arguments={
            'robot_model': 'locobot_base',
            'use_rviz': 'true',
            'world_filepath': world_file,
            'use_sim_time': 'true'
        }.items(),
    )
    
    # Delay spawning LoCoBot and HuNav until Gazebo is ready
    locobot_spawn_event = RegisterEventHandler(
        OnProcessStart(
            target_action=gzserver_process,
            on_start=[
                LogInfo(msg='Gazebo launched, spawning LoCoBot and HuNav agent...'),
                TimerAction(
                    period=3.0,
                    actions=[locobot_gazebo, hunav_loader_node, hunav_manager_node, hunav_evaluator_node]
                )
            ]
        )
    )
    
    # Launch Description
    ld = LaunchDescription()
    ld.add_action(set_log_level)
    ld.add_action(set_env_gazebo_model)
    ld.add_action(set_env_gazebo_resource)
    ld.add_action(set_env_gazebo_plugin)
    ld.add_action(declare_arg_world)
    ld.add_action(declare_agent_conf)
    ld.add_action(declare_metrics_conf_file)
    ld.add_action(gzserver_process)
    ld.add_action(gzclient_process)
    ld.add_action(locobot_spawn_event)
    
    return ld