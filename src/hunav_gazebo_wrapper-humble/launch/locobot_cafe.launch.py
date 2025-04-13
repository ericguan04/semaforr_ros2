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
    # Debugging
    set_log_level = SetLaunchConfiguration('log_level', 'debug')
    
    # Activate GPU for Gazebo
    use_nvidia_gpu = [
        '__NV_PRIME_RENDER_OFFLOAD=1 ',
        '__GLX_VENDOR_LIBRARY_NAME=nvidia ',
    ]

    

    # ----------------------------------------------------------
    #       Set world generation parameters
    # ----------------------------------------------------------
    world_file_name = LaunchConfiguration('base_world')
    gz_obs = LaunchConfiguration('use_gazebo_obs')
    rate = LaunchConfiguration('update_rate')
    robot_name = LaunchConfiguration('robot_name')
    global_frame = LaunchConfiguration('global_frame_to_publish')
    use_navgoal = LaunchConfiguration('use_navgoal_to_start')
    navgoal_topic = LaunchConfiguration('navgoal_topic')
    ignore_models = LaunchConfiguration('ignore_models')
    navigation = LaunchConfiguration('navigation')



    # ----------------------------------------------------------
    #       Declare Launch Arguments
    # ----------------------------------------------------------
    # Launch argument for world file
    declare_arg_world = DeclareLaunchArgument(
        'base_world', 
        default_value='empty_cafe.world',
        description='Specify world file name'
    )
    # Launch argument for agent configuration file
    declare_agents_conf_file = DeclareLaunchArgument(
        'configuration_file', 
        default_value='agents.yaml',
        description='Specify configuration file name in the cofig directory'
    )
    # Launch argument for metrics configuration file
    declare_metrics_conf_file = DeclareLaunchArgument(
        'metrics_file', 
        default_value='metrics.yaml',
        description='Specify the name of the metrics configuration file in the cofig directory'
    )
    # Launch argument for Gazebo obstacles
    declare_gz_obs = DeclareLaunchArgument(
        'use_gazebo_obs', default_value='True',
        description='Whether to fill the agents obstacles with closest Gazebo obstacle or not'
    )
    # Launch argument for update rate of the plugin
    declare_update_rate = DeclareLaunchArgument(
        'update_rate', default_value='100.0',
        description='Update rate of the plugin'
    )
    # Launch argument for robot name
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='locobot',
        description='Specify the name of the robot Gazebo model'
    )
    # Launch argument for global frame
    declare_frame_to_publish = DeclareLaunchArgument(
        'global_frame_to_publish', default_value='map',
        description='Name of the global frame in which the position of the agents are provided'
    )
    # Launch argument for using navigation goal to start
    declare_use_navgoal = DeclareLaunchArgument(
        'use_navgoal_to_start', default_value='False',
        description='Whether to start the agents movements when a navigation goal is received or not'
    )
    # Launch argument for navigation goal topic
    declare_navgoal_topic = DeclareLaunchArgument(
        'navgoal_topic', default_value='goal_pose',
        description='Name of the topic in which navigation goal for the robot will be published'
    )
    # Launch argument for ignoring models
    declare_ignore_models = DeclareLaunchArgument(
        'ignore_models', default_value='ground_plane cafe',
        description='list of Gazebo models that the agents should ignore as obstacles as the ground_plane. Indicate the models with a blank space between them'
    )
    declare_navigation = DeclareLaunchArgument(
        'navigation', default_value='False',
        description='If launch the pmb2 navigation system'
    )

    declare_arg_namespace = DeclareLaunchArgument(
        'robot_namespace', default_value='',
        description='The type of robot')



    # ----------------------------------------------------------
    #       Set up paths using launch arguments
    # ----------------------------------------------------------
    # Load World file path
    world_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        world_file_name   # LaunchConfiguration('base_world')
    ])

    # Load HuNav Gazebo world path
    world_path = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        'generatedWorld.world' #'empty_cafe.world' #'pmb2_cafe.world'
    ])

    # Load HuNav Gazebo models path
    my_gazebo_models = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'models',
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



    # ----------------------------------------------------------
    #       Setup HuNav Nodes
    # ----------------------------------------------------------
    # Read yaml configuration file and load parameters for HuNav agents
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

    # Load HuNav Gazebo World Generator
    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        output='screen',
        parameters=[{'base_world': world_file},
        {'use_gazebo_obs': gz_obs},
        {'update_rate': rate},
        {'robot_name': robot_name},
        {'global_frame_to_publish': global_frame},
        {'use_navgoal_to_start': use_navgoal},
        {'navgoal_topic': navgoal_topic},
        {'ignore_models': ignore_models}]
    )

    static_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        output='screen',
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        # other option: arguments = "0 0 0 0 0 0 pmb2 base_footprint".split(' ')
        condition=UnlessCondition(navigation)
    )



    # ----------------------------------------------------------
    #       Setup Environment in Gazebo
    # ----------------------------------------------------------
    # Environment variables for Gazebo
    
    # Load Gazebo configuration file
    config_file_name = 'params.yaml' 
    pkg_dir = get_package_share_directory('hunav_gazebo_wrapper') 
    config_file = path.join(pkg_dir, 'launch', config_file_name) 
    model, plugin, media = GazeboRosPaths.get_paths()

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']

    env = {
        'GAZEBO_MODEL_PATH': model,
        'GAZEBO_PLUGIN_PATH': plugin,
        'GAZEBO_RESOURCE_PATH': media
    }
    print('env:', env)

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



    # ----------------------------------------------------------
    #       Setup Gazebo Server and Client
    # ----------------------------------------------------------
    gzserver_cmd = [
        # use_nvidia_gpu,
        'gzserver ',
        world_path, 
        '-s ', 'libgazebo_ros_init.so',
        '-s ', 'libgazebo_ros_factory.so',
        '--ros-args',
        '--params-file', config_file,
    ]
    gzclient_cmd = [
        # use_nvidia_gpu,
        'gzclient',
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
        cmd=gzclient_cmd,
        output='screen',
        shell=True,
        on_exit=Shutdown(),
    )



    # ----------------------------------------------------------
    #       Setup LoCoBot in Gazebo
    # ----------------------------------------------------------
    # Path for LoCoBot launch file from interbotix workspace
    locobot_gazebo_launch = PathJoinSubstitution([
        FindPackageShare('interbotix_xslocobot_sim'),
        'launch',
        'xslocobot.launch.py'
    ])

    map_dir = get_package_share_directory('hunav_rviz2_panel') 
    map_path = path.join(map_dir, 'maps', 'map_cafe2.yaml') 

    # Launch LoCoBot in Gazebo using path to launch file
    locobot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([locobot_gazebo_launch]),
        launch_arguments={
            'robot_model': 'locobot_base',
            'use_rviz': 'true',
            'world_filepath': map_path,
            'use_sim_time': 'true'
        }.items(),
    )



    # ----------------------------------------------------------
    #       Simulation Launch Events
    # ----------------------------------------------------------
    # HUNAV
    hunav_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='HunNavLoader started, launching HuNav_Gazebo_world_generator after 3 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[hunav_gazebo_worldgen_node],
                )
            ]
        )
    )

    # GAZEBO
    gz_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_gazebo_worldgen_node,
            on_start=[
                LogInfo(msg='GenerateWorld started, launching Gazebo after 3 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[gzserver_process, gzclient_process],
                )
            ]
        )
    )

    # LOCOBOT
    locobot_launch_event = RegisterEventHandler(
        OnProcessExit(
            target_action=hunav_gazebo_worldgen_node,
            on_exit=[
                LogInfo(msg='GenerateWorld finished, launching LoCoBot robot after 5 seconds...'),
                TimerAction(
                    period=5.0,
                    actions=[locobot_gazebo],
                )
            ]
        )
    )
    


    # ----------------------------------------------------------
    #       Launch Description
    # ----------------------------------------------------------
    ld = LaunchDescription()
    # ld.add_action(set_log_level)

    # Environment variables
    ld.add_action(set_env_gazebo_model)
    ld.add_action(set_env_gazebo_resource)
    ld.add_action(set_env_gazebo_plugin)

    # Launch Arguments
    ld.add_action(declare_agents_conf_file)
    ld.add_action(declare_metrics_conf_file)
    ld.add_action(declare_arg_world)
    ld.add_action(declare_gz_obs)
    ld.add_action(declare_update_rate)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_frame_to_publish)
    ld.add_action(declare_use_navgoal)
    ld.add_action(declare_navgoal_topic)
    ld.add_action(declare_ignore_models)
    ld.add_action(declare_navigation)
    ld.add_action(declare_arg_namespace)

    # Generate the world with the agents
    # launch hunav_loader and the WorldGenerator 2 seconds later
    ld.add_action(hunav_loader_node)
    ld.add_action(hunav_launch_event)

    # hunav behavior manager node
    ld.add_action(hunav_manager_node)
    # hunav evaluator
    ld.add_action(hunav_evaluator_node)

    # launch Gazebo safter WorldGenerator
    ld.add_action(gz_launch_event)
    ld.add_action(static_tf_node)
    
    # Spawn LoCoBot in Gazebo
    ld.add_action(locobot_launch_event)
    
    return ld