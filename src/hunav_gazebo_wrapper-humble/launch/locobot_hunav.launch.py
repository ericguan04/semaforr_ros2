from pathlib import Path

from interbotix_xs_modules.xs_common import (
    get_interbotix_xslocobot_models,
)
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xslocobot_robot_description_launch_arguments,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    rviz_config_launch_arg = LaunchConfiguration('rvizconfig')
    world_filepath_launch_arg = LaunchConfiguration('world_filepath')
    use_gazebo_gui_launch_arg = LaunchConfiguration('use_gazebo_gui')
    use_gazebo_verbose_launch_arg = LaunchConfiguration('use_gazebo_verbose')
    use_gazebo_debug_launch_arg = LaunchConfiguration('use_gazebo_debug')
    start_gazebo_paused_launch_arg = LaunchConfiguration('start_gazebo_paused')
    enable_gazebo_recording_launch_arg = LaunchConfiguration('enable_gazebo_recording')
    robot_description_launch_arg = LaunchConfiguration('robot_description')

    # Set ignition resource paths
    gz_resource_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value=''),
            ':',
            str(Path(
                FindPackageShare('interbotix_common_sim').perform(context)
            ).parent.resolve()),
            ':',
            str(Path(
                FindPackageShare('interbotix_xslocobot_descriptions').perform(context)
            ).parent.resolve()),
        ]
    )

    gz_model_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            '/usr/share/gazebo-11/models/',
            ':',
            str(Path(
                FindPackageShare('interbotix_common_sim').perform(context)
            ).parent.resolve()),
            ':',
            str(Path(
                FindPackageShare('interbotix_xslocobot_descriptions').perform(context)
            ).parent.resolve()),
        ]
    )

    gz_media_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MEDIA_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MEDIA_PATH', default_value=''),
            ':',
            str(Path(
                FindPackageShare('interbotix_common_sim').perform(context)
            ).parent.resolve()),
            ':',
            str(Path(
                FindPackageShare('interbotix_xslocobot_descriptions').perform(context)
            ).parent.resolve()),
        ]
    )

    # Set GAZEBO_MODEL_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_URI',
        value=['']
    )

    # Set GAZEBO_MODEL_DATABASE_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=['']
    )

    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ]),
        ]),
        launch_arguments={
            'verbose': use_gazebo_verbose_launch_arg,
            'world': world_filepath_launch_arg,
            'pause': start_gazebo_paused_launch_arg,
            'record': enable_gazebo_recording_launch_arg,
            'gdb': use_gazebo_debug_launch_arg,
            'valgrind': use_gazebo_debug_launch_arg,
            'gui': use_gazebo_gui_launch_arg,
        }.items(),
    )

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        namespace=robot_name_launch_arg,
        arguments=[
            '-entity', 'robot_description',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        output={'both': 'log'},
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            f'{robot_name_launch_arg.perform(context)}/controller_manager',
            'joint_state_broadcaster',
        ],
    )

    spawn_arm_controller_node = Node(
        condition=LaunchConfigurationNotEquals(
            launch_configuration_name='robot_model',
            expected_value='locobot_base'
        ),
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            f'{robot_name_launch_arg.perform(context)}/controller_manager',
            'arm_controller',
        ],
    )

    spawn_gripper_controller_node = Node(
        condition=IfCondition(LaunchConfiguration('use_gripper')) and
        LaunchConfigurationNotEquals(
            launch_configuration_name='robot_model',
            expected_value='locobot_base'
        ),
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            f'{robot_name_launch_arg.perform(context)}/controller_manager',
            'gripper_controller',
        ],
    )

    spawn_camera_controller_node = Node(
        name='camera_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            f'{robot_name_launch_arg.perform(context)}/controller_manager',
            'camera_controller',
        ],
    )

    spawn_diffdrive_controller_node = Node(
        name='diffdrive_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            f'{robot_name_launch_arg.perform(context)}/controller_manager',
            'diffdrive_controller',
        ],
    )

    xslocobot_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xslocobot_descriptions'),
                'launch',
                'xslocobot_description.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'rvizconfig': rviz_config_launch_arg,
            'use_sim_time': 'true',
            'robot_description': robot_description_launch_arg,
        }.items(),
    )

    # spawn joint_state_broadcaster after robot is spawned
    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[spawn_joint_state_broadcaster_node]
        )
    )

    # spawn diffdrive_controller after joint_state_broadcaster is spawned
    load_diffdrive_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_diffdrive_controller_node]
        )
    )

    # spawn camera_controller after joint_state_broadcaster is spawned
    load_camera_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_camera_controller_node]
        )
    )

    # spawn arm_controller controller after joint_state_broadcaster is spawned
    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_arm_controller_node]
        )
    )

    # spawn gripper_controller controller after joint_state_broadcaster is spawned
    load_gripper_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_gripper_controller_node]
        )
    )

    return [
        gz_resource_path_env_var,
        gz_model_path_env_var,
        gz_media_path_env_var,
        gz_model_uri_env_var,
        gazebo_launch_include,
        spawn_robot_node,
        load_diffdrive_controller_event,
        load_camera_controller_event,
        load_joint_state_broadcaster_event,
        load_arm_controller_event,
        load_gripper_controller_event,
        xslocobot_description_launch_include,
    ]


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):
    # Set use_sim_time to True
    set_sim_time = SetLaunchConfiguration('use_sim_time', 'True')
    launch_description.add_action(set_sim_time)

    set_public_sim = SetLaunchConfiguration('is_public_sim', 'True')
    launch_description.add_action(set_public_sim)

    set_slam = SetLaunchConfiguration('slam', 'False')
    launch_description.add_action(set_slam)

    set_nav = SetLaunchConfiguration('navigation', 'True')
    launch_description.add_action(set_nav)

    set_log_level = SetLaunchConfiguration('log_level', 'debug')
    #launch_description.add_action(set_log_level)

    # Shows error if is_public_sim is not set to True when using public simulation
    public_sim_check = CheckPublicSim()
    launch_description.add_action(public_sim_check)

    robot_name = 'pmb2'

    pkg_path = get_package_prefix("pmb2_description")
    model_path = os.path.join(pkg_path, "share")
    resource_path = pkg_path

    packages = ["pmb2_description"]

    # model_path = get_model_paths(packages)

    if "GAZEBO_MODEL_PATH" in environ:
        model_path += pathsep + environ["GAZEBO_MODEL_PATH"]

    if "GAZEBO_RESOURCE_PATH" in environ:
        resource_path += pathsep + environ["GAZEBO_RESOURCE_PATH"]

    # model_path = "/home/upo/pmb2_public_ws/install/pmb2_description/share:/usr/share/gazebo-11/models:"

    print("\n\n\n\nmodel in pal_test after first if:", model_path, "\n\n\n")

    launch_description.add_action(
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)
    )


    pmb2_2dnav = get_package_share_directory("pmb2_2dnav")

    nav_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "navigation_launch.py"
    ],)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav_launch]),
        launch_arguments={
            "params_file": os.path.join(
                pmb2_2dnav, "params", "pmb2_nav_public_sim.yaml"
            ),
            "use_sim_time": "True",
        }.items(),
        condition=IfCondition(LaunchConfiguration("navigation"))
    )

    slam_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "slam_launch.py"
    ],)
    slam_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_launch]),
        launch_arguments={
            "params_file": os.path.join(
                pmb2_2dnav, "params", "pmb2_nav_public_sim.yaml"
            ),
            "use_sim_time": "True",
        }.items(),
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    loc_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "localization_launch.py"
    ],)
    loc_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([loc_launch]),
        launch_arguments={
            "params_file": os.path.join(
                pmb2_2dnav, "params", "pmb2_nav_public_sim.yaml"
            ),
            "map": LaunchConfiguration("world_name"),
            "use_sim_time": "True",
        }.items(),
        condition=(IfCondition(LaunchConfiguration("navigation"))),
    )

    rviz_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "rviz_launch.py"
    ],)
    rviz_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments={
            "rviz_config": os.path.join(
                pmb2_2dnav, "config", "rviz", "navigation.rviz"
            ),
            "use_sim_time": "True",
        }.items(),
        condition=IfCondition(LaunchConfiguration("navigation"))
    )

    launch_description.add_action(nav2_bringup_launch)
    launch_description.add_action(loc_bringup_launch)
    launch_description.add_action(slam_bringup_launch)
    launch_description.add_action(rviz_bringup_launch)



    # ----------------------------------------------------------
    #    ROBOT SPAWN
    # ----------------------------------------------------------
    print("Launching robot_spawn.launch.py")
    print("robot name: ", robot_name)


    # ORIGINAL
    robot_spawn = include_scoped_launch_py_description(
        pkg_name='pmb2_gazebo',
        paths=['launch', 'robot_spawn.launch.py'],
        launch_arguments={
            "robot_name": robot_name,
            "x": launch_args.x,
            "y": launch_args.y,
            "z": "0.15",
            "yaw": launch_args.yaw,
        }
    )

    launch_description.add_action(robot_spawn)


    #----------------------------------------------------------
    #   ROBOT BRINGUP
    #----------------------------------------------------------
    print("Launching pmb2_bringup.launch.py")
    pmb2_bringup = include_scoped_launch_py_description(
        pkg_name='pmb2_bringup', paths=['launch', 'pmb2_bringup.launch.py'],
        launch_arguments={
            #'wheel_model': launch_args.wheel_model,
            'laser_model': launch_args.laser_model,
            'add_on_module': launch_args.add_on_module,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'is_public_sim': launch_args.is_public_sim,
        }
    )
    launch_description.add_action(pmb2_bringup)



def get_model_paths(packages_names):
    model_paths = ''
    for package_name in packages_names:
        if model_paths != '':
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, 'share')

        model_paths += model_path

    if 'GAZEBO_MODEL_PATH' in environ:
        model_paths += pathsep + environ['GAZEBO_MODEL_PATH']

    return model_paths
