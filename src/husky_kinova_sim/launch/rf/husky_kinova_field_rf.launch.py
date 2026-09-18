import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction, AppendEnvironmentVariable, GroupAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robotXacroName = 'husky'
    namePackage = 'husky_kinova_sim'
    kinova_arm_package = 'kinova_arm'

    main_ugv_file_relative_path = 'description/husky.urdf.xacro'
    lora_node_file_relative_path = 'description/lora_node/node.urdf.xacro'

    pathModelFile = os.path.join(get_package_share_directory(namePackage), main_ugv_file_relative_path)
    lora_node_model_file = os.path.join(get_package_share_directory(namePackage), lora_node_file_relative_path)

    worldFileRelativePath = os.path.join(
        'worlds', 'field_world', 'field.world'
    )
    pathWorldFile = os.path.join(
        get_package_share_directory(namePackage), worldFileRelativePath
    )

    # Export model path for virtual_maize_field meshes
    set_gazebo_models = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(get_package_share_directory('virtual_maize_field'), 'models')
    )

    # Construct the path to ros2_controllers.yaml
    controllers_yaml_path = os.path.join(
        get_package_share_directory(kinova_arm_package),
        "config",
        "ros2_controllers.yaml"
    )

    robotDescription = xacro.process_file(pathModelFile).toxml()

    gazebo_rosPackagelaunch=PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )

    # Get path to the empty YAML file
    empty_yaml_path = os.path.join(
        get_package_share_directory(namePackage),
        'config',
        'empty.yaml'
    )

    gazeboLaunch=IncludeLaunchDescription(
        gazebo_rosPackagelaunch,
        launch_arguments={
            'gz_args': [' -r -v4 ', pathWorldFile],
            'on_exit_shutdown': "true",
            'initial_positions_file': empty_yaml_path
        }.items()
    )

    robotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
                     'use_sim_time': use_sim_time}]
    )

    spawnModelGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '9.0',
            '-z', '1.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '-1.5708',
        ],
        output='screen',
    )

    # Spawner for Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--ros-args",
            "-- params-file", controllers_yaml_path
        ],
        output="screen",
    )

    # Spawner for Arm Controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--ros-args",
            "--params-file", controllers_yaml_path
        ],
        output="screen",
    )

    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'config',
        'bridge_params.yaml'
    )
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ],
        output='screen'
    )

    move_group_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('kinova_arm'),
            '/launch/move_group.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'load_controllers': ' false'
        }.items(),
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(set_gazebo_models)
    launchDescriptionObject.add_action( DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'))
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(robotStatePublisher)
    launchDescriptionObject.add_action(spawnModelGazebo)

    # --- Lora Node Spawning ---
    node_positions = [
        {'x': -0.5868,  'y': 0.0389,  'z': 0.5},
    ]

    for i, pos in enumerate(node_positions):
        node_name = f"lora_node_{i+1}"
        rf_address = str(i+2)

        node_description = xacro.process_file(
            lora_node_model_file,
            mappings={
                'node_name': node_name,
                'rf_address': rf_address
            }
        ).toxml()

        node_group = GroupAction([
            PushRosNamespace(node_name),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': node_description,
                    'use_sim_time': use_sim_time,
                    'frame_prefix': f"{node_name}/"
                }],
                remappings=[
                    ('/tf_static', '/tf_static'),
                    ('tf_static', '/tf_static'),
                    ('/tf', '/tf'),
                    ('tf', '/tf'),
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '--x', str(pos['x']),
                    '--y', str(pos['y']),
                    '--z', str(pos['z']),
                    '--yaw', '0.0',
                    '--pitch', '0.0',
                    '--roll', '0.0',
                    '--frame-id', 'odom',
                    '--child-frame-id', f"{node_name}/base_link"
                ],
                remappings=[
                    ('/tf_static', '/tf_static'),
                    ('tf_static', '/tf_static'),
                    ('/tf', '/tf'),
                    ('tf', '/tf'),
                ]
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', node_name,
                    '-string', node_description,
                    '-x', str(pos['x']),
                    '-y', str(pos['y']),
                    '-z', str(pos['z']),
                ],
                output='screen',
            )
        ])
        launchDescriptionObject.add_action(node_group)

    launchDescriptionObject.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawnModelGazebo,
            on_exit=[joint_state_broadcaster_spawner],
        )
    ))
    launchDescriptionObject.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            # Spawn arm_controller after joint_state_broadcaster_spawner finishes 
            target_action=joint_state_broadcaster_spawner,
            # target_action=spawnModelGazebo, # Let's try spawning them in parallel after model spawn
            on_exit=[TimerAction(period=2.0, actions=[arm_controller_spawner])], # Delay arm_controller
        )
    ))

    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    launchDescriptionObject.add_action(move_group_launch_include)
    return launchDescriptionObject