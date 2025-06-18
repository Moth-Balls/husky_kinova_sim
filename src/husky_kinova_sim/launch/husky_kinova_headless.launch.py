import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robotXacroName='husky'
    namePackage = 'husky_kinova_sim'
    kinova_arm_package = 'kinova_arm' 
    modelFileRelativePath = 'description/husky.urdf.xacro'
    worldFileRelativePath = 'worlds/plant_row.world'
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    
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

    gazeboLaunch=IncludeLaunchDescription(
        gazebo_rosPackagelaunch,
        launch_arguments={'gz_args': ['-v4 -s -r ', pathWorldFile], 'on_exit_shutdown': "true"}.items()
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
            '-x', '-4.5',
            '-y', '4.5',
            '-z', '0.3',
        ],
        output='screen',
    )

    # Spawner for Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager", # Targets the CM from gz_ros2_control
            "--ros-args",
            "--params-file", controllers_yaml_path # Pass the YAML for parameters
        ],
        output="screen",
    )

    # Spawner for Arm Controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager", # Targets the CM from gz_ros2_control
            "--ros-args",
            "--params-file", controllers_yaml_path # Pass the YAML for parameters
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
    
    # Declare the launch argument
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true", # Default to true for simulation
            description="Use simulation (Gazebo) clock if true",
        )
    )

    move_group_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('kinova_arm'),
            '/launch/move_group.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'))
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(robotStatePublisher)
    launchDescriptionObject.add_action(spawnModelGazebo)
    
    # Add event handlers to spawn controllers after the model is spawned in Gazebo,
    # ensuring the /controller_manager service from gz_ros2_control is likely available.
    launchDescriptionObject.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawnModelGazebo,
            on_exit=[joint_state_broadcaster_spawner],
        )
    ))
    launchDescriptionObject.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            # Spawn arm_controller after joint_state_broadcaster_spawner finishes (or also after spawnModelGazebo)
            # target_action=joint_state_broadcaster_spawner, 
            target_action=spawnModelGazebo, # Let's try spawning them in parallel after model spawn
            on_exit=[arm_controller_spawner],
        )
    ))

    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    launchDescriptionObject.add_action(move_group_launch_include)
    return launchDescriptionObject