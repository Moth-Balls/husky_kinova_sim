import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # This has to match the robot name in .xacro file
    robotXacroName='husky'

    # Name of package
    namePackage = 'husky_bringup'

    # Relative path to the xacro file defining the model
    modelFileRelativePath = 'description/husky.urdf.xacro'

    # Relative path to the world file
    worldFileRelativePath = 'worlds/room.world'

    # Absolute path to the robot model
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

    # Absolute path to the world file
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)

    # Converting .xacro to .urdf (getting robot model from xacro file)
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # This is the launch file from the gazebo_ros package
    gazebo_rosPackagelaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))

    # This is if you are using a custom world file
    gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackagelaunch, launch_arguments={'gz_args': ['-v4 -r ', pathWorldFile], 'on_exit_shutdown': "true"}.items())

    # Gazebo node
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

    # Robot state publisher node
    robotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
        'use_sim_time': True}]
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

    # Create empty launch description object
    launchDescriptionObject = LaunchDescription()

    # Add gazeboLaunch
    launchDescriptionObject.add_action(gazeboLaunch)

    # Add nodes
    launchDescriptionObject.add_action(spawnModelGazebo)
    launchDescriptionObject.add_action(robotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    return launchDescriptionObject