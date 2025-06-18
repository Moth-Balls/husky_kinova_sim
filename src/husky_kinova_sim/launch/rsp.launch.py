from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    package_name = 'husky_kinova_sim'
    urdf_relative_path = 'description/husky.urdf.xacro'

    urdf_path = os.path.join(FindPackageShare(package=package_name).find(package_name), urdf_relative_path)

    robot_description = Command(['xacro', ' ', urdf_path])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation / Gazebo clock if true'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}]
        )
    ])