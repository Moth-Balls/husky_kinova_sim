from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('husky_kinova_sim').find('husky_kinova_sim')
    xacro_file = os.path.join(pkg_share, 'description', 'husky.urdf.xacro')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', xacro_file])}]
        )
    ])
