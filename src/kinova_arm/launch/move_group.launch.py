import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    # Add other arguments if needed, e.g., for RViz, specific robot model
    # For simplicity, focusing on move_group and use_sim_time

    # LaunchConfiguration variables
    use_sim_time_lc = LaunchConfiguration("use_sim_time")

    # This OpaqueFunction is used to resolve LaunchConfigurations to strings when needed for paths
    def configure_move_group(context):
        # Define package name and robot name
        # Assuming your MoveIt config files are in 'kinova_arm' package
        # And your robot configuration files are named based on 'husky'
        # ADJUST 'robot_name_str' IF YOUR CONFIG FILES USE A DIFFERENT NAME (e.g., 'kinova_gen3')
        package_name_moveit_config = "kinova_arm"
        robot_name_str = "husky"  # This should match the name in your SRDF and other config files

        # Get the package share directory
        pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)

        # Construct base path to config files for the specified robot
        # e.g., /path/to/your_ws/install/kinova_arm/share/kinova_arm/config/
        # If your files are directly under config (e.g. kinova_arm/config/husky.srdf), this is fine.
        # If they are under a robot-specific subfolder (e.g. kinova_arm/config/husky/husky.srdf), adjust accordingly.
        # The example used: os.path.join(pkg_share_moveit_config, 'config', robot_name_str)
        # For your current setup, let's assume files are directly in 'kinova_arm/config/'
        config_base_path = os.path.join(pkg_share_moveit_config, 'config')

        # Define paths to configuration files
        # Ensure these files exist at these locations:
        # <your_workspace>/install/kinova_arm/share/kinova_arm/config/husky.srdf (or actual robot name)
        # <your_workspace>/install/kinova_arm/share/kinova_arm/config/kinematics.yaml
        # ... and so on for all files.
        srdf_file_path = os.path.join(config_base_path, f"{robot_name_str}.srdf")
        kinematics_file_path = os.path.join(config_base_path, "kinematics.yaml")
        ompl_planning_file_path = os.path.join(config_base_path, "ompl_planning.yaml")
        moveit_controllers_file_path = os.path.join(config_base_path, "moveit_controllers.yaml")
        # initial_positions_file_path = os.path.join(config_base_path, "initial_positions.yaml") # No longer needed

        # Build MoveItConfigs object
        moveit_config_builder = MoveItConfigsBuilder(robot_name_str, package_name=package_name_moveit_config)
        moveit_config_builder.robot_description_semantic(file_path=srdf_file_path)
        moveit_config_builder.robot_description_kinematics(file_path=kinematics_file_path)
        moveit_config_builder.trajectory_execution(file_path=moveit_controllers_file_path)

        # Planning Pipelines
        # Adjust pipelines and default_planning_pipeline as per your setup
        # Ensure that files like 'ompl_planning.yaml', 'chomp_planning.yaml' (if used)
        # exist in your 'kinova_arm/config/' directory.
        moveit_config_builder.planning_pipelines(
            default_planning_pipeline="ompl",  # or "chomp", "stomp", etc.
            pipelines=["ompl"]  # Add "chomp", "stomp", "pilz_industrial_motion_planner" if configured
            # The 'pipeline_configs' argument is removed.
            # Individual YAML files are found by convention.
        )

        # Planning Scene Monitor
        # publish_robot_description should be false if robot_state_publisher is running elsewhere (it is in your husky_kinova.launch.py)
        moveit_config_builder.planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True
        )

        # Optional: Load joint_limits.yaml if you have it
        # joint_limits_file_path = os.path.join(config_base_path, "joint_limits.yaml")
        # if os.path.exists(joint_limits_file_path):
        #     moveit_config_builder.joint_limits(file_path=joint_limits_file_path)

        # Optional: Pilz planner limits if you use Pilz
        # pilz_cartesian_limits_file_path = os.path.join(config_base_path, "pilz_cartesian_limits.yaml")
        # if os.path.exists(pilz_cartesian_limits_file_path):
        #    moveit_config_builder.pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)

        moveit_configs_obj = moveit_config_builder.to_moveit_configs()

        # Load the SRDF to extract the "default" pose
        with open(srdf_file_path, 'r') as f:
            srdf_content = f.read()

        # Parse the SRDF content to extract the "default" pose
        import xml.etree.ElementTree as ET
        root = ET.fromstring(srdf_content)
        joint_values = {}
        for group_state in root.findall(".//group_state"):
            if group_state.get("name") == "default":
                for joint in group_state.findall("joint"):
                    joint_name = joint.get("name")
                    joint_value = float(joint.get("value"))
                    joint_values[joint_name] = joint_value
                break  # Stop after finding the "default" group state

        # Create move_group node
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_configs_obj.to_dict(),
                {"use_sim_time": use_sim_time_lc},  # Explicitly pass use_sim_time
                # Add other parameters like capabilities or initial_positions if needed
                # {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
                {'start_state': joint_values},  # Set the start state to the "default" pose
            ],
        )

        # If you want to launch RViz from here, you would define its Node here too,
        # similar to the example, passing use_sim_time_lc to it.

        return [move_group_node]  # Add rviz_node here if launching RViz

    ld = LaunchDescription(declared_arguments)
    ld.add_action(OpaqueFunction(function=configure_move_group))
    return ld
