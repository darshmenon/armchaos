import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    mc = (MoveItConfigsBuilder("moveit_resources_fanuc")
          .robot_description(file_path="config/fanuc.urdf.xacro")
          .robot_description_semantic(file_path="config/fanuc.srdf")
          .trajectory_execution(file_path="config/moveit_controllers.yaml")
          .to_moveit_configs())
    yaml = os.path.join(get_package_share_directory("moveit_resources_fanuc_moveit_config"), "config", "ros2_controllers.yaml")
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[mc.robot_description]),
        Node(package="controller_manager", executable="ros2_control_node", parameters=[yaml],
             remappings=[("/controller_manager/robot_description", "/robot_description")]),
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner", arguments=["fanuc_controller", "--controller-manager", "/controller_manager"]),
    ])
