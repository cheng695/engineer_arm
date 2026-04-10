import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load MoveIt configurations
    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="my_robot_moveit_config")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("my_robot_description"),
            "urdf",
            "my_robot.urdf.xacro"
        ))
        .to_moveit_configs()
    )

    # Start the commander node
    commander_node = Node(
        package="my_robot_commander_cpp",
        executable="commander_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False}
        ],
    )

    return LaunchDescription([commander_node])
