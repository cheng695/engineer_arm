import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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

    # Load Servo parameters
    servo_params_file = os.path.join(
        get_package_share_directory("my_robot_moveit_config"),
        "config",
        "servo_params.yaml"
    )

    # Start the commander node
    commander_node = Node(
        package="my_robot_commander_cpp",
        executable="commander_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_params_file,
            {"use_sim_time": use_sim_time}
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        commander_node
    ])
