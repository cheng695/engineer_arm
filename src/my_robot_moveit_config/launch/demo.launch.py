import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="my_robot_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("my_robot_description"),
                "urdf", "my_robot.urdf.xacro"))
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
