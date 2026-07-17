import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    robot_version = LaunchConfiguration("robot", default="v1_0")
    arm_version = LaunchConfiguration("arm_version", default=robot_version)
    chassis_version = LaunchConfiguration("chassis_version", default=robot_version)
    gripper_version = LaunchConfiguration("gripper_version", default=robot_version)
    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="my_robot_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("my_robot_description"),
                "urdf", "my_robot.urdf.xacro"),
            mappings={
                "arm_version": arm_version,
                "chassis_version": chassis_version,
                "gripper_version": gripper_version,
            })
        .to_moveit_configs()
    )
    ld = generate_demo_launch(moveit_config)
    ld.add_action(DeclareLaunchArgument("robot", default_value="v1_0",
        description="Robot description variant shortcut: v1_0/v1_1 or v1.0/v1.1"))
    ld.add_action(DeclareLaunchArgument("arm_version", default_value=robot_version,
        description="Arm description variant override: v1_0 or v1_1"))
    ld.add_action(DeclareLaunchArgument("chassis_version", default_value=robot_version,
        description="Chassis description variant override: v1_0 or v1_1"))
    ld.add_action(DeclareLaunchArgument("gripper_version", default_value=robot_version,
        description="Gripper description variant override: v1_0 or v1_1"))
    return ld
