"""启动纯重力补偿测试。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory("my_robot_bringup")
    arm_bringup = os.path.join(bringup_dir, "launch", "arm_bringup.launch.py")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_bringup),
            launch_arguments={
                "gravity_compensation_mode": "external_gravity_only",
                "gravity_effort_scale": "1.0",
                "gravity_test_mode": "true",
                "start_commander": "false",
                "start_joy": "false",
            }.items(),
        )
    ])
