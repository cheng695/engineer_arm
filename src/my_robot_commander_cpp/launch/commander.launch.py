import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(path):
    try:
        with open(path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    start_joy = LaunchConfiguration("start_joy", default="true")
    joy_dev = LaunchConfiguration("joy_dev", default="/dev/input/js0")
    joy_deadzone = LaunchConfiguration("joy_deadzone", default="0.05")
    joy_autorepeat_rate = LaunchConfiguration("joy_autorepeat_rate", default="100.0")
    joy_coalesce_interval = LaunchConfiguration("joy_coalesce_interval", default="0.01")

    # robot_description（Servo 需要）
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("my_robot_description"), "urdf", "my_robot.urdf.xacro"]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # SRDF（Servo 需要）
    srdf_path = os.path.join(
        get_package_share_directory("my_robot_moveit_config"),
        "config", "my_robot.srdf")
    with open(srdf_path, "r") as f:
        srdf_content = f.read()
    robot_description_semantic = {"robot_description_semantic": srdf_content}

    # 运动学配置（Servo IK 需要，必须和 RViz 用的保持一致）
    kinematics_path = os.path.join(
        get_package_share_directory("my_robot_moveit_config"),
        "config", "kinematics.yaml")
    kinematics_yaml = load_yaml(kinematics_path)
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Servo 参数
    servo_path = os.path.join(
        get_package_share_directory("my_robot_moveit_config"),
        "config", "servo_config.yaml")
    servo_yaml = load_yaml(servo_path)
    servo_params = {}
    if servo_yaml and "moveit_servo" in servo_yaml:
        servo_params = {"moveit_servo": servo_yaml["moveit_servo"]["ros__parameters"]}

    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        parameters=[{
            "dev": joy_dev,
            "deadzone": joy_deadzone,
            "autorepeat_rate": joy_autorepeat_rate,
            "coalesce_interval": joy_coalesce_interval,
            "sticky_buttons": False,
        }],
        condition=IfCondition(start_joy),
    )

    joy_to_servo_node = Node(
        package="my_robot_commander_cpp",
        executable="joy_to_servo_node",
        output="screen",
        parameters=[robot_description, robot_description_semantic, robot_description_kinematics, servo_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_joy", default_value="true"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("joy_deadzone", default_value="0.05"),
        DeclareLaunchArgument("joy_autorepeat_rate", default_value="100.0"),
        DeclareLaunchArgument("joy_coalesce_interval", default_value="0.01"),
        joy_node,
        joy_to_servo_node,
    ])
