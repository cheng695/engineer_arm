import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


def variant_config_dir(moveit_config_pkg, robot_version):
    normalized = robot_version.lower().replace(".", "_")
    variant_names = {
        "v1_0": "V1.0",
        "1_0": "V1.0",
        "v1_1": "V1.1",
        "1_1": "V1.1",
    }
    variant_name = variant_names.get(normalized)
    if not variant_name:
        return os.path.join(moveit_config_pkg, "config")

    variant_dir = os.path.join(moveit_config_pkg, "config", "variants", variant_name)
    return variant_dir if os.path.isdir(variant_dir) else os.path.join(moveit_config_pkg, "config")


def joint_control_directions(arm_version):
    normalized = arm_version.lower().replace(".", "_")
    if normalized in ("v1_1", "1_1"):
        return [1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0]
    return [1.0] * 7


def generate_launch_description():
    robot_version = LaunchConfiguration("robot", default="v1_0")
    arm_version = LaunchConfiguration("arm_version", default=robot_version)
    chassis_version = LaunchConfiguration("chassis_version", default=robot_version)
    gripper_version = LaunchConfiguration("gripper_version", default=robot_version)
    start_joy = LaunchConfiguration("start_joy", default="true")
    joy_dev = LaunchConfiguration("joy_dev", default="/dev/input/js0")
    joy_deadzone = LaunchConfiguration("joy_deadzone", default="0.05")
    joy_autorepeat_rate = LaunchConfiguration("joy_autorepeat_rate", default="100.0")
    joy_coalesce_interval = LaunchConfiguration("joy_coalesce_interval", default="0.01")

    # robot_description（Servo 需要）
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("my_robot_description"), "urdf", "my_robot.urdf.xacro"]),
        " arm_version:=", arm_version,
        " chassis_version:=", chassis_version,
        " gripper_version:=", gripper_version,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    moveit_config_pkg = get_package_share_directory("my_robot_moveit_config")

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

    def make_commander(context):
        selected_config_dir = variant_config_dir(moveit_config_pkg, robot_version.perform(context))

        # SRDF
        srdf_path = os.path.join(selected_config_dir, "my_robot.srdf")
        with open(srdf_path, "r") as f:
            srdf_content = f.read()
        robot_description_semantic = {"robot_description_semantic": srdf_content}

        # 运动学配置
        kinematics_path = os.path.join(selected_config_dir, "kinematics.yaml")
        kinematics_yaml = load_yaml(kinematics_path)
        robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

        # Servo 参数
        servo_path = os.path.join(selected_config_dir, "servo_config.yaml")
        servo_yaml = load_yaml(servo_path)
        servo_params = {}
        if servo_yaml and "moveit_servo" in servo_yaml:
            servo_params = {"moveit_servo": servo_yaml["moveit_servo"]["ros__parameters"]}

        use_servo_str = context.launch_configurations.get("use_servo", "false")
        use_servo_val = use_servo_str.lower() in ("true", "1", "yes", "on")
        selected_arm_version = arm_version.perform(context)
        node = Node(
            package="my_robot_commander_cpp",
            executable="joy_to_servo_node",
            output="screen",
            parameters=[robot_description, robot_description_semantic,
                        robot_description_kinematics, servo_params,
                        {
                            "use_servo": use_servo_val,
                            "arm_version": selected_arm_version,
                            "joint_control_directions": joint_control_directions(selected_arm_version),
                        }],
        )
        return [node]

    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="v1_0",
            description="Robot description variant shortcut: v1_0/v1_1 or v1.0/v1.1"),
        DeclareLaunchArgument("arm_version", default_value=robot_version,
            description="Arm description variant override: v1_0 or v1_1"),
        DeclareLaunchArgument("chassis_version", default_value=robot_version,
            description="Chassis description variant override: v1_0 or v1_1"),
        DeclareLaunchArgument("gripper_version", default_value=robot_version,
            description="Gripper description variant override: v1_0 or v1_1"),
        DeclareLaunchArgument("start_joy", default_value="true"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("joy_deadzone", default_value="0.05"),
        DeclareLaunchArgument("joy_autorepeat_rate", default_value="100.0"),
        DeclareLaunchArgument("joy_coalesce_interval", default_value="0.01"),
        DeclareLaunchArgument("use_servo", default_value="false",
            description="Enable MoveIt Servo. Set true for real robot, false for FDCC-only simulation."),
        joy_node,
        OpaqueFunction(function=make_commander),
    ])
