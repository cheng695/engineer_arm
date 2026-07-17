import os
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 路径定义
    description_package = "my_robot_description"
    bringup_package = "my_robot_bringup"

    # 启动参数（对齐 openarm 命名习惯）
    robot_version = LaunchConfiguration("robot", default="v1_0")
    arm_version = LaunchConfiguration("arm_version", default=robot_version)
    chassis_version = LaunchConfiguration("chassis_version", default=robot_version)
    gripper_version = LaunchConfiguration("gripper_version", default=robot_version)
    use_mock_hardware = LaunchConfiguration("use_mock_hardware", default="false")
    gravity_compensation_mode = LaunchConfiguration("gravity_compensation_mode", default="off")
    active_real_joints = LaunchConfiguration("active_real_joints", default="")
    can0_interface = LaunchConfiguration("can0_interface", default="can0")
    can1_interface = LaunchConfiguration("can1_interface", default="can1")
    controllers_file = LaunchConfiguration("controllers_file", default="arm_controllers.yaml")
    description_xacro_file = os.path.join(
        get_package_share_directory(description_package),
        "urdf",
        "my_robot.urdf.xacro")
    urdf_file = "/tmp/robot_description.urdf"

    def generate_urdf_file(context):
        args = [
            "xacro",
            description_xacro_file,
            f"arm_version:={arm_version.perform(context)}",
            f"chassis_version:={chassis_version.perform(context)}",
            f"gripper_version:={gripper_version.perform(context)}",
            f"use_mock_hardware:={use_mock_hardware.perform(context)}",
            f"gravity_compensation_mode:={gravity_compensation_mode.perform(context)}",
            f"active_real_joints:={active_real_joints.perform(context)}",
            f"can0_interface:={can0_interface.perform(context)}",
            f"can1_interface:={can1_interface.perform(context)}",
            f"robot_description_file:={urdf_file}",
        ]
        with open(urdf_file, "w") as f:
            subprocess.run(args, stdout=f, env=os.environ.copy(), check=True)
        return []

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", "my_robot.urdf.xacro"]
            ),
            " arm_version:=", arm_version,
            " chassis_version:=", chassis_version,
            " gripper_version:=", gripper_version,
            " use_mock_hardware:=", use_mock_hardware,
            " gravity_compensation_mode:=", gravity_compensation_mode,
            " active_real_joints:=", active_real_joints,
            " can0_interface:=", can0_interface,
            " can1_interface:=", can1_interface,
            " robot_description_file:=", urdf_file,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(bringup_package),
            "config",
            controllers_file,
        ]
    )

    # 加载控制增益（Python dict，非 ROS2 YAML 参数文件格式）
    control_gains_path = os.path.join(
        get_package_share_directory(bringup_package),
        "config", "control_gains.yaml")
    with open(control_gains_path, "r") as f:
        control_gains = yaml.safe_load(f)

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, control_gains],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    forward_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # arm_controller 在 joint_state_broadcaster 之后启动
    spawn_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        spawn_arm_controller_event,
        forward_velocity_controller_spawner,
        gripper_controller_spawner,
    ]

    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
            description="Use mock hardware (GenericSystem) instead of real CAN hardware"),
        DeclareLaunchArgument("robot", default_value="v1_0",
            description="Robot description variant shortcut: v1_0/v1_1 or v1.0/v1.1"),
        DeclareLaunchArgument("arm_version", default_value=robot_version,
            description="Arm description variant override: v1_0 or v1_1"),
        DeclareLaunchArgument("chassis_version", default_value=robot_version,
            description="Chassis description variant override: v1_0 or v1_1"),
        DeclareLaunchArgument("gripper_version", default_value=robot_version,
            description="Gripper description variant override: v1_0 or v1_1"),
        DeclareLaunchArgument("gravity_compensation_mode", default_value="off",
            description="Gravity compensation mode: off, assist, gravity_only"),
        DeclareLaunchArgument("active_real_joints", default_value="",
            description="Comma-separated list of joint names using real CAN I/O"),
        DeclareLaunchArgument("can0_interface", default_value="can0",
            description="CAN interface for joints 1-4"),
        DeclareLaunchArgument("can1_interface", default_value="can1",
            description="CAN interface for joints 5-7"),
        OpaqueFunction(function=generate_urdf_file),
        *nodes,
    ])
