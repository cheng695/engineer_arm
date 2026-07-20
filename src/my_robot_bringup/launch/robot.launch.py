import os
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition

from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

def _variant_config_dir(moveit_config_pkg, robot_version):
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

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_version = LaunchConfiguration('robot', default='v1_0')
    arm_version = LaunchConfiguration('arm_version', default=robot_version)
    chassis_version = LaunchConfiguration('chassis_version', default=robot_version)
    gripper_version = LaunchConfiguration('gripper_version', default=robot_version)
    use_mock_hardware = LaunchConfiguration('use_mock_hardware', default='false')
    gravity_compensation_mode = LaunchConfiguration('gravity_compensation_mode', default='off')
    gravity_effort_scale = LaunchConfiguration('gravity_effort_scale', default='0.0')
    active_real_joints = LaunchConfiguration('active_real_joints', default='')
    can0_interface = LaunchConfiguration('can0_interface', default='can0')
    can1_interface = LaunchConfiguration('can1_interface', default='can1')
    start_commander = LaunchConfiguration('start_commander', default='true')
    start_joy = LaunchConfiguration('start_joy', default='true')
    joy_dev = LaunchConfiguration('joy_dev', default='/dev/input/js0')
    use_servo = LaunchConfiguration('use_servo', default='false')

    description_pkg = get_package_share_directory("my_robot_description")
    moveit_config_pkg = get_package_share_directory("my_robot_moveit_config")
    description_xacro_file = os.path.join(description_pkg, "urdf", "my_robot.urdf.xacro")

    # 预生成 URDF 到临时文件（GravityCompensator / FDCC 需要）
    urdf_file = "/tmp/robot_description.urdf"

    def launch_setup(context):
        selected_config_dir = _variant_config_dir(moveit_config_pkg, robot_version.perform(context))
        initial_positions_file = os.path.join(selected_config_dir, "initial_positions.yaml")
        ros2_controllers_file = os.path.join(selected_config_dir, "ros2_controllers.yaml")

        args = [
            "xacro",
            description_xacro_file,
            f"arm_version:={arm_version.perform(context)}",
            f"chassis_version:={chassis_version.perform(context)}",
            f"gripper_version:={gripper_version.perform(context)}",
            f"use_mock_hardware:={use_mock_hardware.perform(context)}",
            f"gravity_compensation_mode:={gravity_compensation_mode.perform(context)}",
            f"gravity_effort_scale:={gravity_effort_scale.perform(context)}",
            f"active_real_joints:={active_real_joints.perform(context)}",
            f"can0_interface:={can0_interface.perform(context)}",
            f"can1_interface:={can1_interface.perform(context)}",
            f"robot_description_file:={urdf_file}",
        ]
        with open(urdf_file, "w") as f:
            subprocess.run(args, stdout=f, env=os.environ.copy(), check=True)

        robot_description_content = Command([
            'xacro ', description_xacro_file,
            ' arm_version:=', arm_version,
            ' chassis_version:=', chassis_version,
            ' gripper_version:=', gripper_version,
            ' use_mock_hardware:=', use_mock_hardware,
            ' gravity_compensation_mode:=', gravity_compensation_mode,
            ' gravity_effort_scale:=', gravity_effort_scale,
            ' active_real_joints:=', active_real_joints,
            ' can0_interface:=', can0_interface,
            ' can1_interface:=', can1_interface,
            ' robot_description_file:=', urdf_file,
        ])
        robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

        moveit_config = (
            MoveItConfigsBuilder("my_robot", package_name="my_robot_moveit_config")
            .robot_description(
                file_path=description_xacro_file,
                mappings={
                    "use_mock_hardware": use_mock_hardware,
                    "arm_version": arm_version,
                    "chassis_version": chassis_version,
                    "gripper_version": gripper_version,
                    "initial_positions_file": initial_positions_file,
                    "gravity_compensation_mode": gravity_compensation_mode,
                    "gravity_effort_scale": gravity_effort_scale,
                    "active_real_joints": active_real_joints,
                    "can0_interface": can0_interface,
                    "can1_interface": can1_interface,
                },
            )
            .robot_description_semantic(file_path=os.path.join(selected_config_dir, "my_robot.srdf"))
            .robot_description_kinematics(file_path=os.path.join(selected_config_dir, "kinematics.yaml"))
            .joint_limits(file_path=os.path.join(selected_config_dir, "joint_limits.yaml"))
            .trajectory_execution(file_path=os.path.join(selected_config_dir, "moveit_controllers.yaml"))
            .sensors_3d(file_path=os.path.join(selected_config_dir, "sensors_3d.yaml"))
            .pilz_cartesian_limits(file_path=os.path.join(selected_config_dir, "pilz_cartesian_limits.yaml"))
            .planning_pipelines(pipelines=["ompl"])
            .to_moveit_configs()
        )
        moveit_params = moveit_config.to_dict()
        sensors_file = os.path.join(selected_config_dir, "sensors_3d.yaml")
        with open(sensors_file, "r") as f:
            sensors_config = yaml.safe_load(f) or {}
        if not sensors_config.get("sensors"):
            moveit_params.pop("sensors", None)
            for key in ("default_sensor", "kinect_depthimage"):
                moveit_params.pop(key, None)

        # ---- Nodes ----

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}]
        )

        bringup_pkg = get_package_share_directory("my_robot_bringup")
        commander_pkg = get_package_share_directory("my_robot_commander_cpp")

        # 加载控制增益（Python dict，非 ROS2 YAML 参数文件格式）
        control_gains_path = os.path.join(bringup_pkg, "config", "control_gains.yaml")
        with open(control_gains_path, "r") as f:
            control_gains = yaml.safe_load(f)

        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description,
                ros2_controllers_file,
                control_gains,
                {'use_sim_time': use_sim_time}
            ],
            output="screen",
        )

        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_params,
                {'use_sim_time': use_sim_time}
            ],
        )

        rviz_config_path = os.path.join(get_package_share_directory("my_robot_bringup"), "config", "rviz_config.rviz")
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
            parameters=[robot_description, moveit_params, {'use_sim_time': use_sim_time}],
        )

        # Controller Spawners
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

        gripper_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        )

        spawn_arm_controller_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner]
            )
        )

        spawn_gripper_controller_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[gripper_controller_spawner]
            )
        )

        commander_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(commander_pkg, "launch", "commander.launch.py")),
            launch_arguments={
                "robot": robot_version,
                "arm_version": arm_version,
                "chassis_version": chassis_version,
                "gripper_version": gripper_version,
                "start_joy": start_joy,
                "joy_dev": joy_dev,
                "use_servo": use_servo,
            }.items(),
            condition=IfCondition(start_commander),
        )

        return [
            robot_state_publisher,
            ros2_control_node,
            move_group_node,
            rviz_node,
            joint_state_broadcaster_spawner,
            spawn_arm_controller_event,
            spawn_gripper_controller_event,
            commander_launch,
        ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
            description='Use mock hardware (simulation) if true'),
        DeclareLaunchArgument('robot', default_value='v1_0',
            description='Robot description variant shortcut: v1_0/v1_1 or v1.0/v1.1'),
        DeclareLaunchArgument('arm_version', default_value=robot_version,
            description='Arm description variant override: v1_0 or v1_1'),
        DeclareLaunchArgument('chassis_version', default_value=robot_version,
            description='Chassis description variant override: v1_0 or v1_1'),
        DeclareLaunchArgument('gripper_version', default_value=robot_version,
            description='Gripper description variant override: v1_0 or v1_1'),
        DeclareLaunchArgument('gravity_compensation_mode', default_value='off',
            description="Gravity compensation: 'off', 'assist', or 'gravity_only'"),
        DeclareLaunchArgument('gravity_effort_scale', default_value='0.0',
            description='Gravity compensation effort scale'),
        DeclareLaunchArgument('active_real_joints', default_value='',
            description='Comma-separated joint names for real CAN I/O'),
        DeclareLaunchArgument('can0_interface', default_value='can0',
            description='CAN interface for joints 1-4'),
        DeclareLaunchArgument('can1_interface', default_value='can1',
            description='CAN interface for joints 5-7'),
        DeclareLaunchArgument('start_commander', default_value='true',
            description='Start joystick commander node'),
        DeclareLaunchArgument('start_joy', default_value='true',
            description='Start joy_node for gamepad input'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0',
            description='Joystick device path'),
        DeclareLaunchArgument('use_servo', default_value='false',
            description='Enable MoveIt Servo output in commander'),

        OpaqueFunction(function=launch_setup),
    ])
