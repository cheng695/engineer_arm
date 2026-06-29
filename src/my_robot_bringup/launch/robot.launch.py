import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler

from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware', default='false')
    gravity_compensation_mode = LaunchConfiguration('gravity_compensation_mode', default='off')
    active_real_joints = LaunchConfiguration('active_real_joints', default='')
    can0_interface = LaunchConfiguration('can0_interface', default='can0')
    can1_interface = LaunchConfiguration('can1_interface', default='can1')

    description_pkg = get_package_share_directory("my_robot_description")
    moveit_config_pkg = get_package_share_directory("my_robot_moveit_config")
    description_xacro_file = os.path.join(description_pkg, "urdf", "my_robot.urdf.xacro")
    initial_positions_file = os.path.join(moveit_config_pkg, "config", "initial_positions.yaml")

    robot_description_content = Command([
        'xacro ', description_xacro_file,
        ' use_mock_hardware:=', use_mock_hardware,
        ' gravity_compensation_mode:=', gravity_compensation_mode,
        ' active_real_joints:=', active_real_joints,
        ' can0_interface:=', can0_interface,
        ' can1_interface:=', can1_interface,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="my_robot_moveit_config")
        .robot_description(
            file_path=description_xacro_file,
            mappings={
                "use_mock_hardware": use_mock_hardware,
                "initial_positions_file": initial_positions_file,
                "gravity_compensation_mode": gravity_compensation_mode,
                "active_real_joints": active_real_joints,
                "can0_interface": can0_interface,
                "can1_interface": can1_interface,
            },
        )
        .robot_description_semantic(file_path="config/my_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ---- Nodes ----

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    bringup_pkg = get_package_share_directory("my_robot_bringup")

    # 加载控制增益（Python dict，非 ROS2 YAML 参数文件格式）
    control_gains_path = os.path.join(bringup_pkg, "config", "control_gains.yaml")
    with open(control_gains_path, "r") as f:
        control_gains = yaml.safe_load(f)

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(moveit_config_pkg, "config", "ros2_controllers.yaml"),
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
            moveit_config.to_dict(),
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
        parameters=[robot_description, moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
            description='Use mock hardware (simulation) if true'),
        DeclareLaunchArgument('gravity_compensation_mode', default_value='off',
            description="Gravity compensation: 'off', 'assist', or 'gravity_only'"),
        DeclareLaunchArgument('active_real_joints', default_value='',
            description='Comma-separated joint names for real CAN I/O'),
        DeclareLaunchArgument('can0_interface', default_value='can0',
            description='CAN interface for joints 1-4'),
        DeclareLaunchArgument('can1_interface', default_value='can1',
            description='CAN interface for joints 5-7'),

        robot_state_publisher,
        ros2_control_node,
        move_group_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        spawn_arm_controller_event,
        spawn_gripper_controller_event,
    ])
