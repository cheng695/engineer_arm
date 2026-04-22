import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Set arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware', default='false')
    gravity_compensation_mode = LaunchConfiguration('gravity_compensation_mode', default='off')
    active_real_joints = LaunchConfiguration('active_real_joints', default='')

    # Paths to xacro/config
    description_pkg = get_package_share_directory("my_robot_description")
    moveit_config_pkg = get_package_share_directory("my_robot_moveit_config")
    description_xacro_file = os.path.join(description_pkg, "urdf", "my_robot.urdf.xacro")
    initial_positions_file = os.path.join(moveit_config_pkg, "config", "initial_positions.yaml")

    # Build one robot description and share it across state publisher, ros2_control and MoveIt.
    robot_description_content = Command([
        'xacro ', description_xacro_file,
        ' use_mock_hardware:=', use_mock_hardware,
        ' gravity_compensation_mode:=', gravity_compensation_mode,
        ' active_real_joints:=', active_real_joints,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Load MoveIt configurations
    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="my_robot_moveit_config")
        .robot_description(
            file_path=description_xacro_file,
            mappings={
                "use_mock_hardware": use_mock_hardware,
                "initial_positions_file": initial_positions_file,
                "gravity_compensation_mode": gravity_compensation_mode,
                "active_real_joints": active_real_joints,
            },
        )
        .robot_description_semantic(file_path="config/my_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Controller Manager (ros2_control_node)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(get_package_share_directory("my_robot_moveit_config"), "config", "ros2_controllers.yaml"),
            {'use_sim_time': use_sim_time}
        ],
        output="screen",
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
    )

    # RViz
    rviz_config_path = os.path.join(get_package_share_directory("my_robot_bringup"), "config", "rviz_config.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time}
        ],
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
        condition=IfCondition(use_mock_hardware),
    )

    # Sequencing logic to prevent Controller Manager freeze
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware (simulation) if true'),

        DeclareLaunchArgument(
            'gravity_compensation_mode',
            default_value='off',
            description="Gravity compensation mode: 'off', 'assist', or 'gravity_only'"),

        DeclareLaunchArgument(
            'active_real_joints',
            default_value='',
            description='Comma-separated joint names that should use real CAN I/O; other joints use local fake feedback for full-stack debugging.'),
        
        robot_state_publisher,
        ros2_control_node,
        move_group_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        spawn_arm_controller_event,
        spawn_gripper_controller_event,
    ])
