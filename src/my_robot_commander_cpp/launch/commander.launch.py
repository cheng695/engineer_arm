import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    start_joy = LaunchConfiguration('start_joy', default='true')
    joy_dev = LaunchConfiguration('joy_dev', default='/dev/input/js0')
    joy_deadzone = LaunchConfiguration('joy_deadzone', default='0.05')
    joy_autorepeat_rate = LaunchConfiguration('joy_autorepeat_rate', default='100.0')
    joy_coalesce_interval = LaunchConfiguration('joy_coalesce_interval', default='0.01')

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
        remappings=[
            ('panda_arm_controller/joint_trajectory', 'arm_controller/joint_trajectory'),
            ('joint_trajectory', 'arm_controller/joint_trajectory'),
            ('/panda_arm_controller/joint_trajectory', '/arm_controller/joint_trajectory'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'start_joy',
            default_value='true',
            description='Launch joy_node together with commander.'),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path for joy_node.'),
        DeclareLaunchArgument(
            'joy_deadzone',
            default_value='0.05',
            description='joy_node deadzone.'),
        DeclareLaunchArgument(
            'joy_autorepeat_rate',
            default_value='100.0',
            description='joy_node autorepeat rate in Hz so held buttons keep publishing.'),
        DeclareLaunchArgument(
            'joy_coalesce_interval',
            default_value='0.01',
            description='joy_node coalesce interval in seconds.'),
        joy_node,
        commander_node
    ])
