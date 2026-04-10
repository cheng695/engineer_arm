import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware', default='true')

    pkg_share_description = get_package_share_directory('my_robot_description')
    pkg_share_bringup = get_package_share_directory('my_robot_bringup')

    # Robot Description with use_mock_hardware flag
    robot_description_content = Command([
        'xacro ', os.path.join(pkg_share_description, 'urdf', 'my_robot.urdf.xacro'),
        ' use_mock_hardware:=', use_mock_hardware
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # RViz config
    rviz_config_path = os.path.join(pkg_share_bringup, 'config', 'rviz_config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='true',
            description='Use mock hardware (simulation) if true'),

        robot_state_publisher,

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'),
    ])
