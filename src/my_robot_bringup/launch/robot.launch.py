import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share_description = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_share_description, 'urdf', 'my_robot.urdf.xacro')

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(robot_description, value_type=str)
            }]),

        # Add more bringup nodes here (sensor drivers, hardware interface, etc.)
    ])
