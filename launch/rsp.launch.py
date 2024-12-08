import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    xacro_file_name_arg = DeclareLaunchArgument(
        'xacro_file_name',
        default_value='a24_sim.xacro',
        description='Name of the XACRO file to process'
    )

    # Retrieve the values of the launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_file_name = LaunchConfiguration('xacro_file_name')

    # Get the package share directory
    pkg_path = get_package_share_directory('a24')

    # Instead of processing the xacro file directly in Python, we use the Command substitution
    # This ensures the file name argument is properly resolved at runtime.
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([pkg_path, 'urdf', xacro_file_name])
    ])

    # Set parameters for the robot_state_publisher
    params = {
        'robot_description': robot_description,
        'use_sim_time': use_sim_time
    }

    # Create the robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Return the LaunchDescription including our new argument
    return LaunchDescription([
        use_sim_time_arg,
        xacro_file_name_arg,
        node_robot_state_publisher
    ])
