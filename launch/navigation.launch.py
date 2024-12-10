import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of the a24 package
    a24_dir = get_package_share_directory('a24')
    config_dir = os.path.join(a24_dir, 'config')
    nav2_params_path = os.path.join(config_dir, 'nav2_params.yaml')

    # Path to the AMCL launch file
    amcl_launch_file = os.path.join(a24_dir, 'launch', 'amcl.launch.py')

    # Path to the navigation bringup launch file (from nav2_bringup)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    # Define the remapper node
    remapper_node = Node(
        package='a24',
        executable='remapper',
        name='remapper',
        output='screen'
    )

    # Include the AMCL launch file
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(amcl_launch_file),
        # If your amcl.launch.py expects arguments, you can supply them here
        launch_arguments={}
    )

    # Include the Nav2 navigation launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_path
        }.items()
    )

    # Return the LaunchDescription composed of all actions
    return LaunchDescription([
        remapper_node,
        amcl_launch,
        nav2_launch
    ])
