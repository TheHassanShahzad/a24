import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
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

    # Declare launch arguments for world and map
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='home.world',
        description='Name of the world file to load'
    )
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value='home.yaml',
        description='Name of the map file to load'
    )

    # Use LaunchConfiguration to get the arguments
    world_name = LaunchConfiguration('world')
    map_name = LaunchConfiguration('map')

    # Define the remapper node
    remapper_node = Node(
        package='a24',
        executable='remapper',
        name='remapper',
        output='screen'
    )

    is_occupied_service = Node(
        package='a24',
        executable='is_occupied',
        name='is_occupied',
        output='screen'
    )

    coord_2_depth_service = Node(
        package='a24',
        executable='coord_2_depth',
        name='coord_2_depth',
        output='screen'
    )
    # Include the AMCL launch file and pass the world and map arguments
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(amcl_launch_file),
        launch_arguments={
            'world': world_name,
            'map': map_name
        }.items()
    )

    take_image_node = Node(
        package='a24',
        executable='take_image',
        name='take_image',
        output='screen'
    )
    
    # Include the Nav2 navigation launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_path
        }.items()
    )

    set_initial_pose_node = Node(
        package='a24',
        executable='set_initial_pose',
        name='set_initial_pose',
        output='screen'
    )

    go_to_goal_node = Node(
        package='a24',
        executable='go_to_goal',
        name='go_to_goal',
        output='screen'
    )

    # Return the LaunchDescription composed of all actions
    return LaunchDescription([
        declare_world_arg,
        declare_map_arg,
        remapper_node,
        is_occupied_service,
        coord_2_depth_service,
        take_image_node,
        amcl_launch,
        nav2_launch,
        set_initial_pose_node,
        go_to_goal_node
    ])