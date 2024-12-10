import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    a24_dir = get_package_share_directory('a24')
    
    # Declare launch arguments
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='basic.world',
        description='Name of the world file to load in Gazebo'
    )
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value='basic.yaml',
        description='Name of the map file to load for navigation'
    )
    
    # Use LaunchConfiguration for dynamic substitution
    world_name = LaunchConfiguration('world')
    map_name = LaunchConfiguration('map')
    
    # Paths to the required files
    gazebo_launch_path = os.path.join(a24_dir, 'launch', 'simulation.launch.py')
    maps_dir = os.path.join(a24_dir, 'maps')
    rviz_dir = os.path.join(a24_dir, 'rviz')
    rviz_config_path = os.path.join(rviz_dir, 'amcl.rviz')

    map_file_path = PathJoinSubstitution([maps_dir, map_name])
    
    return LaunchDescription([
        # Declare launch arguments
        declare_world_arg,
        declare_map_arg,
        
        # Include the simulation launch file with the world argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={'world': world_name}.items()
        ),
        
        # Run the nav2_map_server with the specified map file and use_sim_time parameter
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_map_server', 'map_server',
                 '--ros-args', '-p', ['yaml_filename:=', map_file_path], '-p', 'use_sim_time:=true'],
            output='screen'
        ),
        
        # Bring up the lifecycle of the map_server
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
            output='screen'
        ),
        
        # Run the nav2_amcl with the use_sim_time parameter
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_amcl', 'amcl',
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen'
        ),
        
        # Bring up the lifecycle of amcl
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'amcl'],
            output='screen'
        ),
        
        # Start rviz2 with the specified configuration file
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        )
    ])
