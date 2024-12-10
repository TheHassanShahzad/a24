import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    a24_dir = get_package_share_directory('a24')
    # nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    maps_dir = os.path.join(a24_dir, 'maps')
    # config_dir = os.path.join(a24_dir, 'config')
    rviz_dir = os.path.join(a24_dir, 'rviz')
    
    # Paths to the launch files and map file
    gazebo_launch_path = os.path.join(a24_dir, 'launch', 'simulation.launch.py')
    # navigation_launch_path = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    map_file_path = os.path.join(maps_dir, 'basic.yaml')
    rviz_config_path = os.path.join(rviz_dir, 'amcl.rviz')
    # nav2_params_path = os.path.join(config_dir, 'nav2_params.yaml')
    
    return LaunchDescription([
        # Include the maze_gazebo.launch.py launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        ),
        
        # Run the nav2_map_server with the specified map file and use_sim_time parameter
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '-p', f'yaml_filename:={map_file_path}', '-p', 'use_sim_time:=true'],
            output='screen'
        ),
        
        # Bring up the lifecycle of the map_server
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
            output='screen'
        ),
        
        # Run the nav2_amcl with the use_sim_time parameter
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_amcl', 'amcl', '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen'
        ),
        
        # Bring up the lifecycle of amcl
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'amcl'],
            output='screen'
        ),
        
        # Include the navigation_launch.py from nav2_bringup with use_sim_time and map_subscribe_transient_local parameters
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(navigation_launch_path),
        #     launch_arguments={
        #         'use_sim_time': 'true',
        #         'map_subscribe_transient_local': 'true',
        #         'params_file': nav2_params_path
        #     }.items()
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(navigation_launch_path),
        #     launch_arguments={
        #         'use_sim_time': 'true',
        #         'map_subscribe_transient_local': 'true'
        #     }.items()
        # ),
              
        # Start rviz2 with the specified configuration file
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()