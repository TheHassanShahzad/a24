from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directories
    pkg_share = FindPackageShare('a24')
    slam_toolbox_share = FindPackageShare('slam_toolbox')

    # Launch files and config paths
    simulation_launch = PathJoinSubstitution([pkg_share, 'launch', 'simulation.launch.py'])
    mapper_params = PathJoinSubstitution([pkg_share, 'config', 'mapper_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'cartographer.rviz'])

    # Include simulation launch file
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include SLAM Toolbox launch file with params
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                slam_toolbox_share,
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'slam_params_file': mapper_params,
            'use_sim_time': 'true'
        }.items()
    )

    # Launch RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        simulation,
        slam,
        rviz
    ])
