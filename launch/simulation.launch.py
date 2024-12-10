import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Declare launch arguments
    kitten_arg = DeclareLaunchArgument(
        'kitten', default_value='basic', description='A string argument for kitten'
    )
    
    # Retrieve the value of the kitten argument
    kitten_value = LaunchConfiguration('kitten')

    package_name = 'a24'
    this_dir = get_package_share_directory(package_name)
    
    # File paths with substitutions
    world_file = PathJoinSubstitution([
        this_dir,
        'worlds',
        kitten_value,
        TextSubstitution(text='.world')
    ])

    print(f"Static portion of world file path: {this_dir}/worlds/<kitten_value>.world")
    print(f"World file path will be: {world_file}")

    # print(f"World file path: {world_file}")
    # resolved_world_file = perform_substitutions(None, [world_file])
    # print(f"Resolved world file: {resolved_world_file}")

    gazebo_params_file = os.path.join(this_dir, 'gazebo', 'gazebo_params.yaml')
    rviz_config_file = PathJoinSubstitution([
        this_dir,
        'rviz',
        kitten_value,
        TextSubstitution(text='.rviz')
    ])

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
                get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
            )]
        ),
        launch_arguments={'use_sim_time': 'true', 'xacro_file_name': 'a24_sim.xacro'}.items()
    )

    # Include the Gazebo launch file, with the world file specified
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
            )]
        ),
        launch_arguments={
            'world': world_file,  # Pass the substitution directly
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'a24'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Add RViz2 node with the custom display configuration
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        # Declare the kitten argument
        kitten_arg,
        # Include nodes and launch files
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        # rviz2
    ])
